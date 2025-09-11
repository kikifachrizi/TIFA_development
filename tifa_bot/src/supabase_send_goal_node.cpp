#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <tuple>

#include <libpq-fe.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

static std::tuple<double,double,double,double> yaw_to_quat(double yaw_rad){
  const double h = yaw_rad * 0.5;
  return {0.0, 0.0, std::sin(h), std::cos(h)};
}
static std::string esc(const std::string &s){ std::string o; o.reserve(s.size()+8); for(char c: s) o += (c=='\'')?"''":std::string(1,c); return o; }

class SupabaseSendGoalNode : public rclcpp::Node {
public:
  SupabaseSendGoalNode() : Node("supabase_send_goal_node") {
    db_url_          = declare_parameter<std::string>("db_url", "");
    table_           = declare_parameter<std::string>("table", "navigation_goals");
    default_frame_   = declare_parameter<std::string>("default_frame_id", "map");
    poll_hz_         = declare_parameter<double>("poll_hz", 2.0);
    nav2_ns_         = declare_parameter<std::string>("nav2_namespace", "");
    goal_timeout_sec_= declare_parameter<double>("goal_timeout_sec", 0.0); // 0 = no timeout

    if (db_url_.empty()) throw std::runtime_error("param db_url kosong");

    // DB connect
    conn_ = PQconnectdb(db_url_.c_str());
    if (PQstatus(conn_) != CONNECTION_OK) throw std::runtime_error(PQerrorMessage(conn_));
    exec_ok("set application_name='supabase_send_goal_node'");

    // Action client
    const std::string action_name = nav2_ns_.empty() ?
      "navigate_to_pose" : (nav2_ns_ + "/navigate_to_pose");
    ac_ = rclcpp_action::create_client<NavigateToPose>(this, action_name);

    // Timer loop
    const double period = 1.0 / std::max(0.1, poll_hz_);
    timer_ = create_wall_timer(std::chrono::duration<double>(period),
             std::bind(&SupabaseSendGoalNode::tick, this));
  }

  ~SupabaseSendGoalNode() override { if (conn_) PQfinish(conn_); }

private:
  bool exec_ok(const std::string &sql){
    PGresult *res = PQexec(conn_, sql.c_str());
    const bool ok = (PQresultStatus(res)==PGRES_COMMAND_OK);
    if(!ok) RCLCPP_ERROR(get_logger(),"SQL err: %s", PQerrorMessage(conn_));
    PQclear(res); return ok;
  }

  // Claim 1 row ready -> sending ; return (id,x,y,yaw,frame)
  std::optional<std::tuple<int,double,double,double,std::string>> claim_one(){
    const std::string sql =
      "begin; with cte as (select id from public."+table_+" "
      "where status='ready' order by created_at limit 1 for update skip locked) "
      "update public."+table_+" t set status='sending', updated_at=now() "
      "from cte where t.id=cte.id "
      "returning t.id, t.goal_x, t.goal_y, t.goal_yaw, coalesce(t.frame_id,'"+default_frame_+"');";
    PGresult *res = PQexec(conn_, sql.c_str());
    auto st = PQresultStatus(res);
    if (st!=PGRES_TUPLES_OK && st!=PGRES_COMMAND_OK){ PQclear(res); exec_ok("rollback;"); return std::nullopt; }
    if (PQntuples(res)==0){ PQclear(res); exec_ok("rollback;"); return std::nullopt; }

    int id = std::stoi(PQgetvalue(res,0,0));
    double x = std::stod(PQgetvalue(res,0,1));
    double y = std::stod(PQgetvalue(res,0,2));
    double yaw = std::stod(PQgetvalue(res,0,3));
    std::string frame = PQgetvalue(res,0,4);

    PQclear(res); exec_ok("commit;");
    return std::make_optional(std::make_tuple(id,x,y,yaw,frame));
  }

  void set_status(int id, const std::string &st, const std::string &result=""){
    std::string sql;
    if (result.empty()){
      sql = "update public."+table_+" set status='"+esc(st)+"', updated_at=now() where id="+std::to_string(id)+";";
    } else {
      sql = "update public."+table_+" set status='"+esc(st)+"', "
            "meta=coalesce(meta,'{}'::jsonb)||jsonb_build_object('result','"+esc(result)+"'), "
            "updated_at=now() where id="+std::to_string(id)+";";
    }
    if(!exec_ok(sql)) exec_ok("rollback;");
  }

  void tick(){
    if (busy_) return;
    if (!ac_->wait_for_action_server(0s)) return;

    auto job = claim_one();
    if (!job) return;

    auto [id,x,y,yaw,frame] = *job;

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = now();
    ps.header.frame_id = frame.empty()? default_frame_ : frame;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = 0.0;
    auto q = yaw_to_quat(yaw);
    ps.pose.orientation.x = std::get<0>(q);
    ps.pose.orientation.y = std::get<1>(q);
    ps.pose.orientation.z = std::get<2>(q);
    ps.pose.orientation.w = std::get<3>(q);

    NavigateToPose::Goal goal; goal.pose = ps;

    busy_ = true; current_id_ = id; set_status(id,"active");

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    opts.goal_response_callback = [this](GoalHandle::SharedPtr gh){
      if (!gh){ set_status(current_id_,"aborted","rejected_by_nav2"); busy_=false; current_id_=-1; return; }
      goal_handle_ = gh;
      if (goal_timeout_sec_>0.0){
        timeout_timer_ = create_wall_timer(std::chrono::duration<double>(goal_timeout_sec_),
                          std::bind(&SupabaseSendGoalNode::maybe_cancel, this));
      }
    };
    opts.feedback_callback = [](GoalHandle::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback>&){};
    opts.result_callback = [this](const GoalHandle::WrappedResult &res){
      std::string st="error";
      switch(res.code){
        case rclcpp_action::ResultCode::SUCCEEDED: st="succeeded"; break;
        case rclcpp_action::ResultCode::CANCELED:  st="canceled";  break;
        case rclcpp_action::ResultCode::ABORTED:   st="aborted";   break;
        default: st="error"; break;
      }
      set_status(current_id_, st, "nav2_status="+std::to_string(static_cast<int>(res.code)));
      busy_=false; current_id_=-1; goal_handle_.reset(); timeout_timer_.reset();
    };

    ac_->async_send_goal(goal, opts);
  }

  void maybe_cancel(){
    if (!busy_ || !goal_handle_) return;
    ac_->async_cancel_goal(goal_handle_);
  }

  // params
  std::string db_url_, table_, default_frame_, nav2_ns_;
  double poll_hz_{2.0}, goal_timeout_sec_{0.0};

  // db
  PGconn *conn_{nullptr};

  // action
  rclcpp_action::Client<NavigateToPose>::SharedPtr ac_;
  rclcpp::TimerBase::SharedPtr timer_, timeout_timer_;
  GoalHandle::SharedPtr goal_handle_;

  // state
  bool busy_{false};
  int current_id_{-1};
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SupabaseSendGoalNode>());
  rclcpp::shutdown();
  return 0;
}
