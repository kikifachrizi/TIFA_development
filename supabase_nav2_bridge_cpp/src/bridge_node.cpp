#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <tuple>

#include <libpq-fe.h>  // libpq

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class SupabaseNav2BridgeCpp : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  SupabaseNav2BridgeCpp()
  : Node("supabase_nav2_bridge_cpp"), busy_(false)
  {
    // ---- Parameters ----
    db_url_           = this->declare_parameter<std::string>("db_url", "");
    table_            = this->declare_parameter<std::string>("table", "navigation_goals");
    default_frame_    = this->declare_parameter<std::string>("default_frame_id", "map");
    poll_hz_          = this->declare_parameter<double>("poll_hz", 2.0);
    nav2_namespace_   = this->declare_parameter<std::string>("nav2_namespace", "");
    goal_timeout_sec_ = this->declare_parameter<double>("goal_timeout_sec", 0.0); // 0 = no client-timeout

    if (db_url_.empty()) {
      RCLCPP_FATAL(get_logger(), "Parameter 'db_url' kosong.");
      throw std::runtime_error("db_url empty");
    }

    // ---- Connect DB ----
    conn_ = PQconnectdb(db_url_.c_str());
    if (PQstatus(conn_) != CONNECTION_OK) {
      RCLCPP_FATAL(get_logger(), "Gagal connect DB: %s", PQerrorMessage(conn_));
      throw std::runtime_error("DB connect failed");
    }
    exec_ok("set application_name = 'supabase_nav2_bridge_cpp'");

    // ---- Action client ----
    std::string action_name = nav2_namespace_.empty()
      ? "navigate_to_pose" : (nav2_namespace_ + "/navigate_to_pose");
    ac_ = rclcpp_action::create_client<NavigateToPose>(this, action_name);

    // ---- Poll timer ----
    double period = 1.0 / std::max(0.1, poll_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&SupabaseNav2BridgeCpp::tick, this));
  }

  ~SupabaseNav2BridgeCpp() override {
    if (conn_) { PQfinish(conn_); conn_ = nullptr; }
  }

private:
  // ---------- Utils ----------
  static std::tuple<double,double,double,double> yaw_to_quat(double yaw) {
    double half = yaw * 0.5;
    double qz = std::sin(half);
    double qw = std::cos(half);
    return {0.0, 0.0, qz, qw};
  }

  static std::string esc_quotes(const std::string &s) {
    std::string out; out.reserve(s.size() + 8);
    for (char c : s) out += (c=='\'') ? "''" : std::string(1, c);
    return out;
  }

  bool exec_ok(const std::string &sql) {
    PGresult *res = PQexec(conn_, sql.c_str());
    bool ok = (PQresultStatus(res) == PGRES_COMMAND_OK);
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "SQL error: %s", PQerrorMessage(conn_));
    }
    PQclear(res);
    return ok;
  }

  // ---------- DB: claim satu goal ----------
  // Skema tabel:
  // id SERIAL PK, status TEXT, goal_x DOUBLE, goal_y DOUBLE, goal_yaw DOUBLE, frame_id TEXT, meta JSONB, ...
  std::optional<std::tuple<int,double,double,double,std::string>> claim_next_goal() {
    // Atomic FIFO claim: READY -> SENDING
    std::string sql =
      "begin; "
      "with cte as ( "
      "  select id from " + table_ + " "
      "  where status = 'ready' "
      "  order by created_at "
      "  limit 1 "
      "  for update skip locked "
      ") "
      "update " + table_ + " t "
      "set status = 'sending', updated_at = now() "
      "from cte "
      "where t.id = cte.id "
      "returning t.id, t.goal_x, t.goal_y, t.goal_yaw, coalesce(t.frame_id, '" + default_frame_ + "');";

    PGresult *res = PQexec(conn_, sql.c_str());
    auto st = PQresultStatus(res);

    if (st != PGRES_TUPLES_OK && st != PGRES_COMMAND_OK) {
      RCLCPP_ERROR(get_logger(), "Claim SQL error: %s", PQerrorMessage(conn_));
      PQclear(res);
      exec_ok("rollback;");
      return std::nullopt;
    }

    if (PQntuples(res) == 0) {
      PQclear(res);
      exec_ok("rollback;");
      return std::nullopt;
    }

    int id          = std::stoi(PQgetvalue(res, 0, 0));
    double gx       = std::stod(PQgetvalue(res, 0, 1));
    double gy       = std::stod(PQgetvalue(res, 0, 2));
    double gyaw     = std::stod(PQgetvalue(res, 0, 3));
    std::string fid = PQgetvalue(res, 0, 4);

    PQclear(res);
    exec_ok("commit;");
    return std::make_optional(std::make_tuple(id, gx, gy, gyaw, fid));
  }

  void set_status(int id, const std::string &status, const std::string &result_text = "") {
    // Jika result_text != "", kita taruh ke meta.result (overwrite sederhana)
    std::string sql;
    if (result_text.empty()) {
      sql =
        "update " + table_ +
        " set status='" + esc_quotes(status) + "', updated_at=now() "
        " where id=" + std::to_string(id) + ";";
    } else {
      sql =
        "update " + table_ +
        " set status='" + esc_quotes(status) + "', "
        "     meta = coalesce(meta, '{}'::jsonb) || jsonb_build_object('result','" + esc_quotes(result_text) + "'), "
        "     updated_at=now() "
        " where id=" + std::to_string(id) + ";";
    }
    if (!exec_ok(sql)) exec_ok("rollback;");
  }

  // ---------- Loop ----------
  void tick() {
    if (busy_) return;

    if (!ac_->wait_for_action_server(0s)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 3000, "Nav2 action server belum siap...");
      return;
    }

    auto job = claim_next_goal();
    if (!job.has_value()) return;

    auto [id, x, y, yaw, frame] = job.value();

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = now();
    ps.header.frame_id = frame.empty() ? default_frame_ : frame;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    auto q = yaw_to_quat(yaw);
    ps.pose.orientation.x = std::get<0>(q);
    ps.pose.orientation.y = std::get<1>(q);
    ps.pose.orientation.z = std::get<2>(q);
    ps.pose.orientation.w = std::get<3>(q);

    NavigateToPose::Goal goal;
    goal.pose = ps;

    busy_ = true;
    current_id_ = id;
    set_status(id, "active");

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.feedback_callback = [this](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> &fb) {
      (void)fb; // optionally write progress to DB
    };
    options.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
      if (!handle) {
        RCLCPP_WARN(this->get_logger(), "Goal REJECTED oleh Nav2.");
        set_status(current_id_, "aborted", "rejected_by_nav2");
        busy_ = false;
        current_id_ = -1;
        timeout_timer_.reset();
        return;
      }
      goal_handle_ = handle;
      // Client-side timeout (opsional)
      if (goal_timeout_sec_ > 0.0) {
        timeout_timer_ = this->create_wall_timer(
          std::chrono::duration<double>(goal_timeout_sec_),
          std::bind(&SupabaseNav2BridgeCpp::maybe_cancel, this)
        );
      }
    };
    options.result_callback = [this](const GoalHandle::WrappedResult &result) {
      std::string status = "error";
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: status = "succeeded"; break;
        case rclcpp_action::ResultCode::CANCELED:  status = "canceled";  break;
        case rclcpp_action::ResultCode::ABORTED:   status = "aborted";   break;
        default:                                   status = "error";     break;
      }
      set_status(current_id_, status, "nav2_status=" + std::to_string(static_cast<int>(result.code)));

      busy_ = false;
      current_id_ = -1;
      goal_handle_.reset();
      timeout_timer_.reset();
    };

    ac_->async_send_goal(goal, options);
  }

  void maybe_cancel() {
    if (!busy_ || !goal_handle_) return;
    RCLCPP_WARN(get_logger(), "Client-side timeout terpicu -> kirim cancel ke Nav2.");
    ac_->async_cancel_goal(goal_handle_);
    // hasil final tetap akan ditangani di result_callback
  }

private:
  // params
  std::string db_url_, table_, default_frame_, nav2_namespace_;
  double poll_hz_{2.0};
  double goal_timeout_sec_{0.0};

  // db
  PGconn *conn_{nullptr};

  // action client
  rclcpp_action::Client<NavigateToPose>::SharedPtr ac_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  GoalHandle::SharedPtr goal_handle_;

  bool busy_;
  int  current_id_{-1};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SupabaseNav2BridgeCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
