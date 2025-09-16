#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

void msgcallbck(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("New Msg %s", msg->data.c_str());
}

int main(int argc, char **argv){
    ros::init(argc,argv,"subcpp");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter",10, msgcallbck);
    ros::spin();

    return 0;
}