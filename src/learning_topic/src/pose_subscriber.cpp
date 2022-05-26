#include <ros/ros.h>
#include "turtlesim/Pose.h"

void poseCallback(const turtlesim::Pose::ConstPtr& msg){
    ROS_INFO("Turtle pose: x:%0.6f, y:%0.6f", msg->x, msg->y);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "pose_subscriber");
    ros::NodeHandle n;
    
    // 创建一个Subscriber, 订阅名为/turtle1/pose的topic, 10为队列长度, 注册回调函数poseCallback
    // 一旦有消息进来, 就会调用poseCallback
    ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 10, poseCallback);

    // 等待循环回调函数, 是一个死循环, 循环等待消息
    ros::spin();
    return 0;
}