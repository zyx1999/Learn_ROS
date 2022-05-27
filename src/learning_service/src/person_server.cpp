/**
 * 该例程执行/show_person服务, 服务数据类型learning_service::Person
 */
#include <ros/ros.h>
#include "learning_service/Person.h"

// service回调函数, 输入参数req, 输出参数res
bool personCallback(learning_service::Person::Request &req,
                    learning_service::Person::Response &res) {
    // 显示请求数据
    ROS_INFO("Person: name:%s  age:%d  sex:%d", req.name.c_str(), req.age,
             req.sex);

    // 设置反馈数据
    res.result = "OK";
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "person_server");
    ros::NodeHandle node;
    
    // 创建名为/show_person的server, 注册回调函数personCallback
    ros::ServiceServer person_service = node.advertiseService("/show_person", personCallback);

    // 循环等待回调函数
    ROS_INFO("Ready to show person information.");
    ros::spin();
    return 0;
}