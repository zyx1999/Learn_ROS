/**
 * 该例程请求/spawn服务, 服务数据类型turtlesim::Spawn
 */

#include <ros/ros.h>
#include <turtlesim/Spawn.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtle_spawn");
    ros::NodeHandle node;

    // 发现/spawn服务后, 创建一个服务客户端, 连接名为/spawn的service
    ros::service::waitForService("/spawn");
    ros::ServiceClient add_turtle =
        node.serviceClient<turtlesim::Spawn>("/spawn");

    // 初始化turtlesim::Spawn的请求数据
    turtlesim::Spawn srv;
    srv.request.x = 2.0;
    srv.request.y = 2.0;
    srv.request.name = "turtle2";

    // 请求服务调用
    ROS_INFO("Call service to spawn turtle[x:%0.6f, y:%0.6f, name:%s]",
             srv.request.x, srv.request.y, srv.request.name.c_str());
    add_turtle.call(srv);

    // 显示服务调用结果
    ROS_INFO("Spawn turtle successfully [name:%s]", srv.response.name.c_str());
    return 0;
}