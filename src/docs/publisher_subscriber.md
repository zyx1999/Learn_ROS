# Publisher/Subscriber以及Topic消息机制的实现
## 目标
1. 定义一个Publisher，在person_publisher.cpp中实现
2. 定义一个Subscriber，在person_subscriber.cpp中实现
3. 定义一个Topic message，在Person.msg中实现
4. 实现话题消息通信
---
工程结构：最顶层目录为`~/catkin_ws/`
```
├── build/
├── devel/
└── src/
    ├─── learning_topic/
        ├── CMakeLists.txt
        ├── package.xml
        ├── include/
        ├── msg/
            ├── Person.msg
        └── src/
            ├── person_publisher.cpp
            └── person_subscriber.cpp
    └── CMakeLists.txt
```

## 0. 初始化
a. 创建功能包
```
cd ~/catkin_ws/src
catkin_create_pkg learning_topic roscpp rospy std_msgs geometry msgs_turtlesim
```
## 1. Topic Message
a. 定义话题消息——创建`Person.msg`文件（添加如下内容）
```
string name
uint8 sex
uint8 age

uint8 unknown = 0
uint8 male    = 1
uint8 female  = 2
```
---
b. 在`package.xml`中添加功能包依赖
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```
---
c. 在`learning_topic/CMakeLists.txt`中添加编译选项
```
find_package(
    ...
    ...
    message_generation
)
add_message_files(FILES Person.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(
    ...
    ...
    message_runtime
)
```
---
d. 编译生成相关语言文件
```
cd ~/catkin_ws/
# 将会在devel/include/learning_topic/下生成Person.h文件
catkin_make
```
## 2. Publisher实现
a. 实现发布者——创建`person_publisher.cpp`文件
1. 初始化ROS节点
2. 向ROS Master注册节点信息（话题名、消息类型）
3. 创建消息数据
4. 发布消息
---
b. 配置`learning_topic/CMakeLists.txt`
```
# 生成可执行文件
add_executable(person_publisher src/person_publisher.cpp)
# 设置链接库
target_link_libraries(person_publisher ${catkin_LIBRARIES})
# 添加依赖
add_dependencies(person_publisher ${PROJECT_NAME}_generate_message_cpp)
```

## 3. Subscriber实现
a. 实现订阅者——创建`person_subscriber.cpp`文件
1. 初始化ROS节点
2. 订阅需要的话题
3. 定义处理消息的回调函数
4. 循环等待话题消息，接收到消息后进入回调函数
5. 在回调函数中完成消息处理
---
b. 配置`learning_topic/CMakeLists.txt`
```
# 生成可执行文件
add_executable(person_subscriber src/person_subscriber.cpp)
# 设置链接库
target_link_libraries(person_subscriber ${catkin_LIBRARIES})
# 添加依赖
add_dependencies(person_subscriber ${PROJECT_NAME}_generate_message_cpp)
```
## 4. 编译并运行

```
cd ~/catkin_ws/
catkin_make
source devel/setup.bash

# terminal 1
roscore

# terminal 2
rosrun learning_topic person_publisher

# terminal 3
rosrun learning_topic person_subscriber
```
注：publisher和subscriber编译生成的可执行文件在`devel/lib/learning_topic/`