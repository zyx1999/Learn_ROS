#Client-Server通信模式
### 目标
编程实现Client-Server通信模式, 并自定义消息数据
1. 定义Server端, 文件`person_server.cpp`实现
2. 定义Client端, 文件`person_client.cpp`实现
3. 定义服务数据, 文件`Person.srv`实现

## 0. 创建功能包
```
cd ~/catkin/src
catkin_create_pkg learning_service roscpp rospy std_msgs geometry_msgs
```
## 1. 定义服务数据
a. 定义srv文件
```
cd ~/catkin/src/learning_service/
mkdir srv
cd srv
touch Person.srv
```
`Person.srv`相比于`Person.msg`略有不同，不同之处在于它分为两个部分。以`---`为分隔，其上为`Request`的数据结构，其下为`Response`的数据结构。
`Person.src`文件内容如下：
```
string name
uint8 age
uint8 sex

uint8 unknown = 0
uint8 male    = 1
uint8 female  = 2
---
string result
```
---
b. 在`package.xml`中添加功能包依赖
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```
---
c. 在`CMakeLists.txt`中添加编译选项
```
find_package(
    ...
    ...
    message_generation
)
add_service_files(FILES Person.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(
    ...
    ...
    message_runtime
)
```
---
d. 编译生成3个`.h`头文件
```
cd ~/catkin/
catkin_make
```
## 2. 实现Server
a. 创建`person_server.cpp`文件
1. 初始化ROS节点
2. 创建Server实例
3. 创建回调函数
4. 循环等待服务请求，进入回调函数
5. 在回调函数中完成服务处理，返回应答数据
---
b. 配置`learning_service/CMakeLists.txt`
```
# 生成可执行文件
add_executable(person_server src/person_server.cpp)
# 设置链接库
target_link_libraries(person_server ${catkin_LIBRARIES})
# 添加依赖
add_dependencies(person_server ${PROJECT_NAME}_gencpp)
```

## 3. 实现Client
a. 创建`person_client.cpp`文件
1. 初始化ROS节点
2. 创建一个Client实例
3. 发布服务请求数据
4. 等待Server处理之后的应答结果
---
b. 配置`learning_service/CMakeLists.txt`
```
# 生成可执行文件
add_executable(person_client src/person_client.cpp)
# 设置链接库
target_link_libraries(person_client ${catkin_LIBRARIES})
# 添加依赖
add_dependencies(person_client ${PROJECT_NAME}_gencpp)
```
## 4. 编译运行
```
cd ~/catkin_ws
catkin_make

# terminal 1
roscore

# terminal 2
rosrun learning_service person_server

# terminal 3
rosrun learning_service person_client
```