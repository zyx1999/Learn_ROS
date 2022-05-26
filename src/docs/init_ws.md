# 创建工作空间与功能包
## 创建工作空间
A.创建工作空间
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```
B.编译工作空间
```
cd ~/catkin_ws/
catkin_make
```
C.设置环境变量
```
source devel/setup.bash
```
D.检查环境变量
```
echo $ROS_PACKAGE_PATH
```

## 创建功能包
A.创建功能包
```
cd ~/catkin_ws/src
catkin_create_pkg <package_name> [depend1] [depend2] ...
(i.e. catkin_create_pkg learning_topic std_msgs rospy roscpp)
```
B.编译功能包
```
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
```