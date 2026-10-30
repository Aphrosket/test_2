# 第二题：全向小车仿真与雷达建图

姓名：王晨航  
学号：2024213356  
环境：Ubuntu 22.04.5、ROS2 Humble、Gazebo Fortress。  
ROS2包名：`omni_project`。

## 实现内容

- 在Gazebo中创建小车和含长方体、圆柱体障碍的地图，使用WASDQE全向控制。
- 根据已知地图进行A*规划、路径平滑和闭环跟踪，使小车自动从A运动到B。
- 使用模拟雷达与slam_toolbox扫描建图，利用RViz显示并保存地图。

默认A=(-3,-2)、B=(3,2)，单位米。模型采用理想全向速度驱动。已有A→B轨迹及保存的SLAM地图；完整无碰撞与地图质量仍需结合实际演示验收。附加Nav2导航已有代码，但当前缺少依赖，不列入下面的基础演示。

## 启动说明

以下按已经编译的当前工程 `~/recruitment_ws/src/robot_task` 编写。切换小项前，先停止上一项的控制、RViz和仿真，重新打开场景使小车回到A。键盘控制、自动跟踪和Nav2不要同时运行。

## （1）WASDQE全向控制：打开2个终端

### 终端1（打开Gazebo地图和小车，保持运行）

```bash
source /opt/ros/humble/setup.bash
source ~/recruitment_ws/install/setup.bash
env -u WAYLAND_DISPLAY QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 ros2 launch omni_project sim.launch.py
```

### 终端2（键盘控制小车）

```bash
source /opt/ros/humble/setup.bash
source ~/recruitment_ws/install/setup.bash
ros2 run omni_project teleop
```

点击终端2再按键：W/S前后，A/D左右平移，Q/E旋转，空格停车，X退出。

## （2）已知地图自动A到B：打开3个终端

### 终端1（打开Gazebo地图和小车，保持运行）

```bash
source /opt/ros/humble/setup.bash
source ~/recruitment_ws/install/setup.bash
env -u WAYLAND_DISPLAY QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 ros2 launch omni_project sim.launch.py
```

### 终端2（自动规划并跟踪到B）

```bash
source /opt/ros/humble/setup.bash
source ~/recruitment_ws/install/setup.bash
ros2 run omni_project follow --ros-args -p use_sim_time:=true
```

### 终端3（RViz查看规划路径和雷达）

```bash
source /opt/ros/humble/setup.bash
source ~/recruitment_ws/install/setup.bash
rviz2 -d ~/recruitment_ws/src/robot_task/config/view.rviz --ros-args -p use_sim_time:=true
```

RViz的Fixed Frame设为odom。等待终端2提示“到达目标，已停车”。

## （3）雷达建图：打开4个终端

### 终端1（打开Gazebo地图和小车，保持运行）

```bash
source /opt/ros/humble/setup.bash
source ~/recruitment_ws/install/setup.bash
env -u WAYLAND_DISPLAY QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 ros2 launch omni_project sim.launch.py
```

### 终端2（启动SLAM建图，保持运行）

```bash
source /opt/ros/humble/setup.bash
source ~/recruitment_ws/install/setup.bash
ros2 launch omni_project mapping.launch.py
```

### 终端3（RViz显示实时地图）

```bash
source /opt/ros/humble/setup.bash
source ~/recruitment_ws/install/setup.bash
rviz2 -d ~/recruitment_ws/src/robot_task/config/view.rviz --ros-args -p use_sim_time:=true
```

RViz设置：Fixed Frame=map，Map话题=/map，LaserScan话题=/scan。

### 终端4（键盘开车扫描并保存地图）

```bash
source /opt/ros/humble/setup.bash
source ~/recruitment_ws/install/setup.bash
ros2 run omni_project teleop
```

低速绕地图和障碍行驶，再回起点。按空格停车、X退出键盘，保持终端1、2运行，在终端4继续输入：

```bash
mkdir -p ~/omni_results/maps
ros2 run nav2_map_server map_saver_cli -f ~/omni_results/maps/slam_map_new --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
xdg-open ~/omni_results/maps/slam_map_new.pgm
```


关闭顺序：先停车，再Ctrl+C关闭控制、SLAM和RViz，最后Ctrl+C关闭仿真。
