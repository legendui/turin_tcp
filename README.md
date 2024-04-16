# 图灵机器人ROS2工具包

### ROS2中间件项目概述

该包通过TCP协议与图灵机器人进行实时通信。它包括三个主要部分：TCP连接节点（`TCPConnectionNode`），关节状态发布器（`JointStatePublisher`），和数据监听器（`JointStateListener`）。这些组件共同工作，可以从图灵机器人获取关节状态数据，处理这些数据，并在ROS2系统中发布它们。

### 主要功能

- **TCP连接节点**：负责建立和维护与服务器的TCP连接，并发送及接收数据。
- **关节状态发布器**：定期向服务器发送请求，获取关节数据，并将接收到的数据转换为ROS2可以处理的格式。
- **数据监听器**：监听来自TCP连接节点的数据，处理XML格式的关节数据，并将其发布到ROS2话题上。

### 环境依赖

项目运行基于以下环境：
- ROS2 humble
- Python 3.8 或更高版本
- 操作系统推荐使用 Ubuntu 20.04

### 安装步骤

1. **安装ROS2 humble**：
   - 根据官方文档[安装ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)。
2. **创建工作空间**：
   - 打开终端，执行以下命令：
     ```bash
     mkdir -p ~/ros2_ws/src
     cd ~/ros2_ws
     colcon build
     source install/setup.bash
     ```
3. **克隆项目仓库**：
   - 将项目仓库克隆到`src`文件夹中：
     ```bash
     cd ~/ros2_ws/src
     git clone https://github.com/legendui/turin_tcp.git
     ```
4. **构建项目**：
   - 返回工作空间根目录，执行构建：
     ```bash
     cd ~/ros2_ws
     colcon build
     source install/setup.bash
     ```

### 运行项目

- 启动TCP连接节点：
  ```bash
  ros2 run turin_tcp tcp_connection_node
  //改变ip和端口地址
  ros2 run turin_tcp tcp_connection_node --ros-args -p server_ip:=192.168.1.5 -p server_port:=8527

  ```
- 启动关节状态发布器：
  ```bash
  ros2 run turin_tcp joint_state_publisher
  //改变关机角想要监听topic
  ros2 run turin_tcp joint_state_publisher --ros-args -p joint_state_topic:= joint_state -p update_interval:= 1 //单位是秒
  ```
- 启动图灵关节角监听器：
  ```bash
  ros2 run turin_tcp joint_state_listener
  //改变关节角监听和发布速度
  ros2 run turin_tcp joint_state_listener --ros-args -p update_interval:=1 //单位是秒
  ```
 
### 项目结构

- `/src`：包含所有源代码文件。
- `/launch`：包含ROS2启动脚本。
- `/config`：包含配置文件。
