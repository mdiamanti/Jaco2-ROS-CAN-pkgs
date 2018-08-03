# Jaco2-ROS-CAN-pkgs
Current repository includes the appropriate ROS packages that allow remote communication and control of Kinova Jaco<sup>2</sup> 6 DOF robotic arm via the use of CAN protocol. 

The ROS packages of current repository were developed in order to be used along with the `raspberry_can.project` of [Jaco2-CodeSys-projects](https://github.com/mdiamanti/Jaco2-CodeSys-projects) repository.

## File System
- `can_comm` ROS package provides all the functionality required for remote communication of Kinova Jaco<sup>2</sup> Gazebo simulation with any system that supports the use of [socketcan](https://github.com/linux-can/can-utils), an implementation of CAN protocols for Linux.    
- `gazebo_plugin` ROS package offers a ROS action server so as to control the robotic arm by executing trajectories of type [control_msgs/FollowJointTrajectoryAction](http://docs.ros.org/hydro/api/control_msgs/html/action/FollowJointTrajectory.html). More specifically, the robotic arm that is controlled through this package is Kinova Jaco<sup>2</sup> 6 DOF with three robot fingers, or else the model j2n6s300 as it is named in the required [kinova-ros](https://github.com/Kinovarobotics/kinova-ros#installation) package.

## Hardware Prerequisites
**Special attention** must be paid to the availability of the appropriate hardware that ensures the possibility of interconnecting our system with a CANbus. In addition, don't forget that the hardware that extends the capabilities of our system to support CANbus must be also compatible with ROS framework. A list of tested drivers and devices is given in the official ROS page [here](http://wiki.ros.org/socketcan_interface?distro=melodic). At the moment of writing, a safe enough solution is to use one of the above listed devices along with socketcan drivers. 

## Supported Versions and Hardware
- The configuration used was 64 bit Ubuntu 14.04 and ROS indigo. These packages may work with other configurations as well, but they have only been tested for the one recommended.
- Regarding the hardware that extends the capabilities of a pc to support CANbus, [PCAN-USB](https://www.peak-system.com/PCAN-USB.199.0.html?&L=1) from Peak System was used along with the socketcan drivers. In order to be able to use the hardware, you should first follow step by step the installation and hardware configuration instructions given by the provider.
   1. Installation of [Linux Device Drivers](https://www.peak-system.com/fileadmin/media/linux/index.htm).
   2. Hardware Configuration as reported in the [User Manual](https://www.peak-system.com/produktcd/Pdf/English/PCAN-USB_UserMan_eng.pdf).
   These packages may work with other hardware as well, but they have only been tested for the one recommended.

## Installation
The installation as follows was tested on Ubuntu 14.04 and ROS indigo.

#### Step 1. Install kinova-ros
Install [kinova-ros](https://github.com/Kinovarobotics/kinova-ros#installation) package, following the installation instructions that are analytically reported in the link.

#### Step 2. Install ros_canopen
Install [ros_canopen](https://github.com/ros-industrial/ros_canopen) package, choosing between the branch that corresponds to your ROS version (in this case indigo).
```
cd cd ~/catkin_ws/src
git clone https://github.com/ros-industrial/ros_canopen.git
```

#### Step 3. Install Jaco2-ROS-CAN-pkgs
Add the git repository in your catkin workspace:
```
cd ~/catkin_ws/src
git clone https://github.com/mdiamanti/JACO2-ROS-TCPIP-pkgs.git
```
Afterwards, compile your catkin workspace as usual:
```
cd ..
catkin_make
```
## How to use this repository
The `can_comm` and `gazebo_plugin` packages were developed in order to be used in combination and to control remotely the movement of the robotic arm. Otherwise, each package can be used separately after some small modifications to meet a different purpose. 

The `can_comm` node receives through the socketcan device the desired joint angular position of the arm, which is then sent to `gazebo_plugin` node that is responsible for executing the corresponding trajectory. The significant thing about this process is that an extra node is needed to expose CAN frames from socketcan to a ROS topic. For this reason, `socketcan_bridge` package from [ros_canopen](https://github.com/ros-industrial/ros_canopen) is also used. More information about its role is given in the official ROS page [here](http://wiki.ros.org/socketcan_bridge?distro=melodic).

An overview of the above described process is given by the following ROS graph, generated with the use of the ROS tool rqt_graph.

![can_rosgraph](https://user-images.githubusercontent.com/39567867/43530243-bfb9b44c-95b5-11e8-88d9-71f7507c367b.png)

The form of the joint angular position message that is received from `can_comm` node constitutes a string with the angular position of each joint in rads (variable of type double). Each angular position is separated from the others by spaces. Some examples are as follows:
```
"0.0 2.9 1.3 4.2 1.4 0.0"         -> home position of the arm
"3.14 3.14 3.14 3.14 3.14 3.14"   -> candle position of the arm
```
## Execution

#### Step 1. Launch j2n6s300 robotic arm in Gazebo
First of all, in order to use these packages you should launch j2n6s300 robotic arm in Gazebo using the launch file included in [kinova_gazebo](https://github.com/Kinovarobotics/kinova-ros/tree/master/kinova_gazebo) package:
```
roslaunch kinova_gazebo robot_launch.launch 
or
roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=j2n6s300
```

#### Step 2. Launch socketcan_bridge, can_comm and gazebo_plugin packages
If you wish to use `can_comm` and `gazebo_plugin` packages in combination, as described in the previous section, you can launch the .launch file included in `can_comm` package. In this way, `socketcan_bridge`, `can_comm` and `gazebo_plugin` nodes are launched:
```
roslaunch can_comm can_comm_launch.launch
```
Otherwise, you can launch each node separately in the following way:
```
rosrun gazebo_plugin joint_trajectory_client
and/or
rosrun socketcan_bridge socketcan_bridge_node
rosrun can_comm can_master_node
```
