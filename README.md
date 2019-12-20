# ur5_robotiq_meta_package
Packages for combining the UR5 arm and Robotiq 2F-85 hand.

## Purpose
This meta package contains references to the Robotiq 2G-85 and UR5 packages and mounts the Robotiq 2G-85 hand on the UR5 arm into a single urdf.

## Usage

Run the following commands to add this package and relevant subpackages to your catkin workspace denoted as <catkin_ws>:
```bash
cd <catkin_ws>/src
git clone git@github.com:CRLab/ur5_robotiq_meta_package.git
cd ur5_robotiq_meta_package
gitman install
cd ../..
source /opt/ros/kinetic/setup.bash
catkin build
```

## Running in Simulation

In order to bring up the arm and hand together with moveit, run:
```bash
roslaunch ur5_robotiq_moveit_config demo.launch
```

To visualize and verify the combined urdf:
```bash
roslaunch ur5_robotiq_description test_ur5_robotiq.launch
```


## Running on Real Robot

### UR5 Arm Only

Roslaunch:

```bash
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.0.102
```

Start the external control program on the robot's teach-pendant according to instructions [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#quick-start). This allows the robot to take commands from your machine.

MoveIt!
Add `<remap from="/follow_joint_trajectory" to="/scaled_pos_traj_controller/follow_joint_trajectory"/>` to ur5_moveit_planning_execution.launch
```bash
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```

### Robotiq Hand Only

```bash
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py
```

Using Action service
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0 Robotiq2FGripperRobotInput:=gripper/input Robotiq2FGripperRobotOutput:=gripper/output
roslaunch robotiq_2f_gripper_action_server robotiq_2f_gripper_action_server.launch


### Running both Arm and Hand Together
- Turn on Robot
- Plug in hand to USB and Run `sudo chmod 666 /dev/ttyUSB0`. This assumes `/dev/ttyUSB0` port was assigned to the hand connection. Confirm with `ls /dev/ttyUSB*`
- Roslaunch to bring up both arm and hand

```bash
roslaunch ur5_robotiq_bringup ur5_robotiq_bringup.launch
```
- On the Teach Pendant, start the external control program according to [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#quick-start)

- Start Moveit!
```bash
roslaunch ur5_robotiq_moveit_config ur5_robotiq_moveit_planning_execution.launch
```

- Start Rviz
```bash
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```