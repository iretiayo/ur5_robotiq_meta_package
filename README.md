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

According to instructions [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#quick-start), start the external control program on the robot's teach-pendant and then roslaunch:

### UR5 Arm Only
```bash
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.0.102
```

MoveIt!
Add `<remap from="/follow_joint_trajectory" to="/scaled_pos_traj_controller/follow_joint_trajectory"/>` to ur5_moveit_planning_execution.launch
```bash=
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```

### Robotiq Hand Only

```bash
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py
```


rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0 Robotiq2FGripperRobotInput:=gripper/input Robotiq2FGripperRobotOutput:=gripper/output
roslaunch robotiq_2f_gripper_action_server robotiq_2f_gripper_action_server.launch
