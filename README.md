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