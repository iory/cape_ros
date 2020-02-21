# CAPE Ros Wrapper

This is Wrapper of CAPE Plane and Cylinder Extraction

## Install

Please Install ROS

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
cd ~/catkin_ws/src
git clone https://github.com/iory/cape_ros.git
cd ~/catkin_ws
rosdep install -y -r --from-paths src --ignore-src src
catkin build -DCMAKE_BUILD_TYPE=Release
source ~/catkin_ws/devel/setup.bash
```

## Run Sample

```
roslaunch cape_ros sample_cape.launch
```
