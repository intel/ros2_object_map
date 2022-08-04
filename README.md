DISCONTINUATION OF PROJECT.

This project will no longer be maintained by Intel.

Intel has ceased development and contributions including, but not limited to, maintenance, bug fixes, new releases, or updates, to this project. 

Intel no longer accepts patches to this project.

If you have an ongoing need to use this project, are interested in independently developing it, or would like to maintain patches for the open source software community, please create your own fork of this project. 
# ros2_object_map

## 1 Introduction
ros2_object_map is ROS2 package which designes to mark tag of objects on map when SLAM. It uses [ros2_object_analytics](https://github.com/intel/ros2_object_analytics) for object detection.

![Architecture of Object Map](https://github.com/intel/ros2_object_map/blob/master/object_map/object_map/ObjectMap.PNG "architecture of object map")

## 2 Prerequisite
  * [ros2_object_analytics](https://github.com/intel/ros2_object_analytics) installed

## 3 Build Dependencies
  * [object_analytics_msgs](https://github.com/intel/ros2_object_analytics)
  * [object_msgs](https://github.com/intel/ros2_object_msgs)

## 4 Building
  ```bash
  cd ~/ros2_ws/src
  git clone https://github.intel.com/otc-rse/ros2_object_map.git
  cd ..
  ament build --only-packages object_map object_map_msgs
  source install/local_setup.bash
  ```

## 5 Running the demo

###  Step1: Launch Realsense Camera
```
[terminal 1] 
source ~/ros2_ws/install/local_setup.bash
realsense_ros2_camera
```

### Step2: Launch object_analytics
```
[terminal 2]
source ~/ros2_ws/install/local_setup.bash
launch `ros2 pkg prefix object_analytics_launch`/share/object_analytics_launch/launch/analytics_movidius_ncs.py
```

### Step3: Launch ros2_object_map
```
[terminal 3]
source source ~/ros2_ws/install/local_setup.bash
ros2 run object_map object_map_node
```

## 6 View in ROS Rviz

### Step4: Launch ROS1 roscore
```
[terminal 4]
source /opt/ros/kinetic/setup.bash
roscore
```

### Step5: Launch ROS1 bridge
```
[terminal 5]
source /opt/ros/kinetic/setup.bash
source ros2_ws/install/local_setup.bash
ros2 run ros1_bridge dynamic_bridge

```

### Step6: Launch ROS1 Rviz
```
[terminal 6]
source /opt/ros/kinetic/setup.bash
roslaunch turtlebot_rviz_launchers view_robot.launch

within rviz gui, click "Add", and select "MarkerArray", then input "/object_map/Markers" into "Marker Topic"
```

## 6 Interface
### 6.1 Topic
  * ```/object_map/Markers``` : Publish MarkerArray on RVIZ
  * ```/object_map/map_save``` : Subscribe map_save topic to save object maps
  * ```/movidius_ncs_stream/detected_objects```: Subscribe ObjectsInBoxes from object_analytics
  * ```/object_analytics/tracking```: Subscribe TrackedObjects from object_analytics
  * ```/object_analytics/localization```: Subscribe ObjectsInBoxes3D from object_analytics

### 6.2 Save object map
```
ros2 topic pub --once /object_map/map_save std_msgs/Int32 -1

```
## 7 Known Issues
### 1   Map tag cannot be correctly displayed in Rviz while robot is moving

reason: tf2 python api is not supported in ROS2 currrently

next step: will implement it while tf2-python api is ready in ROS2  

### 2   Configure File is not supported 

reason: yaml configure file and dynamic configure file are not supported in ROS2 currently

next step: will implement it while it is ready in next release of ROS2


###### *Any security issue should be reported using process at https://01.org/security*
