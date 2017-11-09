# Pointing gestures object detection
  Assignment 11: Robot beliefs browsing using pointing gestures

![N|Solid](https://github.com/LBarranco/pointing_gestures_object_detection/blob/master/Pointing%20gestures%20object%20detection%20architecture.png?raw=true)

The goal of the assignment is to allow a human to ask a robot, using pointing gestures, to express its beliefs about a specific object in a table-top scenario, given a vision-based and an ontology-based software packages.

A ROS package composed by three node has been developed:


  - Tracker_node collects data of the skeleton of the person through a Kinect sensor in order to obtain the coordinates of the right hand and the right elbow.
Parameters: 
    - reference frame
    - v_ref
- Object_acquisition_node is a filter that allows a user to decide when the information coming from PITT reflects the real position of the objects on the table, and then publish it.
- Object_detection_node is the center of the architecture: it deals with joining the information coming from both the previous node and decides witch object has been pointed.


### Installation

- Install “openni_launch” using the shell command:
```sh
    $ sudo apt-get install ros-<ros_version>-openni-launch
```
- Install “openni_tracker” and change the frame from “openni_depth_frame” to “camera_link” (line 189).
http://wiki.ros.org/openni_tracker
- Download the ROS package “PITT” in your catkin workspace and use catkin_make to compile it. Set up the launch file “table_segmentation.launch” according to your needs and use it to run the package.
https://github.com/EmaroLab/pitt_geometric_tracking.git
https://github.com/EmaroLab/pitt_object_table_segmentation.git
https://github.com/EmaroLab/pitt_msgs.git
- Download the ROS package “pointing_gestures_object_detection” in your catkin workspace and use catkin_make to compile it. 
Use the launch file “pointing_gestures_object_detection.launch” to run the package.

The ROS package has mostly been developed on Ubuntu 14.04 and using ROS Indigo, but it has been tested even on Ubuntu 16.04 and ROS Kinetic.

### Launching
To launch the package use the following shell commands in this order:
```sh
    $ roslaunch openni_launch openni.launch
    $ roslaunch pitt_object_table_segmentation table_segmentation.launch
    $ roslaunch pointing_gestures_object_detection pointing_gestures_object_detection.launch 
```

### Usage

Overview of the constraints:
- Position of the arm used to point an object.
- Speed of movement of the pointing arm.
- Kinect and openni_tracker constraints.
- Position of the pointing user.
- Fixed position of the objects on the table.

Constraints in details:
- Arm (Fixed in tracker_node)
    - The arm that will be used for pointing must be the right.
    - The position of the arm should be extended: elbow and wrist must be in axes between them, forming a sort of straight line for those two points; in addition, the user hand must be closed to form a punch (or if you want to use the index finger to indicate, it must also be in axes with elbow and wrist).
    - The system will recognize that a user is actually pointing at an object (or scene) only when the person pauses to point at something: during the trajectory made by the arm to reach an object, the pointing is not taken into account.
    - After pointing at an object, the user must move the arm with at least a certain minimum speed to target another object.

- Kinect Sensors (Fixed in openni_tracker)
    - The Kinect that will be used to test the system will be two (one that looks at the table and the other that observes the user); both will be calibrated on the same reference frame “world", placed in the lower part of the body of the Baxter robot. All the spatial coordinates of our system will therefore referred to this base.
    - To track a user, the system uses the "openni_tracker" package. It detects the number of people in the scene and tries to calibrate the users that are in "psi pose”. Only the calibrated user can indicate something.
    - openni_tracker assigns an identifier (starting from 1) to each user who is pointing at the scene. Only the user 1 has the possibility of pointing (obviously after calibrating as explained in the previous point).

- Environment 
    - The number of people in front of the sensor does not affect the environment.
    - The person who intends to be tracked by Kinect and who wants to indicate, must consider two constraints: first of all the precision of the pointing depends on the distance from the Kinect (the further the user is, the worse the precision is); furthermore it is not allowed to touch the object to indicate it.

- Objects
    - The constraints concerning the objects are those imposed by PITT package.


### Credits

Authors:
- Lorenzo Barranco
- Luca Perazzo
- Lorenzo Sorzana

Date: 27/10/2017

### License
BSD (openni)
GNUv2.0	(PITT)	



