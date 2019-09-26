# EEHMD_v1
1st version of the EEHMD database and associated ROS code

## Description

The Electromyography and Enhanced Hand Motion Description (EEHMD) database contains sEMG (surface electromyography) signals and motion capture of human hand. The sEMG signals are captured by a Myo armband. The motion acquisition is achieved with a Leap Motion stereoscopic camera. 

12 subjects can be found in the database. For each subject, 5 records were performed :

- Record 0:
The subject was asked to stay at rest

- Record 1: 
The subject was asked to move with slow movement. The time to get the hand from open to close is about 2 seconds. Each subject was asked to perform the most various gestures he can do.

- Record 2: 
The request is the same as for the record 1 but with a higher velocity.

- Record 3: 
The subject moves his fingers while trying to keep his wrist at rest.

- Record 4: 
Six gestures were repeated several times : Flexion / extension and deviation radial / ulnar of the wrist, opening / closing of the hand. 

In the database folder, each file is named "subject_i.j.bag" where i is the subject number and j the record number.

## ROS implementation

The database was created with the Robot Operating System (ROS). The Leap Motion and Myo armband wrote several topics simultaneously recorded in rosbag files. (Rosbag is a set of tools for recording from and playing back to ROS topics. It is intended to be high performance and avoids deserialization and reserialization of the messages.)

List of topics: 

For the Myo armband: 
- **myo_arm** tells which arm and direction of X axis is worn.
- **myo_EmgArray** sends an 8 values vectors from 0 to 2048 representing the activation of motor unit action potentials and does not translate to volts.
- **myo_gest** is the result of the classification of gestures realized by the myo firm software. 
- **myo_gest_str** is the same with names of gestures instead of labels
- **myo_imu** contains the standard IMU message with quaternion pose, accelerometer and gyro axes.
- **myo_ori** is a vector containing roll, pitch and yaw in rad.
- **myo_ori_deg** is the same as myo\_ori in degrees.

For the Leap Motion:
- **Leapmotion_raw** sends a Leapmsg message (see content in ros_eehmd/msg/leapmsg.msg)

## How to use it

Rosbag can be play with the command line:
```
rosbag play subject_i.j.bag
```
Once built, the ros_eehmd package proposes 4 nodes:

##### myo_subscriber_node
This node is an example of how to subscribe the Myo EMG data. It subscribes the **myo_EmgArray** topic and writes the EMG vector in the command promp as it is sent by the topic.

##### myo_subscriber_node
This node is an example of how to subscribe the Leap Motion data. It subscribes the **Leapmotion_raw** topic and writes the x coordinate of thumb metacarpal bone in the command promp.

##### joint_coordinates_publisher
This code subscribes the **leapmotion_raw** topic and publishes the corresponding joint coordinates in the **joint_coordinates_from_leap** topic. The header is copied for a future synchronization.

##### ros_to_csv
This node subscribes to the **Leapmotion_raw** and **myo_EmgArray** topic, computes the corresponding joint coordinates and temporarily sychronizes "EmgArray" and "joint_coordinates" messages. Finally, both "EmgArray" and "joint_coordinates" can be save in a csv file. To do so, set the file name in the "file_name" ros parameter and set the "recording" flag parameter to true. The output files are "*file_name*_input.csv" with the EMG data and "*file_name*_target.csv" with the joint coordinates. Each row of one file is temporarily matching the same row in the other file.

For example, once the node is running: 
```
rosparam set file_name test_name
rosparam set recording true
```
To stop writting csv, set the "recording" parameter to false.
