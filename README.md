# CAP_SD_Simulation

## Table of contents
- [Run CAP-SD](#How-to-run-CAP-SD)
- [Pre-trained YOLO v5 model](#Pre-trained-YOLO-v5-model)

## How to run CAP-SD

To launch Hector SLAM using UNITY-ROS bridge

    roslaunch grpc_ros_adapter launch_server.launch

Then PLAY UNITY

To launch the pre-trained yolov5 object detection model:

    roslaunch yolov5_ros yolov5.launch

Last step, launch CAP-SD:

    roslaunch cap cap_sd.launch

## Pre-trained YOLO v5 model