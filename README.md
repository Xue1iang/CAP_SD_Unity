# CAP_SD_Simulation


To launch Hector SLAM using UNITY-ROS bridge

    roslaunch grpc_ros_adapter launch_server.launch

Then PLAY UNITY

To launch the pre-trained yolov5 object detection model:

    roslaunch yolov5_ros yolov5.launch

Last step:

    roslaunch capsd cap_sd.launch
