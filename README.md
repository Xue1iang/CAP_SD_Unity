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
The trsultd of pre-trained YOLO v5 model has been attached below. More details (like Hyperparameter Configuration) can be found in [yolov5_CAP_SD](yolov5_CAP-SD).

| ![text1](https://github.com/Xue1iang/IROS2024_CAP-SD/blob/main/yolov5_CAP-SD/P_curve.png) | ![text2](https://github.com/Xue1iang/IROS2024_CAP-SD/blob/main/yolov5_CAP-SD/R_curve.png) |
| :-------------------: | :-------------------: |
|       Precision-Confidence Curve      |       Recall-Confidence Curve       |
| ![text3](https://github.com/Xue1iang/IROS2024_CAP-SD/blob/main/yolov5_CAP-SD/F1_curve.png) |
|       F1-Confidence Curve      |         


<div style="width: 400px;">
  <table>
    <tr>
      <td align="center">
        <img src="https://github.com/Xue1iang/IROS2024_CAP-SD/blob/main/yolov5_CAP-SD/P_curve.png" style="width:100%; max-width:400px;">
      </td>
    </tr>
    <tr>
      <td align="center">
        <strong>Precision-Confidence Curve</strong>
      </td>
    </tr>
  </table>
</div>
