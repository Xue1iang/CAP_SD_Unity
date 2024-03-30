# CAP-SD Simulation

## Table of contents
- [Run CAP-SD](#How-to-run-CAP-SD)
- [All the results](#Results)
- [Pre-trained YOLO v5 model](#Pre-trained-YOLO-v5-model)
- [Game<span style="color: red;"> ？</span> Research <span style="color: green;">✔</span>](#Simulated-nuclear-fuel-pond)
- [Video](#Video)

## How to run CAP-SD

To launch  UNITY-ROS bridge (including Hector SLAM8)

    roslaunch grpc_ros_adapter launch_server.launch

Then PLAY UNITY

To launch the pre-trained yolov5 object detection model:

    roslaunch yolov5_ros yolov5.launch

Last step, launch CAP-SD:

    roslaunch cap cap_sd.launch

## Results

<table>
  <tr>
    <td align="center"><img src="https://github.com/Xue1iang/IROS2024_CAP-SD/blob/main/fig/random_capsd_3dtraj.png" alt="Random trajectory" width="400"/></td>
  </tr>
  <tr>
    <td align="center"><img src="https://github.com/Xue1iang/IROS2024_CAP-SD/blob/main/fig/random_capsd.png" alt="randomxy" width="400"></td>
  </tr>
</table>



## Simulated nuclear fuel pond

## Pre-trained YOLO v5 model
The trsultd of pre-trained YOLO v5 model has been attached below. More details (like Hyperparameter Configuration) can be found in [yolov5_CAP_SD](yolov5_CAP-SD).

| ![text1](/yolov5_CAP-SD/P_curve.png) | ![text2](/yolov5_CAP-SD/R_curve.png) |
| :-------------------: | :-------------------: |
|       Precision-Confidence Curve      |       Recall-Confidence Curve       |
| ![text3](/yolov5_CAP-SD/F1_curve.png) |
|       F1-Confidence Curve      |         


<div style="width: 400px;">
  <table>
    <tr>
      <td align="center">
        <img src="/yolov5_CAP-SD/results.png" style="width:100%; max-width:400px;">
      </td>
    </tr>
    <tr>
      <td align="center">
        <strong>Results customized YOLOv5 model for CAP_SD system</strong>
      </td>
    </tr>
  </table>
</div>

<p align="center">
  <img src="/yolov5_CAP-SD/train_batch1.jpg" style="width: 50%;">
</p>
<div align="center">Train batch</div>


| ![text1](/yolov5_CAP-SD/val_batch0_labels.jpg) | ![text2](/yolov5_CAP-SD/val_batch0_pred.jpg) |
| :-------------------: | :-------------------: |
|       Val_batch_labels        |       Val_batch_predi     |

## Video
Play me 👇

[![Watch the video](/fig/cover.png)](https://www.youtube.com/watch?v=Is7qyu9I0gU)

