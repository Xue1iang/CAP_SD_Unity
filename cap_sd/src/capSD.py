#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import Imu
import xc_defs
from geometry_msgs.msg import PoseStamped, Pose
from apriltag_ros.msg import AprilTagDetectionRawArray, AprilTagDetectionArray
import tf
from std_msgs.msg import Float64
from tf import TransformBroadcaster
import threading
import pandas as pd
from detection_msgs.msg  import BoundingBoxes
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped 

class CapSD:
    
    def __init__(self):
        # initialize angular velocity
        angular_vel_x = np.deg2rad(0.0)
        angular_vel_y = np.deg2rad(0.0)
        angular_vel_z = np.deg2rad(0.0)
        self.gyro_init = np.array([[angular_vel_x], [angular_vel_y], [angular_vel_z]])
        self.latest_values = [0, 0, 0, 0]

        self.jacobianH = np.diag([1.0, 1.0])

        self.rolloffset = 0

        self.depth = 0
        # Initialize slam euler angles
        self.slameul = np.array([0,0,0])
        self.lock = threading.Lock()

        # Initialize
        self.t_w2sb = (0,0,0)
        self.r_w2sb = (0,0,0,1)
        self.imu_q = np.array([0,0,0,1])

        self.xmin = 0
        self.xmax = 0
        self.ymin = 0
        self.ymax = 0
        self.pixel_x = 0
        self.pixel_y = 0

        self.max_range = rospy.get_param('multi-beam_sonar_max_range')
        self.fov_deg  = rospy.get_param('horizontal_range')
        self.width = rospy.get_param('sonar_image_width')
        self.height = rospy.get_param('sonar_image_height')
        q_is = rospy.get_param('q_is')
        self.q_is = np.array([float(q_is[0]), float(q_is[1]), float(q_is[2]), float(q_is[3])]) 
        t_is = rospy.get_param('t_is')
        self.t_is = np.array([float(t_is[0]), float(t_is[1]), float(t_is[2])]) 

        self.q_wb = np.array([0,0,0,1])
        self.t_wb = np.array([0,0,0])
        self.r_b2i = np.array([0,0,0,1])
        self.t_bi = np.array([0,0,0])
        self.r = 0
        self.theta = 0

        self.dts = 1/30
        # observation matrix H
        self.H = np.diag([1.0, 1.0])

        # system noise variance
        self.Q = np.diag([0.01962, 0.034516])

        # observation noise variance
        self.R = np.diag([0.0000179,0.0000179])

        # initialize status
        # self.x_true = np.array([[1.43], [0.03]])
        self.x_true = np.array([[0.0], [0.0]])

        # initialize prediction
        self.x_bar = self.x_true

        # initialize eastimation
        self.x_hat = self.x_true

        # initialize covariance
        self.P = self.Q

        # initialize jacbian matrix of H
        self.jacobianH = np.diag([1.0, 1.0])


        # ------ publishers --------
        self.pub_dscap = rospy.Publisher('/capsd_sys', PoseStamped, queue_size=1000)


        self.pub_one_input = Pose()
        # ------ subscriber -------
        # self.depth_subscriber = rospy.Subscriber('/bluerov/mavros/global_position/rel_alt', Float64, self.callback_depthSensor)
        self.slam_subscriber = rospy.Subscriber('/slam_out_pose', PoseStamped, self.callback_slam)
        self.yolov5_subscriber = rospy.Subscriber('/yolov5/detections', BoundingBoxes,self.callback_yolo)
        self.imu_subscriber = rospy.Subscriber('/mallard/imu', Imu, self.callback_imu)  
        self.depth_subscriber = rospy.Subscriber('/depth_sensor', PoseWithCovarianceStamped, self.callback_depth)      


        # ------ rate and shutdown ------
        self.rate = rospy.Rate(60.0)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    # ------ publisher for clean shutdown ------
    def publisher_def(self, pub_one_input):
        while not self.ctrl_c:
            self.pub_dscap.publish(pub_one_input)
            rospy.loginfo("CAP-SD system is shutting down......")
            break

    # ----- shutdown stuff ---------
    def shutdownhook(self):
        self.ctrl_c = True
        self.stop()

    def stop(self):
        # rospy.loginfo("shutdown hook")
        
        # put safe shutdown commands here
        self.pub_one_input.position.x = 0
        self.pub_one_input.position.y = 0
        self.pub_one_input.position.z = 0

        self.pub_one_input.orientation.x = 0
        self.pub_one_input.orientation.y = 0
        self.pub_one_input.orientation.z = 0
        self.pub_one_input.orientation.w = 1

        self.publisher_def(self.pub_one_input)  # publish zero translation and rotation information when you hit ctrl-c


    def callback_imu(self, imu):
        with self.lock:
            self.gyro_data = np.array([[imu.angular_velocity.x],[imu.angular_velocity.y], [imu.angular_velocity.z]])
            self.accel_data = np.array([[imu.linear_acceleration.x], [imu.linear_acceleration.y], [imu.linear_acceleration.z]])
            
            self.x_hat = self.ekf_update(self.gyro_data, self.accel_data, self.jacobianH)

            self.r_w2b = xc_defs.tilting_plus_slamyaw(self.x_hat[0],self.x_hat[1],self.slameul[2],self.rolloffset)
            self.r_w2b.resize((4,))  

            tf_W_B = TransformBroadcaster()
            tf_W_B.sendTransform((self.t_w2sb[0], self.t_w2sb[1], self.t_w2sb[2]), (self.r_w2b[0], self.r_w2b[1], self.r_w2b[2], self.r_w2b[3]), rospy.Time.now(), 'MallARD_003', 'World')
            p_wr = xc_defs.capsd(self.depth, self.r_w2b, self.t_w2sb, self.r_b2i, self.t_bi, self.q_is, self.t_is, self.r, self.theta)
            # print(self.depth)
            capsd = PoseStamped()
            capsd.pose.position.x = p_wr[0]
            capsd.pose.position.y = p_wr[1]
            capsd.pose.position.z = p_wr[2]
            self.pub_dscap.publish(capsd)

            
    def callback_slam(self,w2sb):
        with self.lock:
            # SLAM translation
            self.t_w2sb = np.array([w2sb.pose.position.x, w2sb.pose.position.y, w2sb.pose.position.z])
            # print(self.t_w2sb)
            # SLAM rotation in quaternion form
            self.r_w2sb = np.array([w2sb.pose.orientation.x, w2sb.pose.orientation.y, w2sb.pose.orientation.z, w2sb.pose.orientation.w])
            # slameul is a 1x3 vector
            slameul = tf.transformations.euler_from_quaternion(self.r_w2sb)
            # Save the previous value of self.slameul before updating it
            # self.previous_slameul = self.slameul.copy() 
            # print(slameul)
            self.slameul = np.array([[slameul[0]], [slameul[1]], [slameul[2]]])

    def callback_yolo(self, pix):
        with self.lock:
            for bbox in pix.bounding_boxes:
                xmin = bbox.xmin
                ymin = bbox.ymin
                xmax = bbox.xmax
                ymax = bbox.ymax
            
            self.pixel_x = (xmin + xmax) / 2
            self.pixel_y = (ymin + ymax) / 2

            self.r, self.theta = xc_defs.pixel_to_sonar_beam(self.width, self.height, self.fov_deg, self.max_range,  self.pixel_x, self.pixel_y)
            # self.r, self.theta = xc_defs.sonar_pixel_to_distance_angle_adjusted(self.width, self.height, self.fov_deg, self.max_range,  self.pixel_x, self.pixel_y)
            # print(self.r, self.theta)

    def ekf_update(self, gyro_angular_vel, accel_data, jacobianH):
        # [step1] prediction
        self.x_bar = xc_defs.get_status(self.x_hat, gyro_angular_vel, self.dts)
        
        # jacobian matrix           
        jacobianF = xc_defs.get_jacobianF(self.x_bar, gyro_angular_vel, self.dts)
        # pre_covariance
        P_bar = np.matmul(np.matmul(jacobianF, self.P), jacobianF.T) + self.Q

        # observation
        roll = np.arctan2(accel_data[1],accel_data[2])
        pitch = np.arctan2(accel_data[0], np.sqrt(accel_data[1]**2 + accel_data[2]**2))
        y = np.array([roll, pitch])
        # print(y)
        # [step2] update the filter
        s = np.matmul(np.matmul(self.H, P_bar), self.H.T) + self.R
        K = np.matmul(np.matmul(P_bar, self.H.T), np.linalg.inv(s))

        # eastimation
        e = y - np.matmul(jacobianH, self.x_bar)
        x_hat = self.x_bar + np.matmul(K, e)
        # post_covariance
        I = np.eye(x_hat.shape[0])
        P = np.matmul((I - np.matmul(K, self.H) ), P_bar)

        # EKF output
        self.x_hat = x_hat
        # Covariance
        self.P = P
        
        return self.x_hat

    def callback_depth(self, data):
        # if depth:
        #     self.depth = -depth.data
        # else:
        #     print("Oops(*.*)!!, It seems CAP_SD can't access to depth sensor's reading......")
        with self.lock:
            self.depth = data.pose.pose.position.z

            


if __name__ == '__main__':

    rospy.init_node('capSD', anonymous=True)
    try:
        sd = CapSD()
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()