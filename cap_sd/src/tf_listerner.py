#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg

def lookup_transform(tf_buffer, target_frame, source_frame):
    try:
        trans = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(3.0))
        print("Transform: ", trans)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('tf_listener_node')

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        lookup_transform(tf_buffer, 'Wmap', 'BlueROV2/base_link')
        rate.sleep()
