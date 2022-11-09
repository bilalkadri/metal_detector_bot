#!/usr/bin/env python  
import roslib
roslib.load_manifest('metal_detector_bot')

import rospy
import tf



#! /usr/bin/env python3
 
# This program converts Euler angles to a quaternion.
# Author: AutomaticAddison.com
 
import numpy as np # Scientific computing library for Python
 
def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]



if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        br.sendTransform((-0.0765, 0.00190302448688115, 0.00943214142854351),
                         (get_quaternion_from_euler(-1.5707963267949, -1.17899991460539, -1.5707963267949)),
                         rospy.Time.now(),
                         "Left_wheel_link",
                         "base_link")
        br.sendTransform((0.0764999999999999, 0.00190302448688115, 0.00943214142854353),
                         (get_quaternion_from_euler(1.5707963267949, -0.566329337388945, 1.5707963267949)),
                         rospy.Time.now(),
                         "Right_wheel_link",
                         "base_link")
        br.sendTransform((-0.0001379719182397, 0.0700435036565304, -0.02045),
                         (get_quaternion_from_euler(1.5707963267949, 0, 0)),
                         rospy.Time.now(),
                         "Ball_caster_front_link",
                         "base_link")
        br.sendTransform((-0.0001379719182397, -0.0700435036565304, -0.014),
                         (get_quaternion_from_euler(1.5707963267949, 0, 0)),
                         rospy.Time.now(),
                         "Ball_caster_back_link",
                         "base_link")
                      
        rate.sleep()



 