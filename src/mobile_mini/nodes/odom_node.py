#!/usr/bin/env python3

import math
from math import sin, cos, pi
import numpy as np

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class Odom:
    def __init__(self):
        rospy.init_node('odometry_publisher')
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.th_pre = 0.0
        self.delta = 0.0

        self.vx = 0
        self.vy = 0
        self.vth = 0

        # self.quat = Quaternion()
        
        self.last_time = rospy.Time.now()

        rospy.Subscriber('vel_pub', Twist, self.vel_callback)
        # rospy.Subscriber('quat_pub', Quaternion, self.quat_callback)
        rospy.Subscriber('theta_pub', Float32, self.yaw_callback)

    def spin(self):
        r = rospy.Rate(10.)
        while not rospy.is_shutdown():
            self.pubAndBroadcast()
            r.sleep()

    def pubAndBroadcast(self):
        current_time = rospy.Time.now()
    # compute odometry in a typical way given the velocities of the robot
        dt = (current_time - self.last_time).to_sec()
        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
        delta_th = self.vth * dt

	    #delta_x = (self.vx * cos(self.delta)) * dt
        #delta_y = (self.vx * sin(self.delta)) * dt	

        self.x += delta_x #- 0.395*np.sign(delta_x)
        self.y += delta_y #- 0.395*np.sign(delta_y)
        self.th += delta_th

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

        # publish the message
        self.odom_pub.publish(odom)
        self.last_time = current_time
        
        print(odom)
    def vel_callback(self,vel):
        self.vx = vel.linear.x
        self.vth = vel.angular.z
    
    # def quat_callback(self,quat):
    #     self.quat.w = quat.w
    #     self.quat.x = quat.x
    #     self.quat.y = quat.y
    #     self.quat.z = quat.z

    def yaw_callback(self,_yaw):
        self.th_pre = self.th
        self.th = _yaw.data
        self.delta = self.th - self.th_pre
        
if __name__ == '__main__':
    odom_object = Odom()
    odom_object.spin()

