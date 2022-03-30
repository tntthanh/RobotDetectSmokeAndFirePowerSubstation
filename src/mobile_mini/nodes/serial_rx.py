#!/usr/bin/env python3

import rospy
import serial
import locale
from locale import atof
from std_msgs.msg import String, Int8, Float32
from geometry_msgs.msg import Twist, Quaternion
import numpy as np

VEL_ERROR_EST_FACTOR =  1.0
L_ERROR_EST_FACTOR =    1.0

SAMPLE_TIME = 11.35 #thoi gian lay mau (ms)
#CONVERT_FACTOR = 3183.098862  #2630.85018 #15167.54 #18554.4170589864      #545.5552478538     
CONVERT_FACTOR = 22765.99347

ALPHA = CONVERT_FACTOR * VEL_ERROR_EST_FACTOR * SAMPLE_TIME / 1000.0
BETA = 1 / ALPHA
DELAY_TIME = 200
SAMPLE_TIME =  11.35 #thoi gian lay mau (ms)

L = 0.32 * L_ERROR_EST_FACTOR

str_msg_rx = ""
flag_uart = 0x0
vel_pub_msg = Twist()
# quat_pub_msg = Quaternion()
theta_pub_msg = Float32()
angular_z = Float32()
enc_L = 0.0 
enc_R = 0.0
v_L = 0.0
v_R = 0.0
num_sample = 0
ave_yaw = 0.0

def doNothing():
    return 

def MOBILE_Vel_Cal():
    global v_L, v_R, enc_L, enc_R
    global vel_pub_msg
    global yaw, pre_yaw
    v_L = BETA * enc_L 
    v_R = BETA * enc_R
    #print("%0.2f/0.2fk" % (v_L,v_R))
    vel_pub_msg.linear.x = (v_L + v_R)/2.0
    vel_pub_msg.angular.z = (v_R - v_L)/L


def main():
    #port_name = rospy.get_param('~port','/dev/ttyAMA0')
    port_name = rospy.get_param('~port','/dev/ttyUSB0')
    baud = int(rospy.get_param('~baud','115200'))
    ser = serial.Serial(
        port = port_name,
        baudrate = baud,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS,
        timeout = 1
    )
       
    rospy.init_node('serial_rx', anonymous=True)
    vel_pub = rospy.Publisher('vel_pub', Twist, queue_size=10)
    
    theta_pub = rospy.Publisher('theta_pub', Float32, queue_size=10)
    angular_pub = rospy.Publisher('angular_z_pub', Float32, queue_size=10)   

    encoder_left = rospy.Publisher('encoder_left', Float32, queue_size=10)
    encoder_right = rospy.Publisher('encoder_right', Float32, queue_size=10)  

    global str_msg_rx
    global flag_uart
    global vel_pub_msg
    global enc_L, enc_R
    global yaw, pre_yaw 
    global num_sample
    global ave_yaw
    global pre_yaw
    global encoders
    
    encoders = Twist()
    yaw = 0.0
    pre_yaw = 0.0
    num_sample = 0
    ave_yaw = 0.0
    pre_yaw = 0.0
    i = 0
    j = 0

    while not rospy.is_shutdown():
        i += 1
        j += 1
        x = ser.readline()
        print("Frame: ", x)
        if x != '':
            tmp = str(x).split('/')
            #print("tmp_line101", tmp)
            if len(tmp) >= 1:
                try:
                    enc_L = float(tmp[0][-6:])
                    enc_R = float(tmp[1])
                    #encoders.linear.x = enc_L
                    #encoders.linear.y = enc_R
                    angular_z.data = float(tmp[3][:-1])
                    theta_pub_msg.data = -(float(tmp[2]))*3.141592653589/180
                    print("Yaw: ", float(tmp[2]))
                    print(enc_L*BETA,enc_R*BETA)

                    MOBILE_Vel_Cal()
                except:
                    pass    

        if i == 1:
            i = 0
            vel_pub.publish(vel_pub_msg)
            angular_pub.publish(angular_z)
            theta_pub.publish(theta_pub_msg)
            
        #encoder_left.publish(enc_L/5)
        #encoder_right.publish(enc_R/5)
        encoder_left.publish(enc_L)
        encoder_right.publish(enc_R)
        # rate.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
