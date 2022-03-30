#!/usr/bin/env python3

import rospy
import serial
import locale
from locale import atof
from std_msgs.msg import String, Int8, Float32
from geometry_msgs.msg import Twist

VEL_ERROR_EST_FACTOR =  1.0
L_ERROR_EST_FACTOR =    1.0

SAMPLE_TIME = 11.35 #thoi gian lay mau (ms)
CONVERT_FACTOR = 22765.85018 #15167.54 #18554.4170589864          #545.5552478538       

ALPHA = CONVERT_FACTOR * VEL_ERROR_EST_FACTOR * SAMPLE_TIME / 1000.0

DELAY_TIME = 200
L = 0.32 * L_ERROR_EST_FACTOR
PHI = L*0.5

str_msg_tx = "+00.00/+00.00k"
str_msg_stop = "+00.00/+00.00/00/00k"
flag_uart = 0x0
enc_L = 0.0
enc_R = 0.0
linear_x = 0.0
angular_z = 0.0
encoders_msg = Twist()

def MOTOR_Enc_Cal():
    global enc_L, enc_R, linear_x, angular_z

    v_L = linear_x - angular_z * PHI
    v_R = linear_x + angular_z * PHI

    enc_L = round(v_L * ALPHA, 2)
    enc_R = round(v_R * ALPHA, 2)
    
    encoders_msg.linear.x = enc_L
    encoders_msg.linear.y = enc_R
    
    encoders.publish(encoders_msg)    

    #print(enc_L)
    #print(enc_R)

def Callback(vel_sub_msg):
    global str_msg_tx
    global flag_uart 
    global enc_L, enc_R, linear_x, angular_z

    linear_x= vel_sub_msg.linear.x
    angular_z= vel_sub_msg.angular.z

    MOTOR_Enc_Cal()
    try:
        if enc_L < 0 and enc_R < 0:
            str_msg_tx = "%06.2f/%06.2f" % (enc_L,enc_R)
        if enc_L > 0 and enc_R < 0:
            str_msg_tx = "+%05.2f/%06.2f" % (enc_L,enc_R)
        if enc_L < 0 and enc_R > 0:
            str_msg_tx = "%06.2f/+%05.2f" % (enc_L,enc_R)
        if enc_L >= 0 and enc_R >= 0:
            str_msg_tx = "+%05.2f/+%05.2f" % (enc_L,enc_R)
        flag_uart = 0x1

        check_1 =  (int(str_msg_tx[1]) + int(str_msg_tx[2]) + int(str_msg_tx[4]) + int(str_msg_tx[5])) 
        check_2 =   (int(str_msg_tx[8]) +  int(str_msg_tx[9]) + int(str_msg_tx[11]) + int(str_msg_tx[12]))
        str_msg_tx += "/%02.0f/%02.0fk" % (check_1, check_2)
        #print(str_msg_tx)
    except:
        str_msg_tx = "+00.00/+00.00/00/00k"
        #print(str_msg_tx)
def main():
    global encoders
    
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

    rospy.init_node('serial_tx', anonymous=True)
    rospy.Subscriber('/cmd_vel',Twist,Callback)

    encoders = rospy.Publisher('Setpoint', Twist, queue_size=10)
    global str_msg_tx
    global str_msg_stop
    global flag_uart
    
    while not rospy.is_shutdown():
        if flag_uart == 0x1:
            flag_uart = 0x0            
            ser.write(str_msg_tx.encode())
            ser.write(str_msg_tx.encode())
            rospy.loginfo(str_msg_tx)
        
    ser.write(str_msg_stop)
    ser.write(str_msg_stop)
    rospy.logwarn("Ros master disconnected")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
