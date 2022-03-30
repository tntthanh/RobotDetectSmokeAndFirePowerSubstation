#!/usr/bin/env python3

import time
import serial
from sensor_msgs.msg import NavSatFix
import rospy
from std_msgs.msg import String, Int8, Float32

GPS = NavSatFix()

port_name = rospy.get_param('~port','/dev/ttyUSB1')
baud = int(rospy.get_param('~baud','115200'))
ser = serial.Serial(
    port = port_name,
    baudrate = baud,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1
)

latitude = 0
longitude = 0
altitude = 0
errorLat = 0
errorLon = 0
errorAlt = 0

string_gps = ''

print("Raspberry's receiving : ")

rospy.init_node('GPS_publisher')
GPS_pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=50)
errorLat_pub = rospy.Publisher("errorLat", Float32, queue_size=10)
errorLon_pub = rospy.Publisher("errorLon", Float32, queue_size=10)
errorAlt_pub = rospy.Publisher("errorAlt", Float32, queue_size=10)
 
try:
    while True: 
        current_time = rospy.Time.now()
        s = ser.readline()
        data = s.decode()           # decode s
        data = data.rstrip()            # cut "\r\n" at last of string
        print("aaa", data)             # print string
        try:
            if data[1:6] == 'GNGBS':
                out = data.split(',')
                errorLat = float(out[2])
                errorLon = float(out[3])
                errorAlt = float(out[4])

                if data[1:6] == 'GNGGA':
                    out = data.split(',')
                    # print(out)
                    lat_str = out[2]
                    log_str = out[4]
                    alt = out[9]

                    altUnit = out[11]

                    latitude = round(float(lat_str[0:2]) + float(lat_str[2:])/60, 8)
                    longitude = round(float(log_str[0:3]) + float(log_str[3:])/60, 8)   
                    altitude = float(alt) + float(altUnit)
                    
                    if out[6] == '5':
                        string_gps += '\nQuality: float' + '\n'
                    if out[6] =='4':
                        string_gps += 'Quality: fix' + '\n'
                    string_gps += 'Number of satelites: ' + out[7]
                    rospy.loginfo(string_gps)
                    string_gps = ''
                    #print(latitude, longitude, altitude)
        except:
            rospy.logwarn('disconnected GPS')
            
        GPS.header.stamp = current_time
        GPS.header.frame_id = "base_link"
        GPS.latitude = latitude
        GPS.longitude = longitude
        GPS.altitude = altitude
        GPS.position_covariance = [1e-05, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0 , 0.0, 1e-07]
        GPS.position_covariance_type = 2
        GPS_pub.publish(GPS)
    errorLat_pub.publish(errorLat)
    errorLon_pub.publish(errorLon)
    errorAlt_pub.publish(errorAlt)
except rospy.ROSInterruptException:
    pass
    #ser.close()
    #rospy.logwarn('disconnected GPS')