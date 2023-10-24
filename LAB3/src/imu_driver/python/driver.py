#!/usr/bin/env python3

import rospy
import utm
import serial
import numpy as np
from imu_driver.msg import imu_msg
from std_msgs.msg import String
import sys

def imu_driver_pub():
    
    pub = rospy.Publisher('imu', imu_msg, queue_size=10)
    rospy.init_node('IMU_Pub', anonymous=True)
    #rate = rospy.Rate(40)
    
    imuread = imu_msg()
    
    while not rospy.is_shutdown():
        '''In the code below, serial port is detected from the roslaunch file argument.'''
        port_name = sys.argv[1]
        serialport = rospy.get_param('~port', port_name) 
        baud_rate = rospy.get_param('~baudrate', 115200)  
        port = serial.Serial(serialport, baud_rate, timeout=3 )
        lbs = port.readline()
        
        line = str(lbs)
        line = line[2:]
        
        split_line = line.split(',')

        if split_line[0] == '$VNYMR':
            
            print(line)

            #time_now = rospy.get_rostime()
            #rospy.loginfo("Current time %i %i", time_now.secs, time_now.nsecs)
            imuread.Header.seq+=1
            imuread.Header.stamp = rospy.Time.now()
            imuread.Header.frame_id = "IMU1_Frame"
            
            yaw = float(split_line[1])
            pitch = float(split_line[2])
            roll = float(split_line[3])
            magX = float(split_line[4])
            magY = float(split_line[5])
            magZ = float(split_line[6])
            accX = float(split_line[7])
            accY = float(split_line[8])
            accZ = float(split_line[9])
            gyroX = float(split_line[10])
            gyroY = float(split_line[11])
            gyroZ = float(split_line[12][0:9])

            #Euler angle to quaternion conversion.
            qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

            imuread.IMU.orientation.x = qx
            imuread.IMU.orientation.y = qy
            imuread.IMU.orientation.z = qz
            imuread.IMU.orientation.w = qw
            imuread.IMU.linear_acceleration.x = accX
            imuread.IMU.linear_acceleration.y = accY
            imuread.IMU.linear_acceleration.z = accZ
            imuread.IMU.angular_velocity.x = gyroX
            imuread.IMU.angular_velocity.y = gyroY
            imuread.IMU.angular_velocity.z = gyroZ
            imuread.MagField.magnetic_field.x = magX
            imuread.MagField.magnetic_field.y = magY
            imuread.MagField.magnetic_field.z = magZ
            pub.publish(imuread)
        
        #rate.sleep()

if __name__ == '__main__':
    try:
        imu_driver_pub()
    except rospy.ROSInterruptException:
        pass