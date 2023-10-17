#!/usr/bin/env python3

import rospy
import utm
import serial
from gps_driver.msg import gps_msg
from std_msgs.msg import String
import sys

def gps_driver_pub():
    
    pub = rospy.Publisher('gps', gps_msg, queue_size=10)
    rospy.init_node('GPS_Pub', anonymous=True)
    rate = rospy.Rate(100)
    
    gpsread = gps_msg()
    
    while not rospy.is_shutdown():
        '''In the code below, serial port is detected from the roslaunch file argument.'''
        port_name = sys.argv[1]
        serialport = rospy.get_param('~port', port_name) 
        baud_rate = rospy.get_param('~baudrate', 57600)  
        port = serial.Serial(serialport, baud_rate, timeout=3 )
        lbs = port.readline()
        
        line = str(lbs)
        line = line[2:]
        
        split_line = line.split(',')

        if split_line[0] == '$GNGGA':
            
            gpsread.header.seq+=1
            gpsread.header.stamp=rospy.Time.now()
            gpsread.header.frame_id = "GPS1_Frame"
            
            latitude = float(split_line[2])
            latitude_DD = int(latitude/100)
            latitude_mm1 = float(latitude) - (latitude_DD * 100)
            latitude_converted = float(latitude_DD + latitude_mm1/60)
            if split_line[3]=='S':
                latitude_converted= latitude_converted*(-1)
            
            longitude = float(split_line[4])
            longitude_DD = int(longitude / 100)
            longitude_mm1 = float(longitude) - (longitude_DD * 100)
            longitude_converted = float(longitude_DD + longitude_mm1/60)
            if split_line[5]=='W':
                longitude_converted=longitude_converted*(-1)
            
            altitude = float(split_line[9])

            newlatlong = utm.from_latlon(latitude_converted,longitude_converted)

            print(f'UTM_East, UTM_north, Zone, Letter: {newlatlong}')
            print(line)
           
            gpsread.latitude = latitude_converted
            gpsread.longitude = longitude_converted
            gpsread.altitude = altitude
            gpsread.utm_easting = newlatlong[0]
            gpsread.utm_northing = newlatlong[1]
            gpsread.zone = newlatlong[2]
            gpsread.letter = newlatlong[3]
            pub.publish(gpsread)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        gps_driver_pub()
    except rospy.ROSInterruptException:
        pass