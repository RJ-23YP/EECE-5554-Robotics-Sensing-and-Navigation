#!/usr/bin/env python3

import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
import pandas as pd

#In the line below, 'walking_data.bag' is a rosbag file consisting of gps data when walking in a straight line for 300 metres.
bag = bagreader('/home/rj/catkin_ws/src/data/walking_data.bag')
bag.topic_table

data = bag.message_by_topic('gps')
gpsreadings = pd.read_csv(data)

print(gpsreadings)

gpsreadings['utm_easting'] = gpsreadings['utm_easting'] - gpsreadings['utm_easting'].min()
gpsreadings['utm_northing'] = gpsreadings['utm_northing'] - gpsreadings['utm_northing'].min()
gpsreadings['longitude'] = gpsreadings['longitude'] - gpsreadings['longitude'].min()
gpsreadings['latitude'] = gpsreadings['latitude'] - gpsreadings['latitude'].min()
gpsreadings['header.stamp.secs'] = gpsreadings['header.stamp.secs'] - gpsreadings['header.stamp.secs'].min()


plt.rcParams.update({'font.size': 30})

gpsreadings[['utm_easting','utm_northing']].plot(xlabel='utm_easting', ylabel='utm_northing')
'''
figure, ax = bagpy.create_fig(1)
ax[0].scatter(x = 'utm_easting', y = 'utm_northing', data = gpsreadings, s= 50, label = 'utm_northing VS utm_easting')
for axis in ax:
    axis.legend()
    axis.set_xlabel('utm_easting (m)', fontsize=40)
    axis.set_ylabel('utm_northing (m)', fontsize=40)
'''

gpsreadings[['altitude']].plot(xlabel='Time(s)', ylabel='Altitude (m)')
'''
figure, bx = bagpy.create_fig(1)
bx[0].scatter(x = 'header.stamp.secs', y = 'altitude', data = gpsreadings, s= 50, label = 'altitude VS time')
for axis in bx:
    axis.legend()
    axis.set_xlabel('time (s)', fontsize=40)
    axis.set_ylabel('altitude (m)', fontsize=40)
'''

#gpsreadings[['longitude', 'latitude']].plot()
figure, cx = bagpy.create_fig(1)
cx[0].scatter(x = 'latitude', y = 'longitude', data = gpsreadings, s= 50, label = 'longitude VS latitude')
for axis in cx:
    axis.legend()
    axis.set_xlabel('latitude', fontsize=30)
    axis.set_ylabel('longitude', fontsize=30)

    
plt.show()
