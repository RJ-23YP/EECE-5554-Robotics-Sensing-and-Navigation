#!/usr/bin/env python3

import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
import pandas as pd

#In the line below, 'stationary_data.bag' is a rosbag file which contains gps readings at a stationary point. 
bag = bagreader('/home/rj/catkin_ws/src/data/stationary_data.bag')
bag.topic_table

data = bag.message_by_topic('gps')
gpsreadings = pd.read_csv(data)

print(gpsreadings)

gpsreadings['utm_easting'] = gpsreadings['utm_easting'] - gpsreadings['utm_easting'].min()
gpsreadings['utm_northing'] = gpsreadings['utm_northing'] - gpsreadings['utm_northing'].min()
gpsreadings['longitude'] = gpsreadings['longitude'] - gpsreadings['longitude'].min()
gpsreadings['latitude'] = gpsreadings['latitude'] - gpsreadings['latitude'].min()
gpsreadings['header.stamp.secs'] = gpsreadings['header.stamp.secs'] - gpsreadings['header.stamp.secs'].min()


plt.rcParams.update({'font.size': 35})

#gpsreadings[['utm_easting','utm_northing']].plot()
figure, ax = bagpy.create_fig(1)
ax[0].scatter(x = 'utm_easting', y = 'utm_northing', data = gpsreadings, s= 50, label = 'utm_northing VS utm_easting')
for axis in ax:
    axis.legend()
    axis.set_xlabel('utm_easting (m)', fontsize=40)
    axis.set_ylabel('utm_northing (m)', fontsize=40)

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
    axis.set_xlabel('latitude', fontsize=35)
    axis.set_ylabel('longitude', fontsize=35)

plt.show()
