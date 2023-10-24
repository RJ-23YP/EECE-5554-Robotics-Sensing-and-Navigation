#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import statistics
import bagpy
from bagpy import bagreader
import pandas as pd
import math
import seaborn as sns
import csv

plt.rcParams.update({'font.size': 16})

bag = bagreader('/home/rj/lab3_ws/src/data/imu_data_15_mins.bag')
imudata = bag.message_by_topic('/imu')
imureadings = pd.read_csv(imudata)

print(imudata)
w = imureadings['IMU.orientation.w'] * (np.pi/180)
x = imureadings['IMU.orientation.x']* (np.pi/180)
y = imureadings['IMU.orientation.y']* (np.pi/180)
z = imureadings['IMU.orientation.z']* (np.pi/180)
print(w, imureadings['IMU.orientation.w'])


#Quaternion to Euler Angles Conversion
a0 = +2.0 * (w * x + y * z)
a1 = +1.0 - 2.0 * (x * x + y *y)
roll = np.degrees(np.arctan2(a0, a1))

a2 = +2.0 * (w * y - z * x)
a2 = np.where(a2>+1.0, +1.0,a2)
a2 = np.where(a2<-1.0, -1.0,a2)
pitch = np.degrees(np.arcsin(a2))

a3 = +2.0 * (w * z + x * y)
a4 = +1.0 - 2.0 * (y * y+ z * z)
yaw = np.degrees(np.arctan2(a3, a4))

imureadings['Header.seq'] = imureadings['Header.seq'] - imureadings['Header.seq'].min()
imureadings['IMU.angular_velocity.x'] = imureadings['IMU.angular_velocity.x'] - imureadings['IMU.angular_velocity.x'].min()
imureadings['IMU.angular_velocity.y'] = imureadings['IMU.angular_velocity.y'] - imureadings['IMU.angular_velocity.y'].min()
imureadings['IMU.angular_velocity.z'] = imureadings['IMU.angular_velocity.z'] - imureadings['IMU.angular_velocity.z'].min()
imureadings['IMU.linear_acceleration.x'] = imureadings['IMU.linear_acceleration.x'] - imureadings['IMU.linear_acceleration.x'].min()
imureadings['IMU.linear_acceleration.y'] = imureadings['IMU.linear_acceleration.y'] - imureadings['IMU.linear_acceleration.y'].min()
imureadings['IMU.linear_acceleration.z'] = imureadings['IMU.linear_acceleration.z'] - imureadings['IMU.linear_acceleration.z'].min()
imureadings['MagField.magnetic_field.x'] = imureadings['MagField.magnetic_field.x'] - imureadings['MagField.magnetic_field.x'].min()
imureadings['MagField.magnetic_field.y'] = imureadings['MagField.magnetic_field.y'] - imureadings['MagField.magnetic_field.y'].min()
imureadings['MagField.magnetic_field.z'] = imureadings['MagField.magnetic_field.z'] - imureadings['MagField.magnetic_field.z'].min()

#Mean and Standard Deviation - Roll, Pitch & Yaw
print('Mean & Standard Deviation of Roll, Pitch & Yaw:')
print('mean = ',statistics.mean(roll))
print('mean = ',statistics.mean(pitch))
print('mean = ',statistics.mean(yaw))
print('standard deviation = ',statistics.stdev(roll))
print('standard deviation = ',statistics.stdev(pitch))
print('standard deviation = ',statistics.stdev(yaw))

#Mean and Standard Deviation - Magnetic Field
print('Mean & Standard Deviation of Magnetic Field:')
for i in ['MagField.magnetic_field.x', 'MagField.magnetic_field.y', 'MagField.magnetic_field.z']:
    print('mean = ',abs(imureadings[i]).mean())
    print('standard deviation = ',imureadings[i].std())

#Mean and Standard Deviation - Linear Acceleration
print('Mean & Standard Deviation of Linear Acceleration:')
for i in ['IMU.linear_acceleration.x', 'IMU.linear_acceleration.y', 'IMU.linear_acceleration.z']:
    print('mean = ',imureadings[i].mean())
    print('standard deviation = ',imureadings[i].std())

#Mean and Standard Deviation - Angular Velocity
print('Mean & Standard Deviation of Angular Velocity:')
for i in ['IMU.angular_velocity.x', 'IMU.angular_velocity.y', 'IMU.angular_velocity.z']:
    print('mean = ',imureadings[i].mean())
    print('standard deviation = ',imureadings[i].std())

#Line Graphs for all the IMU Sensor Readings:
f, ax = plt.subplots(3, 1, figsize=(30, 18))
f.subplots_adjust(hspace=0.4)
ax[0].plot(imureadings['Header.seq'], roll, label = 'Time VS roll')
ax[1].plot(imureadings['Header.seq'], pitch, label = 'Time VS pitch')
ax[2].plot(imureadings['Header.seq'], yaw, label = 'Time VS yaw')
ax[0].set_xlabel('Time (Seconds)')
ax[0].set_ylabel('Roll (Degrees)')
ax[0].set_title('Roll vs Time')
ax[1].set_xlabel('Time (Seconds)')
ax[1].set_ylabel('Pitch (Degrees)')
ax[1].set_title('Pitch vs Time')
ax[2].set_xlabel('Time (Seconds)')
ax[2].set_ylabel('Yaw (Degrees)')
ax[2].set_title('Yaw vs Time')

f, ax = plt.subplots(3, 1, figsize=(30, 18))
f.subplots_adjust(hspace=0.4)
ax[0].plot(imureadings['Header.seq'], imureadings['MagField.magnetic_field.x'], label = 'Time VS MagFieldX')
ax[1].plot(imureadings['Header.seq'], imureadings['MagField.magnetic_field.y'], label = 'Time VS MagFieldY')
ax[2].plot(imureadings['Header.seq'], imureadings['MagField.magnetic_field.z'], label = 'Time VS MagFieldZ')
ax[0].set_xlabel('Time (Seconds)')
ax[0].set_ylabel('MagFieldX (Gauss)')
ax[0].set_title('MagFieldX vs Time')
ax[1].set_xlabel('Time (Seconds)')
ax[1].set_ylabel('MagFieldY (Gauss)')
ax[1].set_title('MagFieldX vs Time')
ax[2].set_xlabel('Time (Seconds)')
ax[2].set_ylabel('MagFieldZ (Gauss)')
ax[2].set_title('MagFieldZ vs Time')

f, ax = plt.subplots(3, 1, figsize=(30, 18))
f.subplots_adjust(hspace=0.4)
ax[0].plot(imureadings['Header.seq'], imureadings['IMU.linear_acceleration.x'])
ax[1].plot(imureadings['Header.seq'], imureadings['IMU.linear_acceleration.y'])
ax[2].plot(imureadings['Header.seq'], imureadings['IMU.linear_acceleration.z'])
ax[0].set_xlabel('Time (Seconds)')
ax[0].set_ylabel('Linear Acceleration_X (m/s\u00b2)')
ax[0].set_title('Linear Acceleration_X vs Time')
ax[1].set_xlabel('Time (Seconds)')
ax[1].set_ylabel('Linear Acceleration_Y (m/s\u00b2)')
ax[1].set_title('Linear Acceleration_Y vs Time')
ax[2].set_xlabel('Time (Seconds)')
ax[2].set_ylabel('Linear Acceleration_Z (m/s\u00b2)')
ax[2].set_title('Linear Acceleration_Z vs Time')

f, ax = plt.subplots(3, 1, figsize=(30, 18))
f.subplots_adjust(hspace=0.4)
ax[0].plot(imureadings['Header.seq'], imureadings['IMU.angular_velocity.x'])
ax[1].plot(imureadings['Header.seq'], imureadings['IMU.angular_velocity.y'])
ax[2].plot(imureadings['Header.seq'], imureadings['IMU.angular_velocity.z'])
ax[0].set_xlabel('Time (Seconds)')
ax[0].set_ylabel('Angular Velocity_X (rad/sec)')
ax[0].set_title('Angular Velocity_X vs Time')
ax[1].set_xlabel('Time (Seconds)')
ax[1].set_ylabel('Angular Velocity_Y (rad/sec)')
ax[1].set_title('Angular Velocity_Y vs Time')
ax[2].set_xlabel('Time (Seconds)')
ax[2].set_ylabel('Angular Velocity_Z (rad/sec)')
ax[2].set_title('Angular Velocity_Z vs Time')

#Histograms for all the IMU Sensor Readings:
f, ax = plt.subplots(3, 1, figsize=(30, 18))
f.subplots_adjust(hspace=0.4)
ax[0].hist(roll, bins= 40)
ax[1].hist(pitch, bins= 40)
ax[2].hist(yaw, bins= 40)
ax[0].set_xlabel('Roll (Degrees)')
ax[0].set_ylabel('Frequency')
ax[0].set_title('Roll vs Frequency')
ax[1].set_xlabel('pitch (Degrees)')
ax[1].set_ylabel('Frequency')
ax[1].set_title('Pitch vs Frequency')
ax[2].set_xlabel('Yaw (Degrees)')
ax[2].set_ylabel('Frequency')
ax[2].set_title('Yaw vs Frequency')

f, ax = plt.subplots(3, 1, figsize=(30, 18))
f.subplots_adjust(hspace=0.4)
ax[0].hist(imureadings['MagField.magnetic_field.x'], bins= 40)
ax[1].hist(imureadings['MagField.magnetic_field.y'], bins= 40)
ax[2].hist(imureadings['MagField.magnetic_field.z'], bins= 40)
ax[0].set_xlabel('MagFieldX (Gauss)')
ax[0].set_ylabel('Frequency')
ax[0].set_title('MagFieldX vs Frequency')
ax[1].set_xlabel('MagFieldY (Gauss)')
ax[1].set_ylabel('Frequency')
ax[1].set_title('MagFieldY vs Frequency')
ax[2].set_xlabel('MagFieldZ (Gauss)')
ax[2].set_ylabel('Frequency')
ax[2].set_title('MagFieldZ vs Frequency')

f, ax = plt.subplots(3, 1, figsize=(30, 18))
f.subplots_adjust(hspace=0.4)
ax[0].hist(imureadings['IMU.linear_acceleration.x'], bins= 40)
ax[1].hist(imureadings['IMU.linear_acceleration.y'], bins= 40)
ax[2].hist(imureadings['IMU.linear_acceleration.z'], bins= 40)
ax[0].set_xlabel('Linear Acceleration_X (m/s\u00b2))')
ax[0].set_ylabel('Frequency')
ax[0].set_title('Linear Acceleration_X vs Frequency')
ax[1].set_xlabel('Linear Acceleration_Y (m/s\u00b2))')
ax[1].set_ylabel('Frequency')
ax[1].set_title('Linear Acceleration_Y vs Frequency')
ax[2].set_xlabel('Linear Acceleration_Z (m/s\u00b2)')
ax[2].set_ylabel('Frequency')
ax[2].set_title('Linear Acceleration_Z vs Frequency')

f, ax = plt.subplots(3, 1, figsize=(30, 18))
f.subplots_adjust(hspace=0.4)
ax[0].hist(imureadings['IMU.angular_velocity.x'], bins= 40)
ax[1].hist(imureadings['IMU.angular_velocity.y'], bins= 40)
ax[2].hist(imureadings['IMU.angular_velocity.z'], bins= 40)
ax[0].set_xlabel('Angular Velocity_X (rad/sec)')
ax[0].set_ylabel('Frequency')
ax[0].set_title('Angular Velocity_X vs Frequency')
ax[1].set_xlabel('Angular Velocity_Y (rad/sec)')
ax[1].set_ylabel('Frequency')
ax[1].set_title('Angular Velocity_Y vs Frequency')
ax[2].set_xlabel('Angular Velocity_Z (rad/sec)')
ax[2].set_ylabel('Frequency')
ax[2].set_title('Angular Velocity_Z vs Frequency')

plt.rcParams.update({'font.size': 16})
plt.show()
