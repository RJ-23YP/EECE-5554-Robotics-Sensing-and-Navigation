#!/usr/bin/env python3

import bagpy
from bagpy import bagreader
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.ticker import FormatStrFormatter


bag = bagreader('/home/rj/lab2_ws/src/data/stationary_data_closed.bag')
data = bag.message_by_topic('/gps')
gpsreadings = pd.read_csv(data)


print('Available Data Columns: ')
print(gpsreadings.columns.values[1:].tolist())

print('Zone:', gpsreadings[["zone_number"]].iloc[0].item())
print('Letter:', gpsreadings[["zone_letter"]].iloc[0].item())

time = gpsreadings[["header.stamp.secs"]].to_numpy()
utm_northing = gpsreadings[["utm_northing"]].to_numpy()
utm_easting = gpsreadings[["utm_easting"]].to_numpy()
altitude = gpsreadings[["altitude"]].to_numpy()
latitude = gpsreadings[["latitude"]].to_numpy()
longitude = gpsreadings[["longitude"]].to_numpy()
fix_quality = gpsreadings["quality"]

print('Total Data Points:', len(time))
print(f'Average Latitude: {np.mean(latitude)}')
print(f'Average Longitude: {np.mean(longitude)}')
print(gpsreadings)

rel_utm_northing = (utm_northing - utm_northing[0]).flatten()
rel_utm_easting = (utm_easting - utm_easting[0]).flatten()
rel_time = (time - time[0]).flatten()
rel_latitude = (latitude - latitude[0]).flatten()
rel_longitude = (longitude - longitude[0]).flatten()
rel_altitude = (altitude - altitude[0]).flatten()


plt.scatter(rel_utm_easting, rel_utm_northing)
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.title('Stationary with Occlusions UTM Easting vs UTM Northing')
plt.show()

plt.hist2d(rel_utm_easting, rel_utm_northing)
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.title('Histogram Stationary with Occlusions UTM Easting vs UTM Northing')
plt.show()

true_val_utm_easting = rel_utm_easting[0]
true_val_utm_northing = rel_utm_northing[0]

plt.plot(rel_utm_easting, label='UTM Easting')
plt.plot(rel_utm_northing, label='UTM Northing')
plt.axhline(y = true_val_utm_easting, linestyle = '-', color='red', label='True Value')  
plt.xlabel('Time (s)')
plt.title('UTM Easting & UTM Northing vs Time')
plt.legend()
plt.show()

error_utm_easting = abs(rel_utm_easting - true_val_utm_easting)
print(error_utm_easting)
plt.plot(error_utm_easting, label='Error UTM Easting vs Time')
plt.ylabel('UTM Easting Error (m)')
plt.xlabel('Time(s)')
plt.title('Error UTM Easting vs Time')
plt.show()

error_utm_northing = abs(rel_utm_northing - true_val_utm_northing)
print(error_utm_northing)
plt.plot(error_utm_northing, label='Error UTM Northing vs Time')
plt.ylabel('UTM Northing Error (m)')
plt.xlabel('Time(s)')
plt.title('Error UTM Northing vs Time')
plt.show()

print(f'Average - UTM Easting Error: {np.mean(error_utm_easting)}')
print(f'Standard Deviation - UTM Easting Error: {np.std(error_utm_easting)}')
print(f'Average - UTM Northing Error: {np.mean(error_utm_northing)}')
print(f'Standard Deviation - UTM Northing Error: {np.std(error_utm_northing)}')