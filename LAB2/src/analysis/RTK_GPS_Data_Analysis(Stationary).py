#!/usr/bin/env python3

import bagpy
from bagpy import bagreader
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.ticker import FormatStrFormatter


bag = bagreader('/home/rj/lab2_ws/src/data/stationary.bag')
data = bag.message_by_topic('/rtk')
gpsreadings = pd.read_csv(data)


print('Available Data Columns: ')
print(gpsreadings.columns.values[1:].tolist())

# Extract and display zone and letter information
print('Zone:', gpsreadings[["zone"]].iloc[0].item())
print('Letter:', gpsreadings[["letter"]].iloc[0].item())

time = gpsreadings[["header.stamp.secs"]].to_numpy()
utm_northing = gpsreadings[["utmNorthing"]].to_numpy()
utm_easting = gpsreadings[["utmEasting"]].to_numpy()
altitude = gpsreadings[["altitude"]].to_numpy()
latitude = gpsreadings[["latitude"]].to_numpy()
longitude = gpsreadings[["longitude"]].to_numpy()
fix_quality = gpsreadings["quality"]


print('Total Data Points:', len(time))
print(f'Average Latitude: {np.mean(latitude)}')
print(f'Average Longitude: {np.mean(longitude)}')


rel_utm_northing = (utm_northing - utm_northing[0]).flatten()
rel_utm_easting = (utm_easting - utm_easting[0]).flatten()
rel_time = (time - time[0]).flatten()
rel_latitude = (latitude - latitude[0]).flatten()
rel_longitude = (longitude - longitude[0]).flatten()
rel_altitude = (altitude - altitude[0]).flatten()


plt.scatter(rel_utm_easting, rel_utm_northing)
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.title('Stationary UTM Easting vs UTM Northing')
plt.show()

print(f'Average UTM Easting: {np.mean(rel_utm_easting)}')
print(f'Standard Deviation UTM Easting: {np.std(rel_utm_easting)}')
print(f'Average UTM Northing: {np.mean(rel_utm_northing)}')
print(f'Standard Deviation UTM Northing: {np.std(rel_utm_northing)}')


colormap = {1:'r', 2:'k', 3:'r', 4:'g', 5:'b'}
color = fix_quality.map(colormap)
plt.scatter(rel_utm_easting, rel_utm_northing, c=color.astype(str))
plt.xlabel('Relative UTM Easting (m)')
plt.ylabel('Relative UTM Northing (m)')
plt.title('Stationary UTM Variance from Mean')
plt.show()

'''
fig, ax = plt.subplots()
plt.scatter(latitude, longitude)
ax.yaxis.set_major_formatter(FormatStrFormatter('%.4f'))
ax.xaxis.set_major_formatter(FormatStrFormatter('%.4f'))
plt.xlabel('Latitude (degrees)')
plt.ylabel('Longitude (degrees)')
plt.title('Latitude vs Longitude')
plt.show()


print(f'Average Latitude: {np.mean(rel_latitude)}')
print(f'Standard Deviation Latitude: {np.std(rel_latitude)}')
print(f'Average Longitude: {np.mean(rel_longitude)}')
print(f'Standard Deviation Longitude: {np.std(rel_longitude)}')


plt.scatter(rel_latitude, rel_longitude)
plt.xlabel('Relative Latitude (degrees)')
plt.ylabel('Relative Longitude (degrees)')
plt.title('Latitude vs Longitude Variance from Mean')
plt.show()
'''

print(f'Altitude Precision: {np.std(altitude, dtype=np.float32)}')
mean_relative_altitude = np.mean(rel_altitude)
std_dev_altitude = np.std(rel_altitude)
print(f'Mean Relative Altitude: {mean_relative_altitude}')
print(f'Altitude Standard Deviation: {std_dev_altitude}')
plt.plot(rel_time, altitude)
plt.xlabel('Relative Time (s)')
plt.ylabel('Altitude (m)')
plt.title('Altitude vs Time')
plt.show()

