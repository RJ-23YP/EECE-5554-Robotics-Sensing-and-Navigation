#!/usr/bin/env python3

import bagpy
from bagpy import bagreader
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.ticker import FormatStrFormatter


bag = bagreader('/home/rj/lab2_ws/src/data/walking_data_closed.bag')
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

plt.plot(rel_utm_easting, rel_utm_northing)
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.title('Walking with Occlusions UTM Northing vs UTM Easting')
plt.legend()
plt.show()

plt.hist2d(rel_utm_easting, rel_utm_northing)
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.title('Histogram Walking with Occlusions UTM Northing vs UTM Easting')
plt.legend()
plt.show()

end_of_seg_1 = np.argmax(rel_utm_easting)
east_segment_1 = rel_utm_easting[0:end_of_seg_1]
north_segment_1 = rel_utm_northing[0:end_of_seg_1]
a, b = np.polyfit(east_segment_1, north_segment_1, 1)
best_fit_1 = a*east_segment_1+b
    
end_of_seg_2 = np.argmax(rel_utm_northing)
east_segment_2 = rel_utm_easting[end_of_seg_1:end_of_seg_2]
north_segment_2 = rel_utm_northing[end_of_seg_1:end_of_seg_2]
a, b = np.polyfit(east_segment_2, north_segment_2, 1)
best_fit_2 = a*east_segment_2+b

end_of_seg_3 = np.argmin(rel_utm_easting)
east_segment_3 = rel_utm_easting[end_of_seg_2:end_of_seg_3]
north_segment_3 = rel_utm_northing[end_of_seg_2:end_of_seg_3]
a, b = np.polyfit(east_segment_3, north_segment_3, 1)
best_fit_3 = a*east_segment_3+b

east_segment_4 = rel_utm_easting[end_of_seg_3:]
north_segment_4 = rel_utm_northing[end_of_seg_3:]
a, b = np.polyfit(east_segment_4, north_segment_4, 1)
best_fit_4 = a*east_segment_4+b

best_fit = np.concatenate((best_fit_1,best_fit_2,best_fit_3,best_fit_4))

ax = plt.subplot(111)
ax.plot(rel_utm_easting, rel_utm_northing, label='Sensor Reading')
ax.plot(rel_utm_easting, best_fit, linestyle='--', linewidth=2, label='Best fit')
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.title('Walking with occlusions UTM Northing vs UTM Easting ')
ax.legend()
plt.show()

error_utm_easting = abs(best_fit - rel_utm_easting)
print(error_utm_easting)
plt.hist(error_utm_easting)
plt.xlabel('UTM Easting Error (m)')
plt.ylabel('Sample Number')
plt.title('Histogram UTM Easting Error')
plt.show()

error_utm_northing = abs(best_fit - rel_utm_northing)
print(error_utm_northing)
plt.hist(error_utm_northing)
plt.xlabel('UTM Northing Error (m)')
plt.ylabel('Sample Number')
plt.title('Histogram UTM Northing Error')
plt.show()

print(f'Average - UTM Easting Error: {np.mean(error_utm_easting)}')
print(f'Standard Deviation - UTM Easting Error: {np.std(error_utm_easting)}')
print(f'Average - UTM Northing Error: {np.mean(error_utm_northing)}')
print(f'Standard Deviation - UTM Northing Error: {np.std(error_utm_northing)}')

