import pandas as pd
import numpy as np
from scipy import integrate
from bagpy import bagreader
import matplotlib.pyplot as plt

# Read the rosbag file and convert it to csv, and later to a numpy array for further processing. 
bag = bagreader('/home/rj/lab4_ws/analysis/data/lab4.bag')

gpsdata = bag.message_by_topic('/gps')
gpsreadings = pd.read_csv(gpsdata)
imudata = bag.message_by_topic('/imu')
imureadings = pd.read_csv(imudata)

gps_parameters = ['Time', 'latitude', 'longitude', 'altitude', 'utm_easting', 'utm_northing']
imu_parameters = ['Time', 'IMU.orientation.x', 'IMU.orientation.y', 'IMU.orientation.z', 'IMU.orientation.w', 'IMU.angular_velocity.x', 'IMU.angular_velocity.y', 
                  'IMU.angular_velocity.z', 'IMU.linear_acceleration.x', 'IMU.linear_acceleration.y', 'IMU.linear_acceleration.z', 'MagField.magnetic_field.x', 
                  'MagField.magnetic_field.y', 'MagField.magnetic_field.z']

gpsdata_df = pd.DataFrame(gpsreadings, columns=gps_parameters).astype(float)
imudata_df = pd.DataFrame(imureadings, columns=imu_parameters).astype(float)

gpsdata_numarr = np.array(gpsdata_df)
imudata_numarr = np.array(imudata_df)

# Split the gps data as per the actual driving conditions, and then split the imu data accordingly
gps_move = gpsdata_numarr[350:, :] #When the car is moving
gps_round = gpsdata_numarr[55:200, :] #When the car is moving in circles
rel_gps_move = gps_move - gps_move[0]

imu_move_first = np.argmin(np.abs(gpsdata_numarr[350, 0] - imudata_numarr[:, 0]))
imu_move = imudata_numarr[imu_move_first:, :]
rel_imu_move = imu_move - imu_move[0]

imu_round_first = np.argmin(np.abs(gpsdata_numarr[55, 0] - imudata_numarr[:, 0]))
imu_round_last = np.argmin(np.abs(gpsdata_numarr[200, 0] - imudata_numarr[:, 0]))
imu_round = imudata_numarr[imu_round_first:imu_round_last, :]

meanx_imu_round = np.mean(imu_round[:, 11])
meany_imu_round = np.mean(imu_round[:, 12])

# Plot the path followed by the car in 2-D space
rel_gpsdata_numarr = gpsdata_numarr - gpsdata_numarr[0] #Scaling the easting and northing values to be plotted
plt.plot(rel_gpsdata_numarr[:, 4], rel_gpsdata_numarr[:, 5])
plt.grid()
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.title('UTM Northing vs Easting')
plt.show()

#Linear Acceleration
acc_x_integrat = integrate.cumtrapz(imu_move[:, 8], imu_move[:, 0], initial=0)

x_data = np.array([imu_move[200,0], imu_move[-1,0]])
y_data = np.array([acc_x_integrat[200], acc_x_integrat[-1]])
m,b = np.polyfit(x_data,y_data,1)

m,b = np.polyfit(x_data,y_data,1)
lobf_dist = np.absolute(imu_move[:,0]*m - acc_x_integrat[:] + b)/(np.sqrt(m**2+1))
lobf_dist[12632:29632] = 0.5*lobf_dist[12632:29632]


# Dead Reckoning

def piwrap(data):
    wrapda = np.remainder(data, 2 * np.pi)
    mask = np.abs(wrapda) > np.pi
    wrapda[mask] -= 2 * np.pi * np.sign(wrapda[mask])
    return wrapda

imu_move_magx_corr = imu_move[:, 11] - meanx_imu_round
imu_move_magy_corr = imu_move[:, 12] - meany_imu_round
yaw_move_mag_corr = np.arctan2(imu_move_magx_corr, imu_move_magy_corr)
yaw_move_int_corr = integrate.cumtrapz(imu_move[:, 7], imu_move[:, 0], initial=0)
yaw_move_int_corr_final = piwrap(yaw_move_int_corr)
unwrap_yaw_move_mag_corr = np.unwrap(yaw_move_mag_corr)
mov_comple_corr = 0.98*yaw_move_int_corr + 0.02*unwrap_yaw_move_mag_corr
mov_comple_corr_final = piwrap(mov_comple_corr)
mov_comple_corr_final = -1*mov_comple_corr_final + 105*np.pi/180

#Quaternion to Euler Angle Conversion
x = imu_move[:, 1]
y = imu_move[:, 2] 
z = imu_move[:, 3] 
w = imu_move[:, 4]

a3 = +2.0 * (w * z + x * y)
a4 = +1.0 - 2.0 * (y * y + z * z)
yaw_move = np.arctan2(a3, a4)
yaw_move = -1*yaw_move + 80*np.pi/180

ev = np.cos(mov_comple_corr_final)
nv = np.sin(mov_comple_corr_final)

ev_mag = ev * lobf_dist
nv_mag = nv * lobf_dist
pos_ev = integrate.cumtrapz(ev_mag, imu_move[:, 0], initial=0)
pos_nv = integrate.cumtrapz(nv_mag, imu_move[:, 0], initial=0)
pos_ev = pos_ev + gps_move[0, 4]
pos_nv = pos_nv + gps_move[0, 5]
plt.close()
plt.plot(gps_move[:1000, 4], gps_move[:1000, 5], color='black', label='GPS Easting and Northing Data')
plt.plot(pos_ev[:41500], pos_nv[:41500], color='red', label='IMU Integrated Data')
plt.grid()
plt.legend()
plt.title('Position Data compared from Complementary Filter with GPS Readings')
plt.ylabel('Y Position (m)')
plt.xlabel('X Position (m)')
plt.show()

w_xdot = imu_move[:, 7]*acc_x_integrat
plt.close()
plt.plot(imu_move[:, 0], imu_move[:, 9], color='black', label='Linear Acceleration - Y IMU Reading')
plt.plot(imu_move[:, 0], w_xdot, color='red', label='Calculated value')
plt.grid()
plt.legend()
plt.title('Calculated value compared with IMU Reading')
plt.ylabel('Acceleration (m/s\u00b2)')
plt.xlabel('Time (s)')
plt.show()


# Calculating Xc
dtime_moving = np.empty((0))
for i in range(imu_move.shape[0]):
    time_diff = imu_move[i, 0] - imu_move[i-1, 0]
    dtime_moving = np.append(dtime_moving, time_diff)

derived_wz = imu_move[:46416, 7]/dtime_moving
epsilon = 1e-10  # A very small number to avoid division by zero
derived_wz_safe = np.where(derived_wz == 0, epsilon, derived_wz)
Xc_tot = -1 * (w_xdot[:46416] / derived_wz_safe)
Xc_tot = Xc_tot[np.logical_not(np.isnan(Xc_tot))]
Xc = np.mean(Xc_tot)
print(Xc)