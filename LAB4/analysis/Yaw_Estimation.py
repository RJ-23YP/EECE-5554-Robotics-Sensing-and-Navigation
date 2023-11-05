import pandas as pd
import math
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
gps_stationary = gpsdata_numarr[:40, :] #When the car is stationary

imu_move_first = np.argmin(np.abs(gpsdata_numarr[350, 0] - imudata_numarr[:, 0]))
imu_move = imudata_numarr[imu_move_first:, :]

imu_round_first = np.argmin(np.abs(gpsdata_numarr[55, 0] - imudata_numarr[:, 0]))
imu_round_last = np.argmin(np.abs(gpsdata_numarr[200, 0] - imudata_numarr[:, 0]))
imu_round = imudata_numarr[imu_round_first:imu_round_last, :]

imu_stationary_first = np.argmin(np.abs(gpsdata_numarr[0, 0] - imudata_numarr[:, 0]))
imu_stationary_last = np.argmin(np.abs(gpsdata_numarr[40, 0] - imudata_numarr[:, 0]))
imu_stationary = imudata_numarr[imu_stationary_first:imu_stationary_last, :]


# Plotting uncorrected magnetometer data
plt.plot(imu_round[:, 11], imu_round[:, 12])
plt.grid()
plt.xlabel('Magnetic Field - X (Gauss)')
plt.ylabel('Magnetic Field - Y (Gauss)')
plt.title('Uncorrected Magnetometer Data')
plt.show()

# Hard Iron Calibration
meanx_imu_round = np.mean(imu_round[:, 11])
meany_imu_round = np.mean(imu_round[:, 12])
magx_hard_corr = imu_round[:, 11] - meanx_imu_round
magy_hard_corr = imu_round[:, 12] - meany_imu_round
plt.plot(imu_round[:, 11], imu_round[:, 12], color='violet', label='Uncorrected Data')
plt.plot(magx_hard_corr, magy_hard_corr, color='yellow', label='Hard Iron Corrected Data')
plt.grid()
plt.legend()
plt.xlabel('Magnetic Field - X (Gauss)')
plt.ylabel('Magnetic Field - Y (Gauss)')
plt.title('Magnetic Field Y vs X')
plt.show()

#Soft Iron Calibration
majoraxis_x = np.argmax(np.abs(magx_hard_corr))
majoraxis_y = np.argmax(np.abs(magy_hard_corr))

circle_radius = math.sqrt((majoraxis_x**2) + (majoraxis_y**2))
print('radius of the circle = ', circle_radius)
angle = np.arcsin((majoraxis_y/circle_radius))
print('angle = ', angle)

Rot_mat = [[np.cos(angle), np.sin(angle)], [np.sin(-angle), np.cos(angle)]]
vec = [magx_hard_corr, magy_hard_corr]

matrix = np.matmul(Rot_mat, vec)
print(np.shape(matrix))
plt.grid()
plt.plot(matrix[0], matrix[1], label = 'Soft-Iron Calibrated', color='red')
circ = plt.Circle((0.0, 0.0), 0.1, fill=False, color='green', label='True value')
plt.gca().add_patch(circ)
plt.gca().set_aspect("equal")
plt.title('Soft_Iron_Calibration Of Magnetic Field X vs Y')
plt.xlabel('Soft_Iron_X (Guass)')
plt.ylabel('Soft_Iron_Y (Guass)')
plt.legend()
plt.show()

#Final Calibration of Magnetometer Data
r = 0.1 #Length of major axis from the graph
q = 0.075 #Length of minor axis from the graph
sigma = q/r
print('sigma = ', sigma)
matrix2 = [[1, 0], [0, sigma]]
rotate = np.matmul(matrix2, matrix)
angle = -angle
Rot_mat1 = [[np.cos(angle), np.sin(angle)], [np.sin(-angle), np.cos(angle)]]
vec1 = np.matmul(Rot_mat1, rotate)
print(np.shape(vec1))
plt.grid()
plt.plot(vec1[0], vec1[1], label='Hard and Soft Iron Calibrated Data', color='green')
circ = plt.Circle((0.0, 0.0), 0.075, fill=False, color='red', label = 'True value')
plt.gca().add_patch(circ)
plt.gca().set_aspect("equal")
plt.title('Calibrated Plot for Magnetic Field X vs Y')
plt.xlabel('Magnetic Field X (Guass)')
plt.ylabel('Magmetic Field Y (Guass)')
plt.legend()
plt.show()


#Obtaining yaw from calibrated magnetometer data

#Quaternion to Euler Angles Conversion
X = imu_round[:, 1]
Y = imu_round[:, 2] 
Z = imu_round[:, 3] 
W = imu_round[:, 4]

a3 = +2.0 * (W * Z + X * Y)
a4 = +1.0 - 2.0 * (Y * Y + Z * Z)
yaw = np.arctan2(a3, a4)

mag_yaw_cor = np.arctan2(vec1[0], vec1[1]) #calculate yaw from the corrected magnetometer data
integrat_yaw_corr = integrate.cumtrapz(imu_round[:, 7], imu_round[:, 0], initial=0)

#Subtract the bias
integrat_yaw_corr = integrat_yaw_corr - (integrat_yaw_corr[0] - mag_yaw_cor[0])

def piwrap(data):
    wrapda = np.remainder(data, 2 * np.pi)
    mask = np.abs(wrapda) > np.pi
    wrapda[mask] -= 2 * np.pi * np.sign(wrapda[mask])
    return wrapda

rel_imu_round = imu_round - imu_round[0]
integrat_yaw_corr_final = piwrap(integrat_yaw_corr)
plt.plot(rel_imu_round[:, 0], integrat_yaw_corr_final, color='violet', label='Yaw integrated from angular velocity')
plt.plot(rel_imu_round[:, 0], mag_yaw_cor, color='orange', label='Yaw from magnetometer')
plt.grid()
plt.legend()
plt.xlabel('Time (seconds)')
plt.ylabel('Yaw (radian)')
plt.title('Yaw calcualted from corrected magnetometer data vs Yaw calculated from integrating angular velocity')
plt.show()

# Complementary filter
mag_yaw_cor_unwrapped = np.unwrap(mag_yaw_cor)
comple_filter = 0.9*integrat_yaw_corr + 0.1*mag_yaw_cor_unwrapped
comple_filter_final = piwrap(comple_filter)
plt.plot(rel_imu_round[:, 0], integrat_yaw_corr_final, color='violet', label='Yaw integrated from angular velocity')
plt.plot(rel_imu_round[:, 0], mag_yaw_cor, color='orange', label='Yaw from magnetometer')
plt.plot(rel_imu_round[:, 0], comple_filter_final, color='red', label='Complementary Filter')
plt.grid()
plt.legend()
plt.xlabel('Time (seconds)')
plt.ylabel('Yaw (radian)')
plt.title('Comparison of Corrected Yaw Data & Integrated Yaw Data with Complementary Filter')
plt.show()

#Complementary Filter Output compared with Yaw from IMU
corr_comp_final_unwrap = np.unwrap(comple_filter_final)
corr_comp_final_unwrap = corr_comp_final_unwrap - (corr_comp_final_unwrap[0]-yaw[0])
comple_filter_final_wrapped = piwrap(corr_comp_final_unwrap)
plt.plot(rel_imu_round[:, 0], yaw, color='green', label='Raw Yaw')
plt.plot(rel_imu_round[:, 0], comple_filter_final_wrapped, color='red', label='Complementary Filter')
plt.grid()
plt.legend()
plt.xlabel('Time (seconds)')
plt.ylabel('Yaw (radian)')
plt.title('Comparison of IMU Yaw Data with Complementary Filter')
plt.show()