import pandas as pd
import numpy as np
from scipy import integrate
from bagpy import bagreader
import matplotlib.pyplot as plt
import math
from scipy.spatial.transform import Rotation as R

# Read the rosbag file and convert it to csv, and later to a numpy array for further processing. 
bag = bagreader('/home/rj/lab4_ws/analysis/data/lab4.bag')

gpsdata = bag.message_by_topic('/gps')
gpsreadings = pd.read_csv(gpsdata)
imudata = bag.message_by_topic('/imu')
imureadings = pd.read_csv(imudata)

# Have time in seconds with nonoseconds precision.
imut = imureadings["Header.stamp.secs"]  + (imureadings["Header.stamp.nsecs"]*1e-9)
imu_time = imut - imut[0]
imu_time = imu_time.to_numpy()

gpst = gpsreadings["header.stamp.secs"] + (gpsreadings["header.stamp.nsecs"]*1e-9)
gps_time = gpst - gpst[0]
gps_time = gps_time.to_numpy()

q = np.array([imureadings["IMU.orientation.x"].transpose(), imureadings["IMU.orientation.y"], imureadings["IMU.orientation.z"], imureadings["IMU.orientation.w"]]).transpose()

p = R.from_quat(q).as_euler('xyz', degrees=False)[:, 1]

utme = gpsreadings["utm_easting"].to_numpy()
utmn = gpsreadings["utm_northing"].to_numpy()

utme = utme - utme[0]
utmn = utmn - utmn[0]

dist = np.zeros(len(utme))
for i in range(len(utme)):
    dist[i] = dist[i-1] + np.sqrt((utme[i] - utme[i-1])**2 + (utmn[i] - utmn[i-1])**2)

vel = np.zeros(len(utme))
dt = np.diff(gps_time)
for i in range(1, len(utme)):
    vel[i] = (dist[i] - dist[i-1]) / dt[i-1]

lax = imureadings["IMU.linear_acceleration.x"].to_numpy()

imu_vel = (integrate.cumtrapz(lax, imu_time, initial=0)) - np.sin(p)*9.81

plt.figure( figsize=(10, 5) )
plt.plot(gps_time, vel, label="GPS Velocity", color="red")
plt.plot(imu_time, imu_vel, label="IMU Velocity", color="blue")
plt.xlabel("Time (seconds)")
plt.ylabel("Velocity (metres/second)")
plt.title("GPS Velocity vs IMU Velocity")
plt.legend()
plt.show()

lax_1 = imureadings["IMU.linear_acceleration.x"].to_numpy().copy()
jerk = np.diff(lax_1, n=1, axis=0) / np.diff(imu_time, n=1, axis=0)

laz, temp = [], []
for i in range(1, len(vel)):
    if np.abs(jerk[i]) <= 10:
        temp.append(i)
    else:
        if len(temp) > 0:
            laz.append(temp)
            temp = []
            

st, et = [], []
for i in laz:
    st.append(imu_time[i[0]])
    et.append(imu_time[i[-1]])

sti, eti = [], []
for i in range(len(st)):
    sti.append(np.argmin(np.abs(imu_time - st[i])))
    eti.append(np.argmin(np.abs(imu_time - et[i])))

lax_2 = imureadings["IMU.linear_acceleration.x"].to_numpy().copy()
lax_2 = lax_2 - np.mean(lax_2[0:440])

for i in range(1, len(sti)):
    s, e = sti[i], eti[i]
    if s < e:
        m = np.mean(lax_2[s:e], axis=0)
        lax_2[s:e] -= m

corr_vel = (integrate.cumtrapz(lax_2, imu_time, initial=0)) - np.sin(p)*9.81

plt.figure( figsize=(10, 5) )
plt.plot(gps_time, vel, label="GPS Velocity", color="red")
plt.plot(imu_time, corr_vel, label="Corrected IMU Velocity", color="green", linewidth=2)
plt.xlabel("Time (seconds)")
plt.ylabel("Velocity (metres/second)")
plt.title("GPS Velocity vs Corrected IMU Velocity")
plt.legend()
plt.show()
