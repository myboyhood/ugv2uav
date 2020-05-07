#!/usr/bin/env python

# import necessary module
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np

# load data from file
# you replace this using with open
data_truth = np.loadtxt("truth_pos.txt")
truth_x = data_truth[:, 1]
truth_y = data_truth[:, 2]
truth_z = data_truth[:, 3]
truth_yaw = data_truth[:, 4]
truth_pitch = data_truth[:, 5]
truth_roll = data_truth[:, 6]

data1 = np.loadtxt("ugv1_esti_pos.txt")
ugv1_esti_x = data1[:, 1]
ugv1_esti_y = data1[:, 2]
ugv1_esti_z = data1[:, 3]
ugv1_esti_yaw = data1[:, 4]
ugv1_esti_pitch = data1[:, 5]
ugv1_esti_roll = data1[:, 6]

data2 = np.loadtxt("ugv2_esti_pos.txt")
ugv2_esti_x = data2[:, 1]
ugv2_esti_y = data2[:, 2]
ugv2_esti_z = data2[:, 3]
ugv2_esti_yaw = data2[:, 4]
ugv2_esti_pitch = data2[:, 5]
ugv2_esti_roll = data2[:, 6]

data3 = np.loadtxt("ugv3_esti_pos.txt")
ugv3_esti_x = data3[:, 1]
ugv3_esti_y = data3[:, 2]
ugv3_esti_z = data3[:, 3]
ugv3_esti_yaw = data3[:, 4]
ugv3_esti_pitch = data3[:, 5]
ugv3_esti_roll = data3[:, 6]

data_fuse = np.loadtxt("fuse_esti_pos.txt")
fuse_esti_x = data_fuse[:, 1]
fuse_esti_y = data_fuse[:, 2]
fuse_esti_z = data_fuse[:, 3]
fuse_esti_yaw = data_fuse[:, 4]
fuse_esti_pitch = data_fuse[:, 5]
fuse_esti_roll = data_fuse[:, 6]
# new a figure and set it into 3d
fig = plt.figure()
ax = fig.gca(projection='3d')

# set figure information
ax.set_title("3D_Curve comparision")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

# draw the figure, the color is r = read
figuret_posi = ax.plot(truth_x, truth_y, truth_z, c='b')
figuret_YPR = ax.plot(truth_yaw, truth_pitch, truth_roll, c='b')

figure1_posi = ax.plot(ugv1_esti_x, ugv1_esti_y, ugv1_esti_z, c='r')
figure1_YPR = ax.plot(ugv1_esti_yaw, ugv1_esti_pitch, ugv1_esti_roll, c='r')

figure2_posi = ax.plot(ugv2_esti_x, ugv2_esti_y, ugv2_esti_z, c='g')
figure2_YPR = ax.plot(ugv2_esti_yaw, ugv2_esti_pitch, ugv2_esti_roll, c='g')

figure3_posi = ax.plot(ugv3_esti_x, ugv3_esti_y, ugv3_esti_z, c='y')
figure3_YPR = ax.plot(ugv3_esti_yaw, ugv3_esti_pitch, ugv3_esti_roll, c='y')

figuref_posi = ax.plot(fuse_esti_x, fuse_esti_y, fuse_esti_z, c='k')
figuref_YPR = ax.plot(fuse_esti_yaw, fuse_esti_pitch, fuse_esti_roll, c='k')
plt.show()
