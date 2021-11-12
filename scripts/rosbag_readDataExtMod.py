import rosbag
import numpy as np
import argparse
import os
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

# This script refer to the path folder defined below, it does not apply to other data folder path
path = '../kratos_quat_crazyflie_dataset/data_fly/DataExtMod/record_1_11122021.bag'

bag = rosbag.Bag(path)

data_ext_mod = []

for topic, msg, t in bag.read_messages(topics=['/crazyflie/data_ext_mod']):
    data_ext_mod.append(msg)

data_ext_mod_list = []
quat_list = []
pos_list = []
pos_target_list = []

## Preprocessing data
for i in range(len(data_ext_mod)):
    data_ext_mod_list.append([data_ext_mod[i].header.stamp.to_nsec()/1e9, data_ext_mod[i].conPadM3, data_ext_mod[i].conPadM4,data_ext_mod[i].Pterm, data_ext_mod[i].Iterm, -data_ext_mod[i].Dterm, data_ext_mod[i].heading, data_ext_mod[i].padDir, data_ext_mod[i].padGain, data_ext_mod[i].flapM1, data_ext_mod[i].flapM2])
    quat_list.append([data_ext_mod[i].poseCurrent.orientation.w,data_ext_mod[i].poseCurrent.orientation.x,data_ext_mod[i].poseCurrent.orientation.y,data_ext_mod[i].poseCurrent.orientation.z])
    pos_list.append([data_ext_mod[i].poseCurrent.position.x,data_ext_mod[i].poseCurrent.position.y,data_ext_mod[i].poseCurrent.position.z])
    pos_target_list.append([data_ext_mod[i].posTarget.x,data_ext_mod[i].posTarget.y,data_ext_mod[i].posTarget.z])

    


data_ext_mod_arr = np.array(data_ext_mod_list)

findFirst = np.where(data_ext_mod_arr[:,0] > 0)[0][0]

data_ext_mod = data_ext_mod_arr[findFirst:,:]
data_ext_mod[:,0] = data_ext_mod[:,0]-data_ext_mod[0,0]

quat = np.array(quat_list)
quat = quat[findFirst:,:] # w x y z

pos = np.array(pos_list)
pos = pos[findFirst:,:]

pos_target = np.array(pos_target_list)
pos_target = pos_target[findFirst:,:]

ts = data_ext_mod[:,0]
conPadM3 = data_ext_mod[:,1]
conPadM4 = data_ext_mod[:,2]
Pterm = data_ext_mod[:,3]
Iterm = data_ext_mod[:,4]
Dterm = data_ext_mod[:,5]
heading = data_ext_mod[:,6]
padDir = data_ext_mod[:,7]
padGain = data_ext_mod[:,8]
flapM1 = data_ext_mod[:,9]                                                                                                
flapM2 = data_ext_mod[:,10]


## Plots for rosbag data
# fig, ax = plt.subplots(5,1,sharey=False)

# ax[0].plot(ts,conPadM3,'r--')
# ax[0].plot(ts,conPadM4,'b--')
# ax[1].plot(ts,pos_target[:,0],'g--')
# ax[1].plot(ts,pos[:,0])
# ax[2].plot(ts,pos_target[:,1],'g--')
# ax[2].plot(ts,pos[:,1])
# ax[3].plot(ts,pos_target[:,2],'g--')
# ax[3].plot(ts,pos[:,2])
# ax[4].plot(ts,flapM1,'r--')
# ax[4].plot(ts,flapM2,'b--')



# ax[0].legend(['M3','M4'])
# ax[1].legend(['X_target','X_current'])
# ax[2].legend(['Y_target','Y_current'])
# ax[3].legend(['Z_target','Z_current'])
# ax[4].legend(['flapM1','flapM2'])




# ax[0].grid(True)
# ax[1].grid(True)
# ax[2].grid(True)
# ax[3].grid(True)
# ax[4].grid(True)




# ax[0].set_ylabel('ConPad')
# ax[1].set_ylabel('X axis(cm)')
# ax[2].set_ylabel('Y axis(cm)')
# ax[3].set_ylabel('Z axis(cm)')
# ax[4].set_ylabel('flap')



# plt.show()
