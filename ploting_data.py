import serial
import time
import pandas as pd
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
try: 
    arduino = serial.Serial("/dev/ttyACM0",115200)
except:
    print("Please Check the port")

prev_position_x =0.0
prev_position_y =0.0
prev_position_z =0.0

prev_velocity_x= 0.0
prev_velocity_y = 0.0
prev_velocity_z = 0.0

x_position = []
y_position = []
z_position = []
fig = plt.figure()

def update_lines(num):
    global prev_position_x, prev_position_y, prev_position_z, prev_velocity_x, prev_velocity_y, prev_velocity_z
    x_p, y_p, z_p, x_v, y_v, z_v = get_position(prev_position_x, prev_position_y, prev_position_z, prev_velocity_x, prev_velocity_y, prev_velocity_z) 
    prev_position_x = x_p
    prev_position_y = y_p
    prev_position_z = z_p
    prev_velocity_x = x_v
    prev_velocity_y = y_v
    prev_velocity_z = z_v

    x_position.append(np.array(x_p))
    y_position.append(np.array(y_p))
    z_position.append(np.array(z_p))

    ax = fig.add_subplot(111, projection='3d')
    ax.clear()
    graph = ax.plot3D(x_position,y_position,z_position,color='orange',marker='o')
    return graph

def lowpass_filter(acc):
    thres = 0.098
    if(abs(acc[0])<thres):
        acc[0]=0
    if(abs(acc[1])<thres):
        acc[1] = 0
    if(abs(acc[2])<thres):
        acc[2] = 0
    return acc



def Rotation_matrix(q):
    temp = np.zeros((3,3))
    temp[0][0] = 2*(q[0]**2+q[1]**2)-1
    temp[0][1] = 2*(q[1]*q[2]-q[0]*q[3])
    temp[0][2] = 2*(q[1]*q[3]+q[0]*q[2])
    temp[1][0] = 2*(q[1]*q[2]+q[0]*q[3])
    temp[1][1] = 2*(q[0]**2+q[2]**2)-1
    temp[1][2] = 2*(q[2]*q[3]-q[0]*q[1])
    temp[2][0] = 2*(q[1]*q[3]-q[0]*q[2])
    temp[2][1] = 2*(q[2]*q[3]+q[0]*q[1])
    temp[2][2] = 2*(q[0]**2+q[3]**2)-1
    return temp

def Modified_acceleration(quaternion, acceleration):
    numpy_acc = np.array(acceleration)
    rotation_matix = Rotation_matrix(quaternion)
    modified_acc_mag_bias = np.dot(np.linalg.inv(rotation_matix),numpy_acc.T)
    modified_acc = modified_acc_mag_bias
    modified_acc[2] = modified_acc[2]-1
    '''modified_acc[0] = modified_acc_mag_bias[0]* math.cos(0.43)+modified_acc_mag_bias[1]*math.sin(0.43)
    modified_acc[1] = modified_acc_mag_bias[1]*math.cos(0.43)- modified_acc_mag_bias[0]*math.sin(0.43)
    modified_acc[2] = modified_acc_mag_bias[2]-1'''
    modified_acc = lowpass_filter(modified_acc)
    return modified_acc

def get_position(prev_position_x, prev_position_y, prev_position_z, prev_velocity_x, prev_velocity_y, prev_velocity_z ):
    while True:
        start_time = time.time()
        a = str(arduino.readline())
        temp = a[2:-5]
        x = temp.split(",")
        try:
            x = list(map(float, x))
        except:
            continue
        acceleration = x[0:3]
        y_p_r = x[3:6]
        quaternion = x[6:10]
        try:
            end_time = time.time()
            delta_t = end_time-start_time
            modified_acceleration = Modified_acceleration(quaternion, acceleration)
            position_x = prev_position_x+modified_acceleration[0]*delta_t+0.5*modified_acceleration[0]*delta_t**2
            position_y = prev_position_y+modified_acceleration[1]*delta_t+0.5*modified_acceleration[1]*delta_t**2
            position_z = prev_position_z+modified_acceleration[2]*delta_t+0.5*modified_acceleration[2]*delta_t**2

            velocity_x = prev_velocity_x+ modified_acceleration[0]*delta_t
            velocity_y = prev_velocity_y+ modified_acceleration[1]*delta_t
            velocity_z = prev_velocity_z +modified_acceleration[2]*delta_t

            return position_x, position_y, position_z, velocity_x, velocity_y, velocity_z
        except:
            continue


ani = animation.FuncAnimation(fig, update_lines, interval=10)
plt.show()