import serial
import time
import csv
import numpy as np
import math
import sys

try: 
    arduino = serial.Serial("/dev/ttyACM0",115200)
except:
    print("Please Check the port")

prev_position_x =0
prev_position_y =0
prev_position_z =0

prev_velocity_x= 0
prev_velocity_y = 0
prev_velocity_z = 0

position = []


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
    '''modified_acc[0] = modified_acc_mag_bias[0]* math.cos(0.43)+modified_acc_mag_bias[1]*math.sin(0.43)
    modified_acc[1] = modified_acc_mag_bias[1]*math.cos(0.43)- modified_acc_mag_bias[0]*math.sin(0.43)'''
    modified_acc[2] = modified_acc_mag_bias[2]-1
    modified_acc = lowpass_filter(modified_acc)
    return modified_acc

print("wave like 8: ")
while True:
    start_time = time.time()
    a = str(arduino.readline())
    temp = a[2:-5]
    x = temp.split(",")
    end_time = time.time()
    delta_t = end_time-start_time
    try:
        x = list(map(float, x))
    except:
        continue
    acceleration = x[0:3]
    y_p_r = x[3:6]
    quaternion = x[6:10]
    try:
        modified_acceleration = Modified_acceleration(quaternion, acceleration)
        position_x = prev_position_x+modified_acceleration[0]*delta_t+0.5*modified_acceleration[0]*delta_t**2
        position_y = prev_position_y+modified_acceleration[1]*delta_t+0.5*modified_acceleration[1]*delta_t**2
        position_z = prev_position_z+modified_acceleration[2]*delta_t+0.5*modified_acceleration[2]*delta_t**2

        velocity_x = prev_velocity_x+ modified_acceleration[0]*delta_t
        velocity_y = prev_velocity_y+ modified_acceleration[1]*delta_t
        velocity_z = prev_velocity_z +modified_acceleration[2]*delta_t

        prev_position_x = position_x
        prev_position_y = position_y
        prev_position_z = position_z

        prev_velocity_x = velocity_x
        prev_velocity_y = velocity_y
        prev_velocity_z = velocity_z

        '''print("Actual_acceleration: ", acceleration)'''
        print("Modifed_acceleration: ", modified_acceleration)
        pos = [position_x, position_y, 0]

        with open("/media/marzan/mini storage/experimental_kalman/confirm_imu_gps_combination/combined/position.csv", "a")as output:
            writer = csv.writer(output, delimiter=",")
            writer.writerow(pos)


    except:
        continue
    
    #gps = x[10:13]
    """***print("acceleration, yaw_pitch_roll,quaternion ", acceleration)
    print("yaw_pitch_roll: ", y_p_r)
    print("quaternion", quaternion)"""
    #print("acceleration, yaw_pitch_roll,quaternion", acceleration, y_p_r, quaternion)
    """try:
        if(gps[0] !=0):
            print("acceleration, yaw_pitch_roll,quaternion, GPS:", acceleration, y_p_r, quaternion, gps)
    except:
        continue"""