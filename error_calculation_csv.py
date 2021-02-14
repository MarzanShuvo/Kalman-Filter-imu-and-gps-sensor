import serial
import time
import pandas as pd
import numpy as np
import math
import csv
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
    modified_acc[0] = modified_acc_mag_bias[0]* math.cos(0.43)+modified_acc_mag_bias[1]*math.sin(0.43)
    modified_acc[1] = modified_acc_mag_bias[1]*math.cos(0.43)- modified_acc_mag_bias[0]*math.sin(0.43)
    modified_acc[2] = modified_acc_mag_bias[2]
    return modified_acc

print("wave like 8:")
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
    print("wave like 8: ")
    try:
        modified_acceleration = Modified_acceleration(quaternion, acceleration)
        with open("/media/marzan/mini storage/experimental_kalman/confirm_imu_gps_combination/combined/acceleration.csv", "a")as output:
            writer = csv.writer(output, delimiter=",")
            writer.writerow(modified_acceleration)

    except:
        continue
    