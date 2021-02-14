import serial
import time
import pandas as pd
import numpy as np
import math
import sys

try: 
    arduino = serial.Serial("/dev/ttyACM0",115200)
except:
    print("Please Check the port")




position = []

# for filtering out the acceleration data. like acc[0 0 0] for rest position
def lowpass_filter(acc):
    thres = 0.098
    if(abs(acc[0])<thres):
        acc[0]=0
    if(abs(acc[1])<thres):
        acc[1] = 0
    if(abs(acc[2])<thres):
        acc[2] = 0
    return acc

#  converting quaternion to rotation
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

#input is quaternion and acceleration
def Modified_acceleration(quaternion, acceleration):
    numpy_acc = np.array(acceleration) # numpy convertion
    rotation_matix = Rotation_matrix(quaternion) # rotating the quaternions
    modified_acc_mag_bias = np.dot(np.linalg.inv(rotation_matix),numpy_acc.T) # converting to NED co-ordinate
    modified_acc = modified_acc_mag_bias
    modified_acc[2] = modified_acc[2]-1 # deleting the gravity
    '''modified_acc[0] = modified_acc_mag_bias[0]* math.cos(0.43)+modified_acc_mag_bias[1]*math.sin(0.43)+0.0691 #average_error_value
    modified_acc[1] = modified_acc_mag_bias[1]*math.cos(0.43)- modified_acc_mag_bias[0]*math.sin(0.43)+0.0674 #average error value
    modified_acc[2] = modified_acc_mag_bias[2]-0.87251 #average error value'''
    modified_acc = lowpass_filter(modified_acc) # filtering out the noise.
    return modified_acc

def measurement_update(p_cov, z, x):
    H = np.eye(6) #declering the zk = Hx+R----> H
    R = np.zeros((6,6)) # noise co varience
    R[0:3, 0:3] = np.eye(3)*(measure_var**2)
    z_k = z+[0, 0, 0] 
    z_k = np.array([z_k])
    y = z_k.T-H.dot(x)
    update = H.dot(p_cov.dot(H.T)) + R
    # here is the problem that is occurings
    try:
        inver = np.linalg.inv(update)
        #K = p_cov.dot(H.T.dot(np.linalg.inv(H.dot(p_cov.dot(H.T)) + R)))
    except Exception as e:
        print("Oops!", e.__class__, "occurred.")

    K = p_cov.dot(H.T.dot(inver))
    p = (np.eye(6)-K.dot(H)).dot(p_cov)
    x = x+K.dot(y)
    
    return p, x


'''print(y)
    update = H.dot(p_cov.dot(H.T)) + R
    print(update)
    
    try:
        inver = np.linalg.inv(update)
        #K = p_cov.dot(H.T.dot(np.linalg.inv(H.dot(p_cov.dot(H.T)) + R)))
    except Exception as e:
        print("Oops!", e.__class__, "occurred.")
    K = p_cov.dot(H.T.dot(inver))
    
    x = x+K.dot(y)
    p = (np.eye(6)-K.dot(H)).dot(p_cov)'''

process_var = 0.03 # process  noice varience
measure_var = 5 # measurement noice varience
x = np.array([[23.971557, 90.361190, 48.500000, 0, 0, 0]]).T
p = np.eye(6) # covarience
p[0:3, 0:3] = np.eye(3)*4**2
p[3:6, 3:6] = np.eye(3)*0.4


while True:
    start_time = time.time()
    a = str(arduino.readline())
    temp = a[2:-5]
    X = temp.split(",")
    try:
        X = list(map(float, X))
    except:
        continue
    acceleration = X[0:3] # acceleration data extracted from the imu sensor using arduino
    y_p_r = X[3:6] #yaw, roll and pitch data is extracted from the imu sensor using arduino
    quaternion = X[6:10] # quaternion data is extracted from the imu sensor using arduino
    gps = X[10:13] # gps data is extracted from gps sensor, latitude, longitude and altitude                         
    #X[10:13]
    
    try:
        #-------------------- prediction steps--------------------------------------
        modified_acceleration = Modified_acceleration(quaternion, acceleration) #NED acceleration
        end_time = time.time()
        delta_t = end_time-start_time
        
        F = np.eye(6)
        F[0:3, 3:6] = np.eye(3)*delta_t
        B = np.vstack((np.eye(3), np.eye(3)))
        B[0:3, 0:3] = np.eye(3)*.5*delta_t**2
        B[3:6, 0:3] = np.eye(3)*delta_t

        Q = np.zeros((6, 6))
        Q[0:3, 0:3] = 0.25*np.eye(3)* np.power(delta_t, 4)*(process_var**2)
        Q[3:6, 0:3] = process_var*np.eye(3)*delta_t**2
        
        x = F.dot(x)+B.dot(np.array([modified_acceleration]).T) # X = F*x+B*u
        p = F.dot(p).dot(F.T)+Q #p = F*p*transpose(F) + Q

        try:
            if(gps[0] !=0):
                #--------------------------updating steps----------------------------------
                p, x = measurement_update(p, gps, x)
                print(p)
               
        except:
            continue

    except:
        continue
    
