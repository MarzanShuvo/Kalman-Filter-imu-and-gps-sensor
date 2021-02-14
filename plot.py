import numpy as np 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

x, y, z = np.loadtxt('/media/marzan/mini storage/experimental_kalman/confirm_imu_gps_combination/combined/position.csv', delimiter=',', unpack=True)


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot3D(x,y,z,color='orange',marker='o')
plt.show()