import numpy as np 

x, y, z = np.loadtxt('/media/marzan/mini storage/experimental_kalman/confirm_imu_gps_combination/combined/acceleration.csv', delimiter=',', unpack=True)

x_a = np.mean(x)
y_a = np.mean(y)
z_a = np.mean(z)

print("x_a, y_a, z_a", x_a, y_a, z_a)
