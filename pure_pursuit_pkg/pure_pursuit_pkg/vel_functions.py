import numpy as np
import scipy.ndimage
import matplotlib.pyplot as plt


def smooth_velocity_profile(velocities, kernel_size=30):
    # Smooths the velocity profile
    k = np.ones(int(kernel_size))
    k = k / np.sum(k)
    return scipy.ndimage.convolve(velocities, k, mode='wrap')


def smooth_forward(velocities, kernel_size=30):
    # Smooths the velocity profile
    k = np.hstack((np.zeros(int(kernel_size / 2)), np.ones(int(kernel_size / 2))))
    k = k / np.sum(k)
    return scipy.ndimage.convolve(velocities, k, mode='wrap')

def smooth_backward(velocities, kernel_size=30):
    # Smooths the velocity profile
    k = np.hstack((np.ones(int(kernel_size / 2)), np.zeros(int(kernel_size / 2))))
    k = k / np.sum(k)
    return scipy.ndimage.convolve(velocities, k, mode='wrap')

def derivative(velocities, kernel_size=30):
    k = np.hstack((np.ones(int(kernel_size / 2)), -1 * np.ones(int(kernel_size / 2))))
    k = k / np.sum(abs(k))
    return scipy.ndimage.convolve(velocities, k, mode='wrap')


vel_array = np.loadtxt('/home/ladavisiv/school/ese615/final_project/Final_Race_F1_tenth/pure_pursuit/scripts/sample_velocity.csv', delimiter=',')


vel_smooth = smooth_velocity_profile(vel_array)
vel_smooth_forward = smooth_forward(vel_smooth)
vel_smooth_backward = smooth_backward(vel_smooth)
derivative_vel = derivative(vel_array)
speed_up = derivative_vel > 0

vel_new = vel_smooth.copy()
vel_new[speed_up] = vel_smooth_backward[speed_up]
vel_new[~speed_up] = vel_smooth_forward[~speed_up]
vel_new = smooth_velocity_profile(vel_new)

(fig, axes) = plt.subplots(nrows=1, ncols=1, sharex=True, num='Velocity')

ax = axes
ax.plot(vel_smooth, 'r', linewidth=1, alpha=0.5, label="Vel Filtered")
ax.plot(vel_smooth_forward, 'g', linewidth=1, alpha=0.5, label="Vel Filtered Forward")
ax.plot(vel_smooth_backward, 'b', linewidth=1, alpha=0.5, label="Vel Filtered Backward")
ax.plot(derivative_vel, 'k', linewidth=1, alpha=0.5, label="Vel Derivative")
ax.set_ylabel('Velocity, m/s')
ax.grid('major')
ax.set_title('Velocity')
ax.legend(loc='upper right')


(fig, axes) = plt.subplots(nrows=1, ncols=1, sharex=True, num='Velocity New')

ax = axes
ax.plot(vel_array, 'r', linewidth=1, alpha=0.5, label="Vel Original")
ax.plot(vel_new, 'g', linewidth=1, alpha=0.5, label="Vel new")
ax.set_ylabel('Velocity, m/s')
ax.grid('major')
ax.set_title('Velocity')
ax.legend(loc='upper right')
plt.show()
print("Done!")


