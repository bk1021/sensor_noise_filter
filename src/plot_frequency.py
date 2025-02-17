#!/usr/bin/python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from geometry_msgs.msg import TwistStamped

# Helper function to convert ROS timestamp to seconds
def extract_time(stamp):
    return stamp.secs + stamp.nsecs * 1e-9

# Callback for true DVL data
def dvl_callback(msg):
    time = extract_time(msg.header.stamp)
    time_buffer.append(time)
    velocity_x_buffer.append(msg.twist.linear.x) 
    velocity_y_buffer.append(msg.twist.linear.y)
    velocity_z_buffer.append(msg.twist.linear.z)

def plot_frequency(fs):
    time = np.array(list(time_buffer))
    time -= time[0]
    velocity_x = np.array(list(velocity_x_buffer))
    velocity_y = np.array(list(velocity_y_buffer))
    velocity_z = np.array(list(velocity_z_buffer))

    fft_x = np.fft.fft(velocity_x)
    fft_y = np.fft.fft(velocity_y)
    fft_z = np.fft.fft(velocity_z)
    freq_x = np.fft.fftfreq(len(time), 1/fs)
    freq_y = np.fft.fftfreq(len(time), 1/fs)
    freq_z = np.fft.fftfreq(len(time), 1/fs)

    # Only plot the positive half of the spectrum (real frequencies)
    fft_x = np.abs(fft_x)[:len(fft_x)//2]
    freq_x = freq_x[:len(freq_x)//2]
    fft_y = np.abs(fft_y)[:len(fft_y)//2]
    freq_y = freq_y[:len(freq_y)//2]
    fft_z = np.abs(fft_z)[:len(fft_z)//2]
    freq_z = freq_z[:len(freq_z)//2]  

    fig, axes = plt.subplots(1, 3, figsize=(18, 4), sharex=True)
    axes[0].plot(freq_x, fft_x)
    axes[0].set_xlabel("Frequency (Hz)")
    axes[0].set_ylabel("Magnitude")
    axes[0].grid(True)

    axes[1].plot(freq_y, fft_y)
    axes[1].set_xlabel("Frequency (Hz)")
    axes[1].set_ylabel("Magnitude")
    axes[1].grid(True)

    axes[2].plot(freq_z, fft_z)
    axes[2].set_xlabel("Frequency (Hz)")
    axes[2].set_ylabel("Magnitude")
    axes[2].grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    try:        
        BUFFER_SIZE = 5000  # Maximum number of data points to store
        time_buffer = deque(maxlen=BUFFER_SIZE)
        velocity_x_buffer = deque(maxlen=BUFFER_SIZE)
        velocity_y_buffer = deque(maxlen=BUFFER_SIZE)
        velocity_z_buffer = deque(maxlen=BUFFER_SIZE)

        rospy.init_node("frequency_plotter", anonymous=True)
        rospy.Subscriber("/sensor/dvl", TwistStamped, dvl_callback)
        rospy.spin()
    except rospy.ROSInterruptException():
        pass
    finally:
        plot_frequency(fs=33)
