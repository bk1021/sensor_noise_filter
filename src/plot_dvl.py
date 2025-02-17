#!/usr/bin/python3
import rospy
from geometry_msgs.msg import TwistStamped
from collections import deque
import matplotlib.pyplot as plt
import numpy as np
import rosgraph
from datetime import datetime, timedelta, timezone
import os
import rospkg


# Parameters for data storage
BUFFER_SIZE = 5000  # Maximum number of data points to store

# Buffers for storing data
time_noisy_buffer = deque(maxlen=BUFFER_SIZE)
time_clean_buffer = deque(maxlen=BUFFER_SIZE)
velocity_x_noisy_buffer = deque(maxlen=BUFFER_SIZE)
velocity_x_clean_buffer = deque(maxlen=BUFFER_SIZE)
velocity_y_noisy_buffer = deque(maxlen=BUFFER_SIZE)
velocity_y_clean_buffer = deque(maxlen=BUFFER_SIZE)
velocity_z_noisy_buffer = deque(maxlen=BUFFER_SIZE)
velocity_z_clean_buffer = deque(maxlen=BUFFER_SIZE)

# Dictionary to store filtered velocity buffers dynamically
filtered_buffers = {}
active_filtered_topics = set()

# Helper function to get the nodes publishing to the specified topic
def get_publisher_nodes(topic_name):
    master = rosgraph.Master('/rostopic')
    state = master.getSystemState()
    publishers = [node for topic, nodes in state[0] if topic == topic_name for node in nodes]
    return publishers

# Helper function to convert ROS timestamp to seconds
def extract_time(stamp):
    return stamp.secs + stamp.nsecs * 1e-9

# Callback for true DVL data
def dvl_callback(msg):
    time = extract_time(msg.header.stamp)
    time_noisy_buffer.append(time)
    velocity_x_noisy_buffer.append(msg.twist.linear.x) 
    velocity_y_noisy_buffer.append(msg.twist.linear.y)
    velocity_z_noisy_buffer.append(msg.twist.linear.z)

def clean_dvl_callback(msg):
    time = extract_time(msg.header.stamp)
    time_clean_buffer.append(time)
    velocity_x_clean_buffer.append(msg.twist.linear.x) 
    velocity_y_clean_buffer.append(msg.twist.linear.y)
    velocity_z_clean_buffer.append(msg.twist.linear.z)

# Callback generator for dynamic filtered topics
def generate_filtered_callback(topic_name):
    if topic_name not in filtered_buffers:
        publishers = get_publisher_nodes(topic_name)
        label = rospy.get_param(f"/{publishers[0]}/label", "Filtered Signal")
        filtered_buffers[topic_name] = {
            "time": deque(maxlen=BUFFER_SIZE),
            "vx": deque(maxlen=BUFFER_SIZE),
            "vy": deque(maxlen=BUFFER_SIZE),
            "vz": deque(maxlen=BUFFER_SIZE),
            "label": label
        }

    def filtered_dvl_callback(msg):
        global shutdown
        if not shutdown:
            time = extract_time(msg.header.stamp)
            filtered_buffers[topic_name]["time"].append(time)
            filtered_buffers[topic_name]["vx"].append(msg.twist.linear.x)
            filtered_buffers[topic_name]["vy"].append(msg.twist.linear.y)
            filtered_buffers[topic_name]["vz"].append(msg.twist.linear.z)

    return filtered_dvl_callback

def monitor_filtered_topics(event):
    global active_filtered_topics
    topic_list = rospy.get_published_topics()
    for topic_name, topic_type in topic_list:
        if topic_name.startswith("/sensor/dvl_filtered") and topic_name not in active_filtered_topics:
            active_filtered_topics.add(topic_name)
            rospy.Subscriber(topic_name, TwistStamped, generate_filtered_callback(topic_name))
            rospy.loginfo(f"Subscribed to new topic: {topic_name}")

def plot_and_evaluate_signals(fs, plot_aligned_filtered_signal=False):
    # Convert buffers to numpy arrays
    time_noisy = np.array(list(time_noisy_buffer))
    time_clean = np.array(list(time_clean_buffer))
    noisy_vx = np.array(list(velocity_x_noisy_buffer))
    noisy_vy = np.array(list(velocity_y_noisy_buffer))
    noisy_vz = np.array(list(velocity_z_noisy_buffer))
    clean_vx = np.array(list(velocity_x_clean_buffer))
    clean_vy = np.array(list(velocity_y_clean_buffer))
    clean_vz = np.array(list(velocity_z_clean_buffer))

    # Store filtered time buffers
    time_filtered_buffers = {}
    for topic_name, filtered_data in filtered_buffers.items():
        time_filtered_buffers[topic_name] = np.array(list(filtered_data["time"]))

    # Find the smallest starting time among all time buffers
    min_time = min(
        np.min(time_noisy),
        np.min(time_clean),
        *(np.min(time) for time in time_filtered_buffers.values())
    )

    # Normalize all time buffers using the smallest time
    time_noisy -= min_time
    time_clean -= min_time
    for topic_name in time_filtered_buffers:
        time_filtered_buffers[topic_name] -= min_time

    snr_x_before, lag_x_before,_ = calculate_snr(clean_vx, noisy_vx)
    snr_y_before, lag_y_before,_ = calculate_snr(clean_vy, noisy_vy)
    snr_z_before, lag_z_before,_ = calculate_snr(clean_vz, noisy_vz)

    # Should be zero lag if no issues
    if lag_x_before > 0:
        rospy.logwarn(f"Lag of {lag_x_before} exists in noisy velocity x before filtering")
    if lag_y_before > 0:
        rospy.logwarn(f"Lag of {lag_y_before} exists in noisy velocity y before filtering")
    if lag_z_before > 0:
        rospy.logwarn(f"Lag of {lag_z_before} exists in noisy velocity z before filtering")

    # Create subplots for X, Y, Z velocities
    fig, axes = plt.subplots(1, 3, figsize=(18, 4), sharex=True)

    # Plot X velocity
    axes[0].plot(time_noisy, noisy_vx, label="Noisy Signal", alpha=0.6)
    axes[0].plot(time_clean, clean_vx, label="Clean Signal", linestyle="--")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Velocity X (m/s)")

    # Plot Y velocity
    axes[1].plot(time_noisy, noisy_vy, alpha=0.6)
    axes[1].plot(time_clean, clean_vy, linestyle="--")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Velocity Y (m/s)")

    # Plot Z velocity
    axes[2].plot(time_noisy, noisy_vz, alpha=0.6)
    axes[2].plot(time_clean, clean_vz, linestyle="--")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("Velocity Z (m/s)")

    for topic_name, filtered_data in filtered_buffers.items():
        time_filtered = time_filtered_buffers[topic_name]
        filtered_vx = np.array(list(filtered_data["vx"]))
        filtered_vy = np.array(list(filtered_data["vy"]))
        filtered_vz = np.array(list(filtered_data["vz"]))
        snr_x, lag_x, aligned_filtered_vx = calculate_snr(clean_vx, filtered_vx)
        snr_y, lag_y, aligned_filtered_vy = calculate_snr(clean_vy, filtered_vy)
        snr_z, lag_z, aligned_filtered_vz = calculate_snr(clean_vz, filtered_vz)
        print(f"Metrics performance of {filtered_data['label']}:")
        print(f"SNR Improvement for Velocity X = {snr_x - snr_x_before} | {snr_x_before} -> {snr_x}")
        print(f"SNR Improvement for Velocity Y = {snr_y - snr_y_before} | {snr_y_before} -> {snr_y}")
        print(f"SNR Improvement for Velocity Z = {snr_z - snr_z_before} | {snr_z_before} -> {snr_z}")
        print(f"Lag for Filtered Velocity X: {lag_x} samples, approximately {lag_x/fs} s")
        print(f"Lag for Filtered Velocity Y: {lag_y} samples, approximately {lag_y/fs} s")
        print(f"Lag for Filtered Velocity Z: {lag_z} samples, approximately {lag_z/fs} s")
        print("\n----------------------------------------------------------------------------------------\n")
        axes[0].plot(time_filtered, filtered_vx, label=filtered_data["label"], linewidth=1.5)
        axes[1].plot(time_filtered, filtered_vy, linewidth=1.5)
        axes[2].plot(time_filtered, filtered_vz, linewidth=1.5)
        if plot_aligned_filtered_signal:
            time_filtered  = time_filtered[:len(aligned_filtered_vx)]
            axes[0].plot(time_filtered, aligned_filtered_vx, label="Aligned Filtered Signal")
            axes[1].plot(time_filtered, aligned_filtered_vy)
            axes[2].plot(time_filtered, aligned_filtered_vz)

    axes[0].legend(loc="best")
    axes[0].grid(True)
    axes[1].grid(True)
    axes[2].grid(True)
    plt.tight_layout()
    rospack = rospkg.RosPack()
    package_path = rospack.get_path("sensor_noise_filter")
    save_dir = os.path.join(package_path, "image_plots")
    os.makedirs(save_dir, exist_ok=True)
    filename = f"dvl_velocity_plot_{datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')}.png"
    file_path = os.path.join(save_dir, filename)
    plt.savefig(file_path)


def calculate_snr(clean_signal, noisy_signal):
    """
    Calculate the Signal-to-Noise Ratio (SNR) in dB.
    
    Args:
        clean_signal (np.ndarray): The original clean signal.
        noisy_signal (np.ndarray): Signal with noise
    
    Returns:
        float: The SNR value in decibels (dB).
    """
    # Ensure both signals have same datapoints, truncate excess datapoints
    if len(noisy_signal) < len(clean_signal):
        # Log a warning and truncate the clean signal
        missing_values = len(clean_signal) - len(noisy_signal)
        rospy.logwarn(f"The noisy signal has {missing_values} fewer samples. "
                      f"Truncating {missing_values} samples from the clean signal.")
        clean_signal = clean_signal[:len(noisy_signal)]
    elif len(noisy_signal) > len(clean_signal):
        # Log a warning and truncate the noisy signal
        missing_values = len(noisy_signal) - len(clean_signal)
        rospy.logwarn(f"The clean signal has {missing_values} fewer samples. "
                      f"Truncating {missing_values} samples from the noisy signal.")
        noisy_signal = noisy_signal[:len(clean_signal)]

    # Finds the optimal lag (if any) of noisy_signal relative to clean_signal using cross-correlation
    correlation = np.correlate(noisy_signal, clean_signal, mode='full')
    lag = np.argmax(correlation) - (len(clean_signal) - 1)
    # Shift the noisy signal to aligns clean signal
    aligned_noisy_signal = np.roll(noisy_signal, -lag)
    
    # Calculate signal power and noise power
    signal_power = np.mean(clean_signal**2)
    if lag > 0:
        # Compensate lag in calculation of noise power
        noise_power = np.mean((aligned_noisy_signal[:-lag] - clean_signal[:-lag])**2)
    elif lag < 0:
        # Compensate lead in calculation of noise power (uncommon)
        noise_power = np.mean((aligned_noisy_signal[lag:] - clean_signal[lag:])**2)
    else:
        noise_power = np.mean((aligned_noisy_signal - clean_signal)**2)
    
    # Avoid division by zero
    if noise_power == 0:
        rospy.logerr("Noise power is zero.")
        return float('inf')  # Infinite SNR
    
    snr = 10 * np.log10(signal_power / noise_power)

    # Calculate and return SNR in decibels
    return snr, lag, aligned_noisy_signal

def shutdown_hook():
    global shutdown
    rospy.loginfo("Shutdown signal received, saving plot...")
    shutdown = True
    fs = rospy.get_param("~fs", 33)
    plot_aligned_filtered_signal = rospy.get_param("~plot_aligned_filtered_signal", True)
    plot_and_evaluate_signals(fs, plot_aligned_filtered_signal)
    rospy.loginfo("Plot saved successfully.")


if __name__ == "__main__":
    shutdown = False
    rospy.on_shutdown(shutdown_hook)
    try:
        rospy.init_node("dvl_plotter", anonymous=True)
        rospy.Subscriber("/sensor/dvl", TwistStamped, dvl_callback)
        rospy.Subscriber("/sensor/dvl_clean", TwistStamped, clean_dvl_callback)
        rospy.Timer(rospy.Duration(0.3), monitor_filtered_topics)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

