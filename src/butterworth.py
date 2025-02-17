#!/usr/bin/python3
import rospy
import numpy as np
from scipy.signal import butter, lfilter
from geometry_msgs.msg import TwistStamped
from rosgraph_msgs.msg import Clock


class butterworth_filter():
    def __init__(self, cutoff=10, fs=33, order=4):
        # Calculate coefficients b, a
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        self.b, self.a = butter(order, normal_cutoff, btype='low', analog=False)
        self.zi = np.zeros(len(self.a) - 1)

    def filter(self, input_value):
        '''
        Perform butterworth filtering and return filtered value
        '''
        filtered_value, self.zi = lfilter(self.b, self.a, [input_value], zi=self.zi)
        return filtered_value


current_secs = 0
current_nsecs = 0


def clock_callback(msg):
    global current_secs, current_nsecs
    current_secs = msg.clock.secs
    current_nsecs = msg.clock.nsecs

def sensor_callback(msg):
    global current_secs, current_nsecs
    filtered_msg = TwistStamped()
    filtered_msg.header.stamp.secs = current_secs
    filtered_msg.header.stamp.nsecs = current_nsecs
    filtered_msg.header.frame_id = "map"
    
    filtered_msg.twist.linear.x = bf_x.filter(msg.twist.linear.x)
    filtered_msg.twist.linear.y = bf_y.filter(msg.twist.linear.y)
    filtered_msg.twist.linear.z = bf_z.filter(msg.twist.linear.z)
    
    # Publish the filtered message
    filtered_sensor_publisher.publish(filtered_msg)


if __name__ == "__main__":
    try:
        # Setup ros node and topics
        rospy.init_node("butterworth", anonymous=True)
        input_sensor_topic = rospy.get_param("~input_sensor_topic", "/sensor/dvl")
        output_sensor_topic = rospy.get_param("~output_sensor_topic", "/sensor/dvl_filtered")
        cutoff = rospy.get_param("~cutoff", 10)
        fs = rospy.get_param("~fs", 33)
        order = rospy.get_param("order", 4)

        # Setup butterworth filter class for each linear velocity 
        bf_x = butterworth_filter(cutoff, fs, order)
        bf_y = butterworth_filter(cutoff, fs, order)
        bf_z = butterworth_filter(cutoff, fs, order)

        filtered_sensor_publisher = rospy.Publisher(output_sensor_topic, TwistStamped, queue_size=10)
        rospy.Subscriber("/clock", Clock, clock_callback)
        rospy.Subscriber(input_sensor_topic, TwistStamped, sensor_callback)

        rospy.spin()

    except rospy.ROSInterruptException():
        pass
