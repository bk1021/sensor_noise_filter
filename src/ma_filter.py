#!/usr/bin/python3
import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
from rosgraph_msgs.msg import Clock


class ma_filter():
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.window = np.zeros(window_size)
        self.window_sum = 0
        self.input_count = 0

    def filter(self, input_value):
        '''
        Perform moving average and return filtered value
        '''
        # Update input count
        self.input_count += 1

        # Calculate window sum
        self.window_sum += input_value - self.window[0]

        # Reload window values
        self.window = np.roll(self.window, -1)  # Shift window values to the left
        self.window[-1] = input_value  # Set the last value to the new input_value

        # Get the average
        filtered_value = self.window_sum / min(self.input_count, self.window_size)
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
    
    filtered_msg.twist.linear.x = ma_filter_x.filter(msg.twist.linear.x)
    filtered_msg.twist.linear.y = ma_filter_y.filter(msg.twist.linear.y)
    filtered_msg.twist.linear.z = ma_filter_z.filter(msg.twist.linear.z)
    
    # Publish the filtered message
    filtered_sensor_publisher.publish(filtered_msg)


if __name__ == "__main__":
    try:
        # Setup ros node and topics
        rospy.init_node("ma_filter", anonymous=True)
        input_sensor_topic = rospy.get_param("~input_sensor_topic", "/sensor/dvl")
        output_sensor_topic = rospy.get_param("~output_sensor_topic", "/sensor/dvl_filtered")
        window_size = rospy.get_param("~window_size", 5)

        # Setup moving average filter class for each linear velocity 
        ma_filter_x = ma_filter(window_size)
        ma_filter_y = ma_filter(window_size)
        ma_filter_z = ma_filter(window_size)

        filtered_sensor_publisher = rospy.Publisher(output_sensor_topic, TwistStamped, queue_size=10)
        rospy.Subscriber("/clock", Clock, clock_callback)
        rospy.Subscriber(input_sensor_topic, TwistStamped, sensor_callback)

        rospy.spin()

    except rospy.ROSInterruptException():
        pass
