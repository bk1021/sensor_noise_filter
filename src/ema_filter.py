#!/usr/bin/python3
import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
from rosgraph_msgs.msg import Clock


class ema_filter():
    def __init__(self, alpha=0.5):
        # alpha can be approximate as 2/(window_size + 1)
        self.alpha = alpha
        self.last_value = 0
        self.first_data_set = False 

    def filter(self, input_value):
        '''
        Perform exponential moving average and return filtered value
        '''
        if not self.first_data_set:
            self.last_value = input_value
            self.first_data_set = True
            return input_value
        
        filtered_value = self.alpha*input_value + (1 - self.alpha)*self.last_value
        self.last_value = filtered_value
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
    
    filtered_msg.twist.linear.x = ema_filter_x.filter(msg.twist.linear.x)
    filtered_msg.twist.linear.y = ema_filter_y.filter(msg.twist.linear.y)
    filtered_msg.twist.linear.z = ema_filter_z.filter(msg.twist.linear.z)
    
    # Publish the filtered message
    filtered_sensor_publisher.publish(filtered_msg)


if __name__ == "__main__":
    try:
        # Setup ros node and topics
        rospy.init_node("ema_filter", anonymous=True)
        input_sensor_topic = rospy.get_param("~input_sensor_topic", "/sensor/dvl")
        output_sensor_topic = rospy.get_param("~output_sensor_topic", "/sensor/dvl_filtered")
        alpha = rospy.get_param("~alpha", 0.5)

        # Setup moving average filter class for each linear velocity 
        ema_filter_x = ema_filter(alpha)
        ema_filter_y = ema_filter(alpha)
        ema_filter_z = ema_filter(alpha)

        filtered_sensor_publisher = rospy.Publisher(output_sensor_topic, TwistStamped, queue_size=10)
        rospy.Subscriber("/clock", Clock, clock_callback)
        rospy.Subscriber(input_sensor_topic, TwistStamped, sensor_callback)

        rospy.spin()

    except rospy.ROSInterruptException():
        pass
