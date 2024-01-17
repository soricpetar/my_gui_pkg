#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy

def publish_joy_click():
    # Initialize the ROS Node
    rospy.init_node('joy_click_publisher', anonymous=True)

    # Create a ROS publisher on the /kalipen/joy topic with message type Joy
    joy_pub = rospy.Publisher('/kalipen/joy', Joy, queue_size=1)

    # Rate at which to publish messages (10 Hz in this case)
    rate = rospy.Rate(0.1)

    # Create a new Joy message
    joy_msg = Joy()
    joy_msg.header.stamp = rospy.Time.now()
    joy_msg.buttons = [1]  # Assuming the click button is the first button

    # Keep publishing while ROS is running
    while not rospy.is_shutdown():
        # Publish the message
        print("publishing joy")
        joy_pub.publish(joy_msg)

        # Sleep for the remainder of the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joy_click()
    except rospy.ROSInterruptException:
        pass
