#!/usr/bin/env python
import roslib
roslib.load_manifest('follow_me')
import rospy

from geometry_msgs.msg import Twist

cmd_vel_topic = rospy.get_param("/cmd_vel_topic", "/cmd_vel")

class Drive(object):
    def __init__(self):
        # Set up publishers and subscribers
        self._command_pub = rospy.Publisher(cmd_vel_topic, Twist)

        # A few parameters
        self.linear_speed = 0.075
        self.angular_speed = -0.225

        # Sleep for a bit to let everything get set up.
        rospy.sleep(10.0)

        while not rospy.is_shutdown():
            # Set the velocity command to output
            command = Twist()
            command.linear.x = self.linear_speed
            command.angular.z = self.angular_speed
            self._command_pub.publish(command)

            rospy.sleep(0.15)            

if __name__ == "__main__":
    rospy.init_node("test_drive")
    main = Drive()
    rospy.spin()

