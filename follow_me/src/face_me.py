#!/usr/bin/env python
import roslib
roslib.load_manifest('follow_me')
import rospy

from geometry_msgs.msg import Twist, PointStamped
from people_msgs.msg import PositionMeasurement

import numpy as np
import math
import tf
import threading
import copy

cmd_vel_topic = rospy.get_param("/cmd_vel_topic", "/cmd_vel")
base_link_frame = rospy.get_param("/base_link_frame", "/base_link")

class FaceMe(object):
    def __init__(self):
        # Set up publishers and subscribers
        self._command_pub = rospy.Publisher(cmd_vel_topic, Twist)
        self._tracker_sub = rospy.Subscriber("/people_tracker_filter", PositionMeasurement, (lambda x: self.tracker_callback(x)))

        # Variable to hold the current goal to look at
        self.goal_lock = threading.Lock()
        self.current_goal = PointStamped()
        self.got_first_goal = False

        # A few parameters
        self.turn_threshold = math.pi/20  # Threshold to stop pursuing the target.
        self.angular_speed = 0.6

        # Need a transform listener to transform points from kinect_ns to base_link frame
        self.tfl = tf.TransformListener()

        # Wait until first goal appears
        while not self.got_first_goal:
            rospy.sleep(0.1)

        while not rospy.is_shutdown():
            # If a new goal has not been found and the last output command is 0, don't do anything.
            # Create a new transformed point in the base_link frame, rather than the kinect frame
            transformed_goal = PointStamped()
            with self.goal_lock:
                goal = copy.copy(self.current_goal)

            try:
                transformed_goal = (self.tfl).transformPoint(base_link_frame, goal)

            except tf.LookupException:
                rospy.loginfo("Exception thrown transforming from kinect frame to base_link frame.")

            # Math to determine which way to turn, if at all.
            angle_to_turn = math.atan2(transformed_goal.point.y, transformed_goal.point.x)

            # Set the velocity command to output
            command = Twist()
            if math.fabs(angle_to_turn) < self.turn_threshold:
                pass # Keep all coordinates as 0, so the robot stops moving

            elif angle_to_turn > 0.0:
                command.angular.z = self.angular_speed
            else:
                command.angular.z = -1*self.angular_speed

            self._command_pub.publish(command)

            rospy.sleep(0.05)            

    def tracker_callback(self, data):
        # Update the current_goal variable
        goal=PointStamped()
        goal.header.frame_id = data.header.frame_id    
        goal.header.stamp = data.header.stamp
        goal.point = copy.copy(data.pos)

        with self.goal_lock:
            self.current_goal = copy.copy(goal)

        # Tell the main loop that a person was finally tracked.
        self.got_first_goal = True

if __name__ == "__main__":
    rospy.init_node("face_me")
    main = FaceMe()
    rospy.spin()

