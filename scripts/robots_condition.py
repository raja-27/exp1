#!/usr/bin/env python3

"""
.. module::robots_condition.py
   :platform: ROS
   :synopsis:: Class for the Controller server to compute the path
.. moduleauthor:: rajatahaaa@live.com

Publisher:
    /state/battery_low to know the battery status

Servers:
    /state/set_pose: server to set the current robot pose
    /state/get_pose: server to get the current robot pose
"""

import threading
import rospy
from helper import InterfaceHelper
from Expo_assignment_1 import architecture_name_mapper as anm
from std_msgs.msg import Bool
from expo_assignment_1.msg import Point
from expo_assignment_1.srv import GetPose, GetPoseResponse, SetPose, SetPoseResponse
import random

# A tag for identifying logs producer
LOG_TAG = anm.NODE_ROBOTS_CONDITION

# Constant value representing the battery usage time


class RobotState:
    """
    Class implementing the services of the robot position and the publisher for the battery level.
    """

    def __init__(self):
        # Initialise this node
        rospy.init_node(anm.NODE_ROBOTS_CONDITION, log_level=rospy.INFO)
        # Initialise robot position
        self._pose = None
        # Initialise battery level
        self._battery_low = False
        self._randomness = rospy.get_param(anm.PARAM_RANDOM_ACTIVE, True)
        if self._randomness:
        	self._random_battery_time = rospy.get_param(anm.PARAM_BATTERY_TIME, [15.0, 60.0])
        rospy.Service(anm.SERVER_GET_POSE, GetPose, self.get_pose)
        rospy.Service(anm.SERVER_SET_POSE, SetPose, self.set_pose)
        # Start publisher on a separate thread
        th = threading.Thread(target=self.is_battery_low_)
        th.start()
        # Log information
        log_msg = (f'Initialise node `{anm.NODE_ROBOTS_CONDITION}` with services `{anm.SERVER_GET_POSE}` and '
                   f'`{anm.SERVER_SET_POSE}`, and topic {anm.TOPIC_BATTERY_LOW}.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    def set_pose(self, request):
        """
        The `robot/set_pose` service implementation.

        Args:
            request(Point): input parameter is the current robot pose to be set,

        Returns:
            SetPoseResponse(): This server returns an empty `response`
        """

        if request.pose is not None:
            # Store the new current robot position
            self._pose = request.pose
        else:
            rospy.logerr(anm.tag_log('Cannot set an unspecified robot position', LOG_TAG))
        return SetPoseResponse()

    def get_pose(self, request):
        """
        Function implementing the 'state/get_pose' service.
        
        Args:
            request: given by the client as empty, it is not used
            
        Returns:
            response(Point): current robot position
        """

        if self._pose is None:
            rospy.logerr(anm.tag_log('Cannot get an unspecified robot position', LOG_TAG))
        # Create the response with the robot pose and return it
        response = GetPoseResponse()
        response.pose = self._pose
        return response

    def is_battery_low_(self):
        """
        Fucntion that publishes the changes of the battery level.
        Publish battery level changes randomly
        Args:
            None
            
        Returns:
            None
        """

        publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
        
        if self._randomness:
        	self.battery_notifier_(publisher)

    def battery_notifier_(self, publisher):
        """
        Publish changes of battery levels. This method runs on a separate thread
    	  delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]`).
        
        Args:
            publisher(Publisher): publisher for the message
            
        Returns:
            None
        """
        delay=0
        while not rospy.is_shutdown():
            # Publish battery level: 'True' if the battery is low, 'False' otherwise
            publisher.publish(Bool(self._battery_low))
            # Simulate battery usage
            if self._battery_low:
            	print("Robot battery is low after",delay,"seconds")
            else:
            	print("Robot battery is full",delay,"seconds")
            if self._randomness:
            	delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
            rospy.sleep(delay)
            # Change battery state
            self._battery_low = not self._battery_low

if __name__ == "__main__":
    # Instantiate the node manager class
    RobotState()

    # Define the helper
    helper = InterfaceHelper()

    # Initialize robot position
    robot_pose_param = rospy.get_param(anm.PARAM_INITIAL_POSE, [0, 0])
    helper.init_robot_pose(Point(x=robot_pose_param[0], y=robot_pose_param[1]))

    rospy.spin()
