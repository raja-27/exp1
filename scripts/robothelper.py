#!/usr/bin/env python3
"""
.. module:: robothelper.py
   :platform: ROS
   :synopsis: Class for help functions
   
.. moduleauthor::rajatahaaa@live.com
 
This class implements an helper member that can be used in the program it is included into to simplify the code.
In particular this helper provides all the action clients used and needed to control the robot plus other functions used to retrieve information from the data and queries acquired.
It is a way to avoid the use of many global variables that could lead to some problems in the code and it also allows an easier re-use of the code.
 
Subscribers:
    :state/battery_low- where the state of the battery (high/low) is published
Servers:
    :state/set_pose- server to set the current robot pose in robots-condition node
  
"""


import rospy
import random
import time

from actionlib import SimpleActionClient
from threading import Lock
from armor_api.armor_client import ArmorClient
from std_msgs.msg import Bool
from expo_assignment_1.msg import PlanAction, ControlAction
from expo_assignment_1.srv import SetPose

client = ArmorClient("armor_client", "my_ontology")


class ActionClientHelper:
    def __init__(self, service_name, action_type, done_callback=None, feedback_callback=None, mutex=None):
        self._is_running = False
        self._is_done = False
        self._results = None
        self._service_name = service_name
        self._mutex = Lock() if mutex is None else mutex
        self._client = SimpleActionClient(service_name, action_type)
        self._external_done_cb = done_callback
        self._external_feedback_cb = feedback_callback
        self._client.wait_for_server()

    def send_goal(self, goal):
        if not self._is_running:
            self._client.send_goal(goal, done_cb=self.done_callback_, feedback_cb=self.feedback_callback_)
            self._is_running = True
            self._is_done = False
            self._results = None
        else:
            print("Warning send a new goal, cancel the current request first!")

    def cancel_goals(self):
        if self._is_running:
            self._client.cancel_all_goals()
            self.reset_client_states()
        else:
            print("Warning cannot cancel a not running service!")

    def reset_client_states(self):
        self._is_running = False
        self._is_done = False
        self._results = None

    def feedback_callback_(self, feedback):
        self._mutex.acquire()
        try:
            if self._external_feedback_cb is not None:
                self._external_feedback_cb(feedback)
        finally:
            self._mutex.release()

    def done_callback_(self, status, results):
        self._mutex.acquire()
        try:
            self._is_running = False
            self._is_done = True
            self._results = results
            if self._external_done_cb is not None:
                self._external_done_cb(status, results)
        finally:
            self._mutex.release()

    def is_done(self):
        return self._is_done

    def is_running(self):
        return self._is_running

    def get_results(self):
        if self._is_done:
            return self._results
        else:
            print("Error: cannot get result")
            return None


class InterfaceHelper:
    def __init__(self):
        self.mutex = Lock()
        self.reset_states()
        rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self.battery_callback_)
        self.planner_client = ActionClientHelper(anm.ACTION_PLANNER, PlanAction, mutex=self.mutex)
        self.controller_client = ActionClientHelper(anm.ACTION_CONTROLLER, ControlAction, mutex=self.mutex)

    def reset_states(self):
        self._battery_low = False

    def battery_callback_(self, msg):
        self.mutex.acquire()
        try:
            self._battery_low = msg.data
        finally:
            self.mutex.release()

    def is_battery_low(self):
        return self._battery_low

    @staticmethod
    def init_robot_pose(point):
        rospy.wait_for_service(anm.SERVER_SET_POSE)
        try:
            service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
            service(point)
            print("Setting initial robot position")
        except rospy.ServiceException as e:
            print("Cannot set current robot position")


