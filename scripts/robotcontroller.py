#! /usr/bin/env python
"""
.. module::robotcontroller.py
   :platform: ROS
   :synopsis:: Class for the Controller server to compute the path
.. moduleauthor:: rajatahaaa@live.com

This class is the server used by the FSM to simulate the movement of the robot from a starting position to a target one.	
The path to follow is passed by the client. The controller starts as soon as the planner ends computing the path.
The server simulates the movement of the robot and then publishes the result. In case the process is interrupted due to some signals (a battery low for example), then it returns nothing because of the preemption.

Servers:
	:attr:`motion/controller`: server used to simulate the movement of the robot.
"""


import random
import rospy
from Expo_assignment_1 import architecture_name_mapper as anm
from actionlib import SimpleActionServer
from expo_assignment_1.msg import ControlFeedback, ControlResult
from expo_assignment_1.srv import SetPose
import expo_assignment_1

LOG_TAG = anm.NODE_CONTROLLER

class ControllingAction(object):

    def __init__(self):
        self._random_motion_time = rospy.get_param(anm.PARAM_CONTROLLER_TIME, [0.1, 2.0])
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER,
                                      expo_assignment_1.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()
        log_msg = (f'`{anm.ACTION_CONTROLLER}` Action Server initialized. It will navigate through the plan with a delay '
                   f'between each via point spanning in [{self._random_motion_time[0]}, {self._random_motion_time[1]}).')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    def execute_callback(self, goal):
        if goal is None or goal.via_points is None or len(goal.via_points) == 0:
            rospy.logerr(anm.tag_log('No via points provided! This service will be aborted!', LOG_TAG))
            self._as.set_aborted()
            return

        feedback = ControlFeedback()
        rospy.loginfo(anm.tag_log('Server is controlling...', LOG_TAG))
        for point in goal.via_points:
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Service has been canceled by the client!', LOG_TAG))
                self._as.set_preempted()
                return
            delay = random.uniform(self._random_motion_time[0], self._random_motion_time[1])
            rospy.sleep(delay)
            feedback.reached_point = point
            self._as.publish_feedback(feedback)
            _set_pose_client(point)
            log_msg = f'Reaching point ({point.x}, {point.y}).'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        result = ControlResult()
        result.reached_point = feedback.reached_point
        rospy.loginfo(anm.tag_log('Motion control successful.', LOG_TAG))
        self._as.set_succeeded(result)
        return  # Succeeded.

def _set_pose_client(pose):
    rospy.wait_for_service(anm.SERVER_SET_POSE)
    try:
        log_msg = f'Set current robot position to the `{anm.SERVER_SET_POSE}` node.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
        service(pose)  # The `response` is not used.
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set the current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

if __name__ == '__main__':
    rospy.init_node(anm.NODE_CONTROLLER, log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()

