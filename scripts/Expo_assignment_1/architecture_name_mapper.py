#!/usr/bin/env python3

"""
.. module::architecture_name_mapper.py
   :platform: ROS
   :synopsis:: Class for the Name Mapper module
.. moduleauthor:: rajatahaaa@live.com

This script defines variable names used in the package to avoid spelling problems. It includes:

- State names for the Finite State Machine and Sub Finite State Machine.
- Transition names and other parameters used to modify shared variables for correct program flow.
- Action, planner, and controller names used in the program.
- Parameters for robot behavior, such as battery threshold, busy parameter, number of points for the planner, and recharging room name.
- Names of all locations in the ontology.
- X and Y coordinates of each location (match one-to-one with names above).
"""


import rospy

# The name of the parameter to define the environment size.
PARAM_ENVIRONMENT_SIZE = 'config/environment_size'

# The name of parameter to set the initial robot position.
PARAM_INITIAL_POSE = 'state/initial_pose'

# The name of the node that sets/gets the pose of the robot and manages its battery.
NODE_ROBOTS_CONDITION = 'robots_condition'

# The name of the server to get the current robot pose.
SERVER_GET_POSE = 'state/get_pose'

# The name of the server to set the current robot pose.
SERVER_SET_POSE = 'state/set_pose'

# The name of the topic where the battery state is published.
TOPIC_BATTERY_LOW = 'state/battery_low'

# Parameter indicating the sleep time [s]
SLEEP_TIME = 0.3

# Parameter indicating the battery time [s]
PARAM_BATTERY_TIME = 'test/random_sense/battery_time'

# The name of the planner node.
NODE_PLANNER = 'planner'

# The name of the action server solving the motion planning problem.
ACTION_PLANNER = 'motion/planner'

# The number of points in the plan. It should be a list [min_n, max_n],
# where the number of points is a random value in the interval [min_n, max_n).
PARAM_PLANNER_POINTS = 'test/random_plan_points'

# The delay between the computation of the next via points.
# It should be a list `[min_time, max_time]`, and the computation will 
# last for a random number of seconds in such an interval.
PARAM_PLANNER_TIME = 'test/random_plan_time'

# The name of the controller node.
NODE_CONTROLLER = 'controller'

# The name of the action server solving the motion control problem.
ACTION_CONTROLLER = 'motion/controller'

# The time required to reach a via point.
# It should be a list `[min_time, max_time]`, and the time to reach a
# via point will be a random number of seconds in such an interval.
PARAM_CONTROLLER_TIME = 'test/random_motion_time'


def tag_log(msg, producer_tag):
    """
    Function used to label each log with a producer tag.

    Args:
        msg (str): message that will be visualized
        producer_tag (str): tag identifying the log producer
            
    Returns:
        log_msg (str): message for the log
    """
    return f'@{producer_tag}>> {msg}'

