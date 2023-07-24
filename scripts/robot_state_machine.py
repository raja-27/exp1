#!/usr/bin/env python3
"""
.. module:: robot_state_machine.py
   :platform: Unix
   :synopsis: Script for the initialization of the rOBOT State Machine.  
.. moduleauthor:: rajatahaaa@live.com

This script is used to define the structure of the finite state machine. Here it is initialized with all its states and their respective transitions.
For each state it is defined how to behave for each transition received so that the machine cannot be stuck or have errors in the changes.
At the beginning of the execution it is also instantiated a helper entity that is passed to each state as parameter to make it easier in some cases to use functions and shared variables. This entity is an attribute of the respective Class :mod:`helper`.
However, the main role of the helper is the sharing of the mutex that is used to access the shared variables without having troubles doing it. The mutex used is of course just one to try to have a perfect syncronization among the state and the reading/writing processes.
 
"""



import smach
import rospy
import random
import smach_ros
import time
import re
import sys
from smach import StateMachine,State
from helper import InterfaceHelper
from load_environment import Ontology_Initialization
from expo_assignment_1.msg import Point, ControlGoal, PlanGoal
from armor_api.armor_client import ArmorClient
from Expo_assignment_1 import architecture_name_mapper as anm
client = ArmorClient("armor_client", "my_ontology")



# The list of names that identify the states of the Finite State Machine.
STATE_INITIALIZE = 'Load_Envornment'
STATE_CHOOSE_CORRIDOR = 'ADOPT_CORRIDOR'
STATE_CHOOSE_URGENT = 'ADOPT_URGENTROOM'
STATE_MOVING_CORRIDOR = "TRAVELING_TO_CORRIDOR"
STATE_MOVING_URGENT = "TRAVELING_TO_URGENT"
STATE_CORRIDOR = 'Normal mode (in_corridor)'
STATE_RECHARGING = 'Recharging(In_E_corridor)'
START_URGENT='Visit rooms (room_emergency)'
START_CORRIDOR='CORRIDOR'
STATE_ROOM='ROOM'  

# The list of names that identify the transitions of the Finite State Machine.
TRANS_INITIALIZED = 'map_loaded'
TRANS_DECIDED_CORRIDOR = 'corridor_chosen'
TRANS_MOVED_CORRIDOR = 'robot_moved_corridor'
TRANS_MOVED_URGENT= 'robot_moved_urgent'
TRANS_DECIDED_URGENT = 'urgentroom_chosen'
TRANS_BATTERY_LOW = 'battery_low'
TRANS_RECHARGING = 'recharging'
TRANS_RECHARGED = 'recharged'


# Sleeping time (in seconds) of the waiting thread to allow the computations
# for getting stimulus from the other components of the architecture.
LOOP_SLEEP_TIME = 0.5


class OntologyHelper:
    """
    A class to used to query an use the armor client commands
    """

    def __init__(self):
        
        self.robot = "Robot1"
        self.charging_room = "E"
        self.urgent = 'urgent'
        self.corridor = 'corridor'
        self.robot_position = 'robot_position'
        self.reachable_locations = 'reachable_locations'

 
        
    def list_formatter(self, oldlocation, start,end):
        """
        This formatter strores the return value when the armor is queried because the return value be of many unwanted symbols

        Args:
            newlocation(str): list which will hold he location and everytime the armor is queried it stores the location alone
        Returns:
            new newlocation[] everytime the system is quried

        """
        newlocation = []
        for location in oldlocation:
        	newlocation.append(re.search(start + '(.+?)' + end,location).group(1))
        return newlocation
            
    
    


	
    def location_acquire(self, location):
        """
	the locations are queried to the armor 
        Args:
            location(str): a string to know whether the location is subjected to urgentrooms or corridors.

        Returns:
            location_list(str): list of queried locations.

        """

        if location == self.corridor:
            corridors_list = self.list_formatter(client.query.ind_b2_class('CORRIDOR'), '#', '>')
            location_list =  corridors_list
            
        if location == self.urgent:
            urgent_rooms = self.list_formatter(client.query.ind_b2_class('URGENT'), '#', '>')
            location_list = urgent_rooms

        if location == self.robot_position:
            current_position = self.list_formatter(client.query.objectprop_b2_ind('isIn',self.robot), '#', '>')[0]
            location = current_position
            location_list = current_position
            
        if location == self.reachable_locations:
            reachable_destinations =  self.list_formatter(client.query.objectprop_b2_ind('canReach',self.robot), '#', '>')
            location_list = reachable_destinations

        return location_list


    def choose_target(self):
        """
        The function that helps to predict the robot;s movement

        Returns:
            current_position(str): The current rposition of robot

        Returns:
            target(str): The target position chosen from a list of reachable and urgent locations
        
        Returns:
            corridors(str): all the corridors

        """

        corridors_list = self.location_acquire(self.corridor)
        current_position = self.location_acquire(self.robot_position)
        print("\ncurrent position of robot is: ["+(current_position)+"]")
        if current_position=='E':
        	print("\ncoordinate is: (0,0)")
        if current_position=='C1':
        	print("\ncoordinate is: (1,1)")
        if current_position=='C2':
        	print("\ncoordinate is:(-1,1)")
        if current_position=='R1':
        	print("\ncoordinate is:(2,1)")
        if current_position=='R2':
        	print("\ncoordinate is:(2,2)")
        if current_position=='R3':
        	print("\ncoordinate is:(-2,1)")
        if current_position=='R4':
        	print("\ncoordinate is:(-2,2)")
        possible_destinations = self.location_acquire(self.reachable_locations)
        print("Reachable Positions: [" + ", ".join(possible_destinations) +"]")
        urgent_rooms = self.location_acquire(self.urgent)
        print("URGENCY ROOMS: [" + ", " .join(urgent_rooms) + "]")
        ReachableUrgent_room=[i for i in possible_destinations if i in urgent_rooms]
        if not ReachableUrgent_room:
            # if the list of possible destination 
            # contains only one element
            ReachableCorridor_room = [i for i in possible_destinations if i in corridors_list]
            if not ReachableCorridor_room:
            	target = random.choice(possible_destinations)
            else:
                target = random.choice(ReachableCorridor_room)
        # if chosen_target list is not empty
        else:
            # save the first element of the list as the oldest timestamp
            oldest = client.query.dataprop_b2_ind('visitedAt', ReachableUrgent_room[0])
            # clean the string
            oldest =  str(self.list_formatter(oldest, '"', '"')[0])
            for i in range (len(ReachableUrgent_room)):
                choice_last_visit = client.query.dataprop_b2_ind('visitedAt', ReachableUrgent_room[i])
                choice_last_visit =  str(self.list_formatter(choice_last_visit, '"', '"')[0])
                if choice_last_visit <= oldest:
                    target = ReachableUrgent_room[i]
        print("Target to be Reached " + target)
        return current_position, target


    def move_location(self, target, current_position):
        """
        Function used to make the robot move to the acquired target

        Args:
            target(str): The target position chosen from a reachable and urgent locations
            current_pos(str): The current robot position obtained from the ontology
            corridors(str): All corridors  in the map

        """
        

       
        client.manipulation.replace_objectprop_b2_ind('isIn', self.robot, target, current_position)
        print("Robot in " + target + " and monitoring")
        last_change = client.query.dataprop_b2_ind('now', self.robot)
        last_change =  str(self.list_formatter(last_change, '"', '"')[0])
        current_time = str(int(time.time()))
        client.manipulation.replace_dataprop_b2_ind('now', self.robot, 'Long', current_time, last_change)
        corridors=self.list_formatter(client.query.ind_b2_class('CORRIDOR'), '#', '>')
        if target not in corridors:
            last_visit = client.query.dataprop_b2_ind('visitedAt', target)
            last_visit =  str(self.list_formatter(last_visit, '"', '"')[0])
            client.manipulation.replace_dataprop_b2_ind('visitedAt',target,'Long',current_time,last_visit)
        client.utils.apply_buffered_changes()
        client.utils.sync_buffered_reasoner()

    def go_to_recharge(self, robot_location):
        """
        Function to move the robot toits charging room location

        Args:
            robot_location(str): The current robot position obtained f

        """
        print("\nRobot's Battery is low and going to recharge room for recharge ")
        client.manipulation.replace_objectprop_b2_ind('isIn', self.robot, self.charging_room, robot_location)
        client.utils.sync_buffered_reasoner()

class LoadEnvironment(smach.State):
    """
    A class to load the ontology map to the rbot
    """
    def __init__(self):

        
        State.__init__(self, outcomes = [TRANS_INITIALIZED])

    def execute(self, userdata):
        """
        Function responsible of the loading of the
        environment. It calls the Ontology_Initialization() which helps for the robot to know the environment

        Args:
            userdata: not used

        Returns:
            TRANS_INITIALIZED(str): transits to STATE_NORMAL which is a nested statemachine

        """
        Ontology_Initialization()
        print("ONTOLOGY MAP LOADED")
        return TRANS_INITIALIZED
    

class ChooseCorridor(smach.State):
    """
    A class for the root to decide which corridor it should reach
    """

    def __init__(self, interface_helper, ontology_helper):
        
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._ontology = ontology_helper
        # Get the environment size from ROS parameters.
        self.environment_size = rospy.get_param('config/environment_size')
        State.__init__(self, outcomes = [TRANS_RECHARGING, TRANS_DECIDED_CORRIDOR,TRANS_DECIDED_URGENT], output_keys = ['robot_position', 'target','random_plan'])

    def execute(self, userdata):
        """
        Function responsible of the transitions between the 
        STATE_CHOOSE_CORRIDOR and the STATE_RECHARGING or STATE_MOVING_CORRIDOR or STATE_CHOOSE_URGENT.
        In this function theere are three possible transitions where if the battery is low,the state will be STATE_RECHARGING,if the target is
        a corridor the state will be STATE_MOVING_CORRIDOR and if the target is a urgent room he state will be STATE_CHOOSE_URGENT.A planner is used and it plans a path to be followed

        Args:
            userdata: used for output_keys to pass data to the other states.

        Returns:
            TRANS_RECHARGING (str): transition to STATE_RECHARGING.
        
        Returns:
            TRANS_DECIDED_URGENT (str): transition to the STATE_DECISION_URGENT.
            
        .Returns:
            TRANS_DECIDED_CORRIDOR (str): transition to the STATE_DECISION_CORRIDOR.

        """
        # Define a random point to be reached through some via-points to be planned.
        goal = PlanGoal()
        goal.target = Point(x = random.uniform(0, self.environment_size[0]),
                            y = random.uniform(0, self.environment_size[1]))
        robot_position, target = self._ontology.choose_target()
        userdata.robot_position = robot_position
        userdata.target = target
        # Invoke the planner action server.
        self._helper.planner_client.send_goal(goal)
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.planner_client.cancel_goals()
                    self._ontology.go_to_recharge(robot_position)
                    return TRANS_RECHARGING
                # If the target is a Rooom then planner cancels goals and transition TRANS_DECIDED_URGENT occurs
                if target=='R1' or target=='R2' or target=='R3' or target=='R4':
                        self._helper.planner_client.cancel_goals()
                        return TRANS_DECIDED_URGENT	
                 # If the planner finishes its computation, then take the TRANS_DECIDED_CORRIDOR transition
                if self._helper.planner_client.is_done():
                    userdata.random_plan = self._helper.planner_client.get_results().via_points
                    return TRANS_DECIDED_CORRIDOR

            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)


class MoveToTargetCorridor(smach.State):
    """
    A class to move he robot to a cooridor which has been acquired
    """

    def __init__(self, interface_helper, ontology_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._ontology = ontology_helper

        State.__init__(self, outcomes = [TRANS_RECHARGING, TRANS_MOVED_CORRIDOR], input_keys = [ "random_plan",'robot_position', 'target'], output_keys = ['robot_position'])

    def execute(self, userdata):
        """
        Function responsible of the transition between the 
        STATE_MOVING_CORRIDOR and the STATE_RECHARGING or STATE_CHOOSE_CORRIDOR.
        this class is responsible for the robot to move to decided corridor and stays for specific time.A controller is used to enable the movement of the robot

        Args:
            userdata: for input_keys and output_keys to get data and pass data.
        
        Returns:
            TRANS_RECHARGING(str): transition to the STATE_RECHARGING.

        Returns:
            TRANS_MOVED_CORRIDOR(str): transition to STATE_DECISION_CORRIDOR.

        """
        plan = userdata.random_plan
        # Start the action server for moving the robot through the planned via-points.
        goal = ControlGoal(via_points = plan)
        self._helper.controller_client.send_goal(goal)
        robot_position = userdata.robot_position
        target = userdata.target

        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.controller_client.cancel_goals()
                    self._ontology.go_to_recharge(robot_position)
                    return TRANS_RECHARGING
                # If the controller finishes its computation, then take the `went_random_pose` transition, which is related to the `repeat` transition.
                if self._helper.controller_client.is_done():
                    self._ontology.move_location(target, robot_position)
                    #the robot stays in a corridr for specific time period
                    rospy.sleep(anm.MONITOR_TIME)
                    return TRANS_MOVED_CORRIDOR
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)



class ChooseUrgent(smach.State):
    """
    A class for the root to decide which Urgent room it should reach
    """


    def __init__(self, interface_helper, ontology_helper):
        
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._ontology = ontology_helper
        # Get the environment size from ROS parameters.
        self.environment_size = rospy.get_param('config/environment_size')
        State.__init__(self, outcomes = [TRANS_RECHARGING, TRANS_DECIDED_URGENT,TRANS_DECIDED_CORRIDOR], output_keys = ['robot_position', 'target','random_plan'])

    def execute(self, userdata):
        """
        Function responsible of the transitions between the 
        STATE_CHOOSE_URGENT and the STATE_RECHARGING or STATE_MOVING_URGENT or STATE_CHOOSE_CORRIDOR.
        In this function theere are three possible transitions where if the battery is low,the state will be STATE_RECHARGING,if the target is
        a corridor the state will be STATE_DECISON_CORRIDOR and if the target is a urgent room he state will be STATE_MOVING_URGENT.A planner is used and it plans a path to be followed

        Args:
            userdata: used for output_keys to pass data to the other states.

        Returns:
            TRANS_RECHARGING (str): transition to STATE_RECHARGING.
        
        Returns:
            TRANS_DECIDED_URGENT (str): transition to the STATE_DECISION_URGENT.
            
        .Returns:
            TRANS_DECIDED_CORRIDOR (str): transition to the STATE_DECISION_CORRIDOR.

        """
        # Define a random point to be reached through some via-points to be planned.
        goal = PlanGoal()
        goal.target = Point(x = random.uniform(0, self.environment_size[0]),
                            y = random.uniform(0, self.environment_size[1]))
        robot_position, target = self._ontology.choose_target()
        userdata.robot_position = robot_position
        userdata.target = target
        # Invoke the planner action server.
        self._helper.planner_client.send_goal(goal)
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.planner_client.cancel_goals()
                    self._ontology.go_to_recharge(robot_position)
                    return TRANS_RECHARGING
                #if the target is a corridor then planner cancels goal and transition TRANS_DECIDED_CORRIDOR occurs
                if  target=='C1' or target=='C2' or target=='E':
                    self._helper.planner_client.cancel_goals()
                    return TRANS_DECIDED_CORRIDOR
                # If the controller finishes its computation, then take the `went_random_pose` transition, which is related to the `repeat` transition.
                if self._helper.planner_client.is_done():
                    userdata.random_plan = self._helper.planner_client.get_results().via_points
                    return TRANS_DECIDED_URGENT


            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)


class MoveToTargetUrgent(smach.State):
    """
    A class to move the robot to a Urgent room which has been acquired
    """

    def __init__(self, interface_helper, ontology_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._ontology = ontology_helper

        State.__init__(self, outcomes = [TRANS_RECHARGING, TRANS_MOVED_URGENT], input_keys = [ "random_plan",'robot_position', 'target'], output_keys = ['crobot_position'])

    def execute(self, userdata):
        """
        Function responsible of the transition between the 
        STATE_MOVING_URGENT and the STATE_RECHARGING or STATE_CHOOSE_URGENT.
        this class is responsible for the robot to move to decided corridor and stays for specific time.A controller is used to enable the movement of the robot

        Args:
            userdata: for input_keys and output_keys to get data and pass data.
        
        Returns:
            TRANS_RECHARGING(str): transition to the STATE_RECHARGING.

        Returns:
            TRANS_MOVED_URGENT(str): transition to STATE_DECISION_URGENT.

        """
        # Get the plan to a random position computed by the `PLAN_TO_RANDOM_POSE` state.
        plan = userdata.random_plan
        # Start the action server for moving the robot through the planned via-points.
        goal = ControlGoal(via_points = plan)
        """
        ControlGoal: via points to reach the goal 
        """
        self._helper.controller_client.send_goal(goal)
        robot_position = userdata.robot_position
        target = userdata.target

        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.controller_client.cancel_goals()
                    self._ontology.go_to_recharge(robot_position)
                    return TRANS_RECHARGING
                # If the controller finishes its computation, then take the `went_random_pose` transition, which is related to the `repeat` transition.
                if self._helper.controller_client.is_done():
                    self._ontology.move_location(target, robot_position)
                    #the robot stays in  a urgent room for some time
                    rospy.sleep(anm.MONITOR_TIME)
                    return TRANS_MOVED_URGENT
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)




class Recharging(State):
    """
    A class for executing Recharging for the robot
    """

    def __init__(self, interface_helper, ontology_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._ontology = ontology_helper
        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        State.__init__(self, outcomes = [TRANS_RECHARGED])

    # Define the function performed each time a transition is such to enter in this state.
    # Note that the input parameter `userdata` is not used since no data is required from the other states.
    def execute(self, userdata):
        """
        Function responsible of the transition between the 
        STATE_RECHARGING to the STATE_NORMAL.
	the robot will be in this state until the battery is enough
 
        Args:
            userdata: not used

        Returns:
            TRANS_RECHARGED(str): transition to STATE_NORMAL

        """
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is no low anymore take the `charged` transition.
                if not self._helper.is_battery_low():
                    self._helper.reset_states()  # Reset the state variable related to the stimulus.
                    return TRANS_RECHARGED
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)

def main():
    """
    This function creates the state machine and defines all the transitions. Here two nested state machine are created.
    """
    rospy.init_node('Robot_state_machine', log_level = rospy.INFO)
    # Initialise an classes to manage the interfaces with the other nodes in the architecture.
    helper = InterfaceHelper()
    ontology = OntologyHelper()
    sm_main = StateMachine([])
    with sm_main:

        StateMachine.add(STATE_INITIALIZE, LoadEnvironment(),
                         transitions = {TRANS_INITIALIZED: STATE_CORRIDOR})
        
        sm_corridor = StateMachine(outcomes=[TRANS_BATTERY_LOW,START_URGENT])

        with sm_corridor:
            StateMachine.add(STATE_CHOOSE_CORRIDOR, ChooseCorridor(helper, ontology),
                            transitions = {TRANS_RECHARGING : TRANS_BATTERY_LOW,
                                            TRANS_DECIDED_CORRIDOR: STATE_MOVING_CORRIDOR,TRANS_DECIDED_URGENT:START_URGENT})
            StateMachine.add(STATE_MOVING_CORRIDOR, MoveToTargetCorridor(helper, ontology),
                            transitions = {TRANS_RECHARGING : TRANS_BATTERY_LOW,
                                            TRANS_MOVED_CORRIDOR: STATE_CHOOSE_CORRIDOR})
                                            
        sm_room = StateMachine(outcomes=[TRANS_BATTERY_LOW, START_CORRIDOR])
	
        with sm_room:
            StateMachine.add(STATE_CHOOSE_URGENT, ChooseUrgent(helper, ontology),
                            transitions = {TRANS_RECHARGING : TRANS_BATTERY_LOW,
                                            TRANS_DECIDED_URGENT: STATE_MOVING_URGENT,TRANS_DECIDED_CORRIDOR: START_CORRIDOR})
            StateMachine.add(STATE_MOVING_URGENT, MoveToTargetUrgent(helper, ontology),
                            transitions = {TRANS_RECHARGING : TRANS_BATTERY_LOW,
                                            TRANS_MOVED_URGENT: STATE_CHOOSE_URGENT})
           
        StateMachine.add(STATE_CORRIDOR, sm_corridor,
                         transitions={TRANS_BATTERY_LOW: STATE_RECHARGING,START_URGENT:STATE_ROOM})
        StateMachine.add(STATE_ROOM, sm_room,
                         transitions={TRANS_BATTERY_LOW: STATE_RECHARGING,START_CORRIDOR:STATE_CORRIDOR})
            
        StateMachine.add(STATE_RECHARGING, Recharging(helper, ontology),
                            transitions = {TRANS_RECHARGED: STATE_CORRIDOR})
    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('sm_introspection', sm_main, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm_main.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
   
if __name__ == "__main__":
    main()
