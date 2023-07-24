#! /usr/bin/env python3
"""
.. module:: load_environment.py
   :platform: Unix
   :synopsis: Script for the initialization of the Finite State Machine.  
.. moduleauthor:: rajatahaaa@live.com

This script is used to define the structure of the finite state machine. Here it is initialized with all its states and their respective transitions.
For each state it is defined how to behave for each transition received so that the machine cannot be stuck or have errors in the changes.
At the beginning of the execution it is also instantiated a helper entity that is passed to each state as parameter to make it easier in some cases to use functions and shared variables. This entity is an attribute of the respective Class :mod:`helper`.
However, the main role of the helper is the sharing of the mutex that is used to access the shared variables without having troubles doing it. The mutex used is of course just one to try to have a perfect syncronization among the state and the reading/writing processes.
 
"""

import random
import time
import re
import sys
import rospy
import roslib
from Expo_assignment_1 import architecture_name_mapper as anm
from expo_assignment_1.msg import Point
from armor_api.armor_client import ArmorClient
from os.path import dirname, realpath

client = ArmorClient("armor_client", "my_ontology")

path = dirname(realpath(__file__))
# path of .owl file
path = path + "/../topological_map/"

# Initializing with buffered manipulation and reasoning
client.utils.load_ref_from_file(path + "topology_plot.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", False, False)

client.utils.mount_on_ref()
client.utils.set_log_to_terminal(True)


def Ontology_Initialization():
    """
    Function used to store all the request to the aRMOR server, through the client, to modify the ontology.
    In particular, it uses a pre-built ontology that is stored in the project folder and modifies it by adding entities and properties.
    It adds entities, adds their properties, adds doors, and adds the timestamp.
    When it ends, it returns to the execute function and changes state.

    (There is also the possibility to save the ontology in another .owl file, and this can be done by uncommenting the last line of code of this script)

    Args:
        none

    Returns:
        none
    """

    # add properties
    client.manipulation.add_objectprop_to_ind('isIn', 'Robot1', 'E')
    client.manipulation.add_ind_to_class('E', 'LOCATION')

    client.manipulation.add_ind_to_class('C1', 'LOCATION')
    client.manipulation.add_ind_to_class('C2', 'LOCATION')
    client.manipulation.add_ind_to_class('R1', 'LOCATION')
    client.manipulation.add_ind_to_class('R2', 'LOCATION')
    client.manipulation.add_ind_to_class('R3', 'LOCATION')
    client.manipulation.add_ind_to_class('R4', 'LOCATION')

    client.manipulation.add_objectprop_to_ind('hasDoor', 'E', 'D5')
    client.manipulation.add_objectprop_to_ind('hasDoor', 'E', 'D6')

    client.manipulation.add_objectprop_to_ind('hasDoor', 'C1', 'D1')
    client.manipulation.add_objectprop_to_ind('hasDoor', 'C1', 'D2')
    client.manipulation.add_objectprop_to_ind('hasDoor', 'C1', 'D5')
    client.manipulation.add_objectprop_to_ind('hasDoor', 'C1', 'D7')

    client.manipulation.add_objectprop_to_ind('hasDoor', 'C2', 'D3')
    client.manipulation.add_objectprop_to_ind('hasDoor', 'C2', 'D4')
    client.manipulation.add_objectprop_to_ind('hasDoor', 'C2', 'D5')
    client.manipulation.add_objectprop_to_ind('hasDoor', 'C2', 'D6')

    client.manipulation.add_objectprop_to_ind('hasDoor', 'R1', 'D1')
    client.manipulation.add_objectprop_to_ind('hasDoor', 'R2', 'D2')
    client.manipulation.add_objectprop_to_ind('hasDoor', 'R3', 'D3')
    client.manipulation.add_objectprop_to_ind('hasDoor', 'R4', 'D4')

    # set each one different from the other
    client.call('DISJOINT', 'IND', '', ['E', 'C1', 'C2', 'R1', 'R2', 'R3', 'R4', 'D1', 'D2', 'D3', 'D4', 'D5', 'D6', 'D7'])

    # take the current time
    _actual_time = str(int(time.time()))

    # add the timestamp
    client.manipulation.add_dataprop_to_ind('visitedAt', 'R1', 'Long', _actual_time)
    client.manipulation.add_dataprop_to_ind('visitedAt', 'R2', 'Long', _actual_time)
    client.manipulation.add_dataprop_to_ind('visitedAt', 'R3', 'Long', _actual_time)
    client.manipulation.add_dataprop_to_ind('visitedAt', 'R4', 'Long', _actual_time)

    # Sync with reasoner
    client.utils.apply_buffered_changes()
    client.utils.sync_buffered_reasoner()


# Call the Ontology_Initialization function when this script is executed as a standalone script
if __name__ == "__main__":
    Ontology_Initialization()

		
		

