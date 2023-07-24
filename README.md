

---

# Experimental Robotics Laboratory Assignment-1

## Table of Contents
1. [Introduction](#introduction)
2. [Software Architecture](#software-architecture)
    - [Robot Behavior](#robot-behavior)
    - [Final Nodes Diagram](#final-nodes-diagram)
    - [Finite State Machine Diagram](#finite-state-machine-diagram)
3. [Launching the Software](#launching-the-software)
4. [Package In Action](#package-in-action)
    - [Smach Viewer](#smach-viewer)
    - [Rqt Diagram](#rqt-diagram)
5. [Files Description](#files-description)
6. [Authors and Contacts](#authors-and-contacts)

## 1) Introduction
The objective of this assignment is to develop an algorithm for a surveillance robot to determine its movement in an environment with interconnected locations. The robot will exhibit surveillance behavior based on the situation using a Finite State Machine implemented with SMACH. The robot has a battery that needs periodic recharging, and it can move between rooms and corridors in the environment.

## 2) Software Architecture
![Assignment_Component_Diagram drawio](https://github.com/raja-27/exp1/assets/81862489/1b73029a-ddc8-46ab-adc3-d5e5629a374e)
![Assignment_Sequence_Diagram drawio](https://github.com/raja-27/exp1/assets/81862489/65fd7f60-d0b3-4ee2-97fc-b1ef78898a20)



### I) Robot Behavior
The robot's behavior is governed by a Finite State Machine (FSM) designed using the SMACH library. The FSM guides the robot through different states and transitions based on its current conditions. The states and their behaviors are as follows:

- **Initialization**: The robot starts in Room E and waits until the ontology (map) is constructed.
- **Normal Mode (in_corridor)**: Once the ontology is ready, the robot enters the "Normal Mode" state. Here, it checks whether the battery is not low and there are no rooms requiring immediate attention. If these conditions are met, the robot moves randomly through the two corridors and pauses for a while. This behavior allows the robot to cover different areas of the environment while conserving energy.
- **Visit Rooms (room_emergency)**: If the battery is not low but there is a room in need of immediate attention, the robot visits that room and remains there for some time. This state prioritizes the needs of specific rooms over random movement.
- **Recharging (In_E_corridor)**: If the battery is low, the robot transitions to the "Recharging" state, where it stays in Room E until the battery is fully charged. This ensures that the robot can continue its surveillance tasks without interruption.

### II) Final Nodes Diagram
The "Final Nodes Diagram" illustrates the connections between all nodes in the software architecture. It shows how the robot-state node sends the battery level to the Finite State Machine and communicates with the planner and controller nodes. The planner and controller nodes are used to plan and control the robot's movement, and they interact with the Armor node to query and determine the properties of the robot's setup. The diagram helps visualize the flow of information and control within the system.

### III) Finite State Machine Diagram
The "Finite State Machine Diagram" presents a visual representation of the main software, which is the Finite State Machine. It consists of four states: Load_Environment, Normal Mode (in_corridor), Visit Rooms (room_emergency), and Recharging (In_E_corridor). Each state is represented as a node, and the transitions between states are shown as arrows. This diagram gives an overview of how the robot's behavior is organized and how it transitions between different states based on its conditions.

## 3) Launching the Software
To run the surveillance robot software, follow these steps:

1. Clone this repository inside your ROS workspace and compile:
   ```bash
   $ cd <your ROS ws/src>
   $ git clone "https://github.com/raja-27/exp1.git"
   $ cd ..
   $ catkin_make
   ```
2. Make the scripts executable:
   ```bash
   $ chmod +x <file_name>
   ```
3. Install `xterm`:
   ```bash
   $ sudo apt install -y xterm
   ```
4. Use the following command to launch the software with randomized stimulus:
   ```bash
   roslaunch expo_assignment_1 survailence_robot.launch
   ```

## 4) Package In Action

### I) Smach Viewer
The "Smach Viewer" is a visualization tool that shows each state the robot is in during the execution. It provides a real-time view of the Finite State Machine and helps in debugging and understanding the robot's behavior. To use the Smach Viewer, first, install it with the following command:
```bash
$ sudo apt-get install ros-noetic-smach-viewer
```
Then, run the Smach Viewer with the following command:
```bash
$ rosrun smach_viewer smach_viewer.py
```

### II) Rqt Diagram
The "Rqt Diagram" is another visualization tool that displays the communication graph between nodes in the ROS system. It shows how different nodes interact with each other, including topics, services, and action connections. To use Rqt, first, install it with the following command:
```bash
$ sudo apt-get install ros-noetic-rqt
```
Then, run Rqt with the following command:
```bash
$ rosrun rqt_graph rqt_graph
```

## 5) Files Description
The "scripts" folder contains executable Python files that play different roles in the assignment:

1. **robot_state_machine.py**: This script defines the structure of the Finite State Machine, its states, and transitions. It initializes the FSM with all its states and their respective behaviors for each transition. To ensure synchronization and avoid issues with shared variables, the script uses a helper entity that acts as an attribute of the Class :mod:`helper`. The helper provides a mutex for accessing shared variables.
2. **robots_condition.py**: This script provides services for setting and retrieving the robot's position, as well as a publisher for broadcasting changes in battery level. The script implements two services, "state/set_pose" and "state/get_pose," to set and retrieve the robot's position using a Point object. The battery level changes are communicated through the "state/battery_low" topic using Boolean messages.
3. **topological_map/**: This folder contains ontology roles, concepts, and individuals used to describe the experimental environment. The ontology is built using **Protege** to define the relationships and properties of different entities in the environment.
4. **Load_environment.py**: This script initializes the environment's Finite State Machine with states and transitions. Similar to the **robot_state_machine.py**, it also uses a helper entity for synchronization and sharing data. The helper provides a mutex to access shared variables safely.
5. **robothelper.py**: This script implements a helper class that includes action clients for robot control and functions to retrieve information and queries. The helper class simplifies the code by providing useful functions and avoiding the use of many global variables.
6. **robotController.py**: This script defines a server used by the FSM to simulate the movement of the robot from a starting position to a target one. The server computes the path to follow, and in case of interruption (e.g., low battery signal), it returns nothing due to preemption.
7. **Planner.py**: This script defines a server used by the FSM to compute the path for the robot from a starting position to a target one.

 The server computes the path as a linear space on 'n' points between the two coordinates. In case of interruption (e.g., low battery signal), it returns nothing due to preemption.

## 6) Authors and Contacts
- **Author**: Raja Tahaa
- **Email**: rajatahaa@live.com

---
