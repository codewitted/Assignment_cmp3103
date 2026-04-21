Autonomous Mobile Robots Assignment 1

Student Name: Benjamin Stuart
Student ID: 27517192

This repository contains the code for the autonomous mobile robots assignment 1.

This submission has solutions for:
Complexity Level 1: Detects the red floor patch, identifies any objects on the patch and pushes them off
Complexity Level 2: Differentiates between colours, clearing only the blue objetcs from the red patch
Complexity Level 3: Pushing an object through a complex environment towards a the red patch

Running the code:

To build the code: colcon build
To source the code: source install/setup.bash

Complexity 1:

Terminal 1:
ros2 launch uol_tidybot tidybot.launch.py

Terminal 2:
ros2 run uol_tidybot generate_objects --ros-args -p n_objects:=5 -p cx:=-0.7 -p  cy:=0.55 -p spread:=0.5
ros2 run uol_tidybot generate_objects --ros-args -p n_objects:=5 -p cx:=-0.7 -p  cy:=0.55 -p spread:=0.5 -p red:=False
ros2 run cmp3103m_ros2_code_fragments task1

Complexity 2: 

Terminal 1:
ros2 launch uol_tidybot tidybot.launch.py

Terminal 2:
ros2 run uol_tidybot generate_objects --ros-args -p n_objects:=5 -p cx:=-0.7 -p  cy:=0.55 -p spread:=0.5
ros2 run uol_tidybot generate_objects --ros-args -p n_objects:=5 -p cx:=-0.7 -p  cy:=0.55 -p spread:=0.5 -p red:=False
ros2 run cmp3103m_ros2_code_fragments task2

Complexity 3:

Terminal 1: 
ros2 launch uol_tidybot tidybot.launch.py world:=level_2_1.world

Terminal 2:
ros2 launch limo_navigation limo_navigation.launch.py map:=/workspaces/cmp3103/task3_map.yaml use_sim_time:=true

Terminal 3:
ros2 run  uol_tidybot generate_objects --ros-args -p n_objects:=1 -p cx:=0.7 -p  cy:=1.15 -p spread:=0.1
ros2 run cmp3103m_ros2_code_fragments task3

https://github.com/LCAS/ROB2002 - reference for waypoint movement used in complexity 3