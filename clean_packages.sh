#!/bin/bash

(cd sbpl_arm_planner && make clean && cd ..)
(cd sbpl_arm_planner_node && make clean && cd ..)
(cd sbpl_collision_checking && make clean && cd ..)
(cd simple_arm_control && make clean && cd ..)
(cd visualize_arm && make clean && cd ..)
