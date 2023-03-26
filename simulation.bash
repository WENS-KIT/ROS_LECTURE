#!/bin/bash

gnome-terminal --tab --title="gazebo" -- bash -c "bash ~/lecture_file/px4_gazebo.bash; exec bash"
gnome-terminal --tab --title="teleop" -- bash -c "sleep 5 && python ~/lecture_file/px4_teleop.py; exec bash"