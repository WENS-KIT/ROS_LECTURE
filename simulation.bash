#!/bin/bash

gnome-terminal --tab --title="gazebo" -- bash -c "./px4_gazebo.bash; exec bash"
gnome-terminal --tab --title="teleop" -- bash -c "./px4_teleop.bash; exec bash"