#!/bin/bash

catkin build

. devel/setup.bash

echo ". devel/setup.bash"
echo ""

roslaunch ec2_bringup miiwa.simulation.launch

