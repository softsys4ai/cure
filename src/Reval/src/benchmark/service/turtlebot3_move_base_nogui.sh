#!/bin/sh

cd ..
cd ..
cd turtlebot3/turtlebot3/turtlebot3_navigation/launch
gnome-terminal -- roslaunch turtlebot3_navigation_nogui.launch map_file:=$HOME/cure/src/Reval/src/turtlebot3/turtlebot3/turtlebot3_navigation/maps/map.yaml
