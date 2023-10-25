#!/bin/sh

robag=$( rosnode kill /my_bag &>/dev/null )
caldis=$( rosnode kill /calculate_distance_traveled &>/dev/null)
kill_check_battery=$( rosnode kill /battery_consumer &>/dev/null )
kill_mvobs_1=$( rosnode kill /combination_obstacle_1 &>/dev/null )
kill_mvobs_2=$( rosnode kill /combination_obstacle_2 &>/dev/null )
kill_obsDistance=$( rosnode kill /turtlebot3_obstacle &>/dev/null )
all=$( killall -e roslaunch &>/dev/null)