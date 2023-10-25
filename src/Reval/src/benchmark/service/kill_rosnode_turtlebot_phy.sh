#!/bin/sh

robag=$( rosnode kill /my_bag &>/dev/null )
caldis=$( rosnode kill /calculate_distance_traveled &>/dev/null)
kill_check_battery=$( rosnode kill reval_tbot_battery &>/dev/null )
kill_obsDistance=$( rosnode kill /turtlebot3_obstacle &>/dev/null )
# all=$( killall -e roslaunch &>/dev/null)