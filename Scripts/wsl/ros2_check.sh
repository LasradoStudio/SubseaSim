#!/usr/bin/env bash
# Report what the SubseaSim ROS 2 bridge is publishing: topics, rates, one TF lookup.
#   wsl -d Ubuntu-24.04 -u root -- bash Scripts/wsl/ros2_check.sh   (from the repo root)
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

echo "== processes"
ps -eo pid,etime,args | grep -E "ros2_bridge|rviz2" | grep -v grep || echo "(none)"
echo "== topics"
ros2 topic list --no-daemon
for t in points odom sonar/raw sonar/image telemetry; do
    rate=$(timeout 8 ros2 topic hz --no-daemon "/subsea/$t" 2>&1 | grep -m1 "average rate")
    echo "/subsea/$t: ${rate:-no messages}"
done
echo "== tf map -> base_link"
timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -m1 -A1 "At time" || echo "(no transform)"
