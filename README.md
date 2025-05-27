# rUBot_mecanum_
This rubot_mecanum workspace is organized in the following sections:
| Document | Contents   |
|------|------|
|   1. rUBot Mecanum Setup  | Install and Setup rUBot_mecanum workspace|
|   2. rUBot Mecanum Bringup  | Bringup the rUBot_mecanum|
|   3. rUBot Mecanum Control  | Navigation and control|
| 4. rUBot HW Mecanum Slam&Navigation | Slam and Navigation |
| 5. rUBot  Projects | SW and HW projects |
| 



roslaunch rubot_control rubot_self_control.launch
roslaunch rubot_control rubot_self_control_holonomic.launch
roslaunch rubot_control rubot_wall_follower_holonomic

roslaunch rubot_slam rubot_slam.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
cd src/rubot_slam/maps
rosrun map_server map_saver -f test_map
roslaunch rubot_slam rubot_navigation.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
roslaunch rubot_projects keras_takePictures_detect_signs.launch

sudo apt-get update
sudo apt-get install python3-opencv
sudo apt install python3-pip
sudo pip install numpy matplotlib
sudo pip install keras
sudo pip install tensorflow

