# ROS2-Project-4
First step to do before running anything, proceed to: /home/$USER/proj4_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_bringup/worlds/Library_Punggol

Unzip the zip file and paste it in the same directory.

Remember to source the bashrc and edit the source to proj4_ws/install/setup.bash

Do add in the bashrc file within the last few lines:

export ROS_DOMAIN_ID=30 #TURTLEBOT3
export TURTLEBOT3_MODEL="waffle_pi"
export LC_NUMERIC="en_US.UTF-8"

For simulation and navigation, run commands:
1) ros2 launch turtlebot3_manipulation_bringup riot_waffle_nav_launch.xml

To test the plates in the moveit_config files, run commands:
1) ros2 launch turtlebot3_manipulation_moveit_config moveit_gazebo.launch.py 
