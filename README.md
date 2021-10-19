# MSc-Robotics-and-Autonomous-Systems
4th Year at the University of Lincoln.

Advanced Artificial Intelligence - Focuses on the theoretical fundamentals and practical applications of decision making, problem solving and interference ablilities in software agents. This module is python baised and available sources include google colab as well as jupiter notebook

Robot Programming - Focuses on programming robots in ROS
instructions found on : (https://github.com/LCAS/CMP9767M/wiki/Workshop-1---Introduction-and-ROS-Basics)

<--------------------------Install dependencies----------------------->

sudo apt-get update && sudo apt-get upgrade
sudo apt-get install ros-melodic-uol-cmp9767m-base ros-melodic-desktop

use gedit to open .bashrc
gedit .bashrc

append source /opt/ros/melodic/setup.bash to the bottom of .bashrc
then run "source .bashrc" in terminal as below

source .bashrc
sudo chown -R computing:computing ~./ros
<----------------launch the thorvald simulation------------------------>

roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small

You may kill all simulator instances with "killall -9 gzserver"

<-------------------Launch thorvald with the keyop------------------>
find image example in Robot programming/useful images
rostopic pub /thorvald_001/teleop_joy/cmd_vel geetry_msgs/Twist "linear:

-r tells ROS to do something at a given rate and not terminate.
