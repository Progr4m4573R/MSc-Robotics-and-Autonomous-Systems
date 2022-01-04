
                           <--------------------------Install dependencies----------------------->

sudo apt-get update && sudo apt-get upgrade


sudo apt-get install ros-melodic-uol-cmp9767m-base ros-melodic-desktop

use gedit to open .bashrc:

gedit ~/.bashrc

append source /opt/ros/melodic/setup.bash to the bottom of .bashrc
then run "source ~/.bashrc" in terminal as below

source .bashrc
                           <---------------Fixes error with lab computer permisisons---------------->
sudo chown -R computing:computing ~/.ros
                            <----------------launch the thorvald simulation------------------------>

Launch with 2 robots:

roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small multi_sim:=true

Launch in a small vineyard:

roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small

You may kill all simulator instances with "killall -9 gzserver"

                              <-------------------Launch thorvald with the keyop------------------>

find image example in Robot programming/useful images
rostopic pub /thorvald_001/teleop_joy/cmd_vel geetry_msgs/Twist "linear:

-r tells ROS to do something at a given rate and not terminate.
                                    <------------------Useful commands-------------------->

type -h next to any commands for more information

rostpic list -v "Lists active publishers and subscribers "
rosservice list "List active services"
rosmsg list/ show - shows a list of all predefined message types 
rossrv - displays information about ros service types
roscore -  starts a new roscore server if one is not active
rosnode list - lists all active nodes

rqt_image_view - shows a robot's perspective in ROS
rosrun rqt_tf_tree rqt_tf_tree - used to display the tf tree of the thorvald robot
using pipelinning we can access specific data about topics such as the thorvald frontscan
rostopic echo /thorvald_001/front_scan
rostopic echo /thorvald_001/front_scan | grep range_max
rostopic echo /thorvald_001/front_scan | grep range_min
rostopic echo /thorvald_001/front_scan | grep angle_max
rostopic echo /thorvald_001/front_scan | grep angle_min

                          <------Showing messages about a topics data such as LaserScan used by thorvald------->
rosmsmg show sensor_msgs/LaserScan or rosmsg info sensor_msgs/LaserScan

                                   <----------showing information about certain topics--------->
rostopic info/ thorvald_001_nav_vel
                                     <---------------Creating a publisher------------>
  rostopic pub /meminfo std_msgs/String "data:'Hey'"
<If you get issues with rqt_launchtree this webpage can help>
  https://answers.ros.org/question/91231/rqt-plugin-not-listedfound-in-list-returned-by-rqt-list-plugins/
Useful information:
Navigating the ROS Filesystem:http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem
