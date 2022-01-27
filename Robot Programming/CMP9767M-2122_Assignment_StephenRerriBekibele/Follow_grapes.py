#! /usr/bin/env python

#The Construct. Exploring ROS with a 2 wheeled robot #7 - Wall Follower Algorithm, Available at: https://www.theconstructsim.com/wall-follower-algorithm/. [Accessed 26th JAnuary 20222]

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Define variables for the directions we want the robot to be able to turn
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
#This state variable allows us to switch between the states we define in stae_dict_
state_ = 0
state_dict_ = {
    0: 'find the grapes',
    1: 'turn left',
    2: 'follow the grapes',
    3: 'turn right',
}
#Created a laser callback to recieve the data from the laser sensor
def clbk_laser(msg):
    global regions_
    regions_= {
        'right': min(min(msg.ranges[0:143]), 10),
        'fright':min(min(msg.ranges[144:287]), 10),
        'front':min(min(msg.ranges[288:431]), 10),
        'fleft':min(min(msg.ranges[432:575]), 10),
        'left':min(min(msg.ranges[576:713]), 10),
    }
    take_action()
#This state variable determines how the robot will switch between each possible state 
def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print('Grape follower - [%s] - %s' %(state,state_dict_[state]))
        state_ = state
#This function decides what actions the robot takes in the differnt states
def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    d = 1.5
    
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

#A Function to enable the robot to drive around until a grape wall is detected.
def find_grapes():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.1
    return msg
#We define an action for each state we want the robot to be able to be in.
def turn_left():
    msg = Twist()
    msg.angular.z = 0.1
    return msg

def follow_the_grapes():
    global regions_
    msg = Twist()
    msg.linear.x = 0.5
    return msg

def turn_right():
    msg = Twist()
    msg.angular.z = 0.1

#The main function allows the robot to do different actions depending on the state it is in. 
def main():
    global pub_
    
    rospy.init_node('reading_laser')#reading_laser
    
    pub_ = rospy.Publisher('/thorvald_001/teleop_joy/cmd_vel', Twist, queue_size=1)
    
    rospy.Subscriber('/thorvald_001/front_scan', LaserScan, clbk_laser)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_grapes()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_grapes()
        elif state_ == 3:
            msg = turn_right()
            pass
        else:
            rospy.logerr('Unknown state!')
        pub_.publish(msg)
        
        rate.sleep()
    print("Following grape wall...")
if __name__ == '__main__':
    main()