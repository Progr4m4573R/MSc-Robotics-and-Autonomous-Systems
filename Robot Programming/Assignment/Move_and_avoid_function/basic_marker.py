import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MarkerBasics(object):
    def __init__(self):
        self.marker_objectlisher = rospy.Publisher('/marker_basic', Marker, queue_size=1)
        self.rate = rospy.Rate(1)
        self.init_marker(index=0,x=-9,y=-7.5,z=0.6)# changing this moves the marker

    def init_marker(self,index, x,y,z):
        self.marker_object = Marker()
        self.marker_object.header.frame_id = "map" #the information that the marker contains is relative to this frame. 
        self.marker_object.header.stamp    = rospy.get_rostime()
        self.marker_object.ns = "thorvald_001" # this is the namespace for the robot
        self.marker_object.id = index# The consecutive index we start in
        self.marker_object.type = Marker.SPHERE # type of marker
        self.marker_object.action = Marker.ADD

        my_point = Point()#the location at which the sphere spwans that way we can get the coordinates of the marker.
        my_point.z = z# change this value to relocate the marker.
        my_point.x = x
        my_point.y = y
        self.marker_object.pose.position = my_point

        self.marker_object.pose.orientation.x = 0
        self.marker_object.pose.orientation.y = 0
        self.marker_object.pose.orientation.z = 0.0
        self.marker_object.pose.orientation.w = 1.0
        self.marker_object.scale.x = 0.3
        self.marker_object.scale.y = 0.3
        self.marker_object.scale.z = 0.3

        self.marker_object.color.r = 1.0
        self.marker_object.color.g = 0.0
        self.marker_object.color.b = 1.0
        # This has to be, otherwise it will be transparent
        self.marker_object.color.a = 1.0

        # If we want it forever, 0, otherwise seconds before desapearing
        self.marker_object.lifetime = rospy.Duration(0)



    def start(self):
        while not rospy.is_shutdown():
            self.marker_objectlisher.publish(self.marker_object)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('marker_basic_node', anonymous=True)
markerbasics_object = MarkerBasics()
try:
    markerbasics_object.start()
except rospy.ROSInterruptException:
    pass