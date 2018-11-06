#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

print "running..."
rospy.init_node('double_node')
pub = rospy.Publisher('doubled', Int32, queue_size=10)
# pub.publish(count)
rate = rospy.Rate(2)

def callback(msg):
    number =  msg.data
    pub.publish( number * 2 ) 
    rate.sleep()

sub = rospy.Subscriber('number', Int32, callback)
rospy.spin()



# rospy.spin()
