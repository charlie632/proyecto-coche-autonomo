#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

print "running..."
rospy.init_node('subs_2')

# pub.publish(count)
rate = rospy.Rate(1)

def callback(msg):
    number =  msg.data
    print(number*2)

sub = rospy.Subscriber('doubled', Int32, callback)

rospy.spin()

