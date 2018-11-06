#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

print "running..."
rospy.init_node('publisher_2')
pub = rospy.Publisher('doubled', Int32, queue_size=10)
rate = rospy.Rate(1)

count = 0


while not rospy.is_shutdown():
  pub.publish(count)
  print(count)
  count = count + 1
  rate.sleep()

