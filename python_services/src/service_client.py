#!/usr/bin/env python
import rospy
from python_services.srv import WordCount
import sys


rospy.init_node('service_client')
rospy.wait_for_service('word_count')
word_counter = rospy.ServiceProxy('word_count', WordCount)
words = ' '.join(sys.argv[1:])
word_count = word_counter(worrods)

print words, '->', word_count.count

