#!/usr/bin/env python
import rospy
from python_services.srv import WordCount,WordCountResponse
def count_words(request):
  return WordCountResponse(len(request.words.split()))
rospy.init_node('service_server')
service = rospy.Service('word_count', WordCount, count_words)
rospy.spin()
