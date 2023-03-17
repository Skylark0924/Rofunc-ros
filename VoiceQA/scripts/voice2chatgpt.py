#!/usr/bin/env python3
# coding=utf-8

import rospy
from std_msgs.msg import String
import sys

def voice2question(msg):
    rospy.loginfo("--------------------")

    if 'hi' in msg.data or 'hey' in msg.data:
        rospy.loginfo(msg.data)
        ask_pub.publish(msg)

    
if __name__ == "__main__":
    rospy.init_node("str_pub")

    question_sub = rospy.Subscriber("speech_recognition/final_result", String, voice2question, queue_size=10)

    # Publish questions to ChatGPT
    ask_pub = rospy.Publisher("/chatgpt_ask", String, queue_size=10)

    rospy.spin()
