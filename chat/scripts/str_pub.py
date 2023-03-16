#!/usr/bin/env python3
# coding=utf-8

import rospy
from std_msgs.msg import String
import sys

if __name__ == "__main__":
    rospy.init_node("str_pub")

    if(len(sys.argv) > 1):
        # rospy.loginfo(sys.argv[1])
        question_msg = String()
        question_msg.data = sys.argv[1]

        # Publish questions to ChatGPT
        ask_pub = rospy.Publisher("/chatgpt_ask", String, queue_size=1)
        rospy.sleep(0.1)
        ask_pub.publish(question_msg)
    else:
        rospy.logwarn("Please add your question after the command with quotation marks")
    
    rospy.sleep(0.1)