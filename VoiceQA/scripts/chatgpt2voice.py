#!/usr/bin/env python3
# coding=utf-8

import sys
import actionlib
import rospy
from rofunc_ros.msg import SpeechAction, SpeechGoal
from std_msgs.msg import String


def response2voice(msg):
    rospy.loginfo("--------------------")
    rospy.loginfo(msg.data)

    global client

    goal = SpeechGoal()

    goal.text = msg
    goal.metadata = ''

    client.send_goal(goal)
    client.wait_for_result()
    print('\n' + client.get_result().response)

if __name__ == '__main__':
    rospy.init_node('tts_action_client')
    client = actionlib.SimpleActionClient('tts', SpeechAction)
    client.wait_for_server()

    response_sub = rospy.Subscriber("/chatgpt_answer", String, response2voice, queue_size=10)




