#!/usr/bin/env python3
# coding=utf-8

import sys
import actionlib
import rospy
from rofunc_ros.msg import SpeechAction, SpeechGoal
from std_msgs.msg import String, Bool


def response2voice(msg):
    rospy.loginfo("[Rofunc-ros: Chat2Voice] --------------------")
    rospy.loginfo(msg.data)

    global client

    goal = SpeechGoal()

    msg = str(msg.data).replace("/", " ")
    msg = str(msg).replace("\\", " ")
    msg = str(msg).replace("\\n", " ")
    goal.text = msg
    goal.metadata = ''

    stop_listen_pub.publish(False)

    client.send_goal(goal)
    client.wait_for_result()
    print('\n' + client.get_result().response)
    stop_listen_pub.publish(True)

    

if __name__ == '__main__':
    rospy.init_node('tts_action_client')
    client = actionlib.SimpleActionClient('tts', SpeechAction)
    client.wait_for_server()

    stop_listen_pub = rospy.Publisher("/t2s/status", Bool, queue_size=1)

    response_sub = rospy.Subscriber("/chatgpt_answer", String, response2voice, queue_size=10)
    rospy.spin()




