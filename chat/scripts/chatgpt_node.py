#!/usr/bin/env python3
# coding=utf-8

import rospy
from std_msgs.msg import String
from urllib import response
import openai
   
def cbQuestion(msg):
    rospy.loginfo("--------------------")
    rospy.loginfo(msg.data)

    global conversation
    global msg_log
    if(conversation == True):
        msg_log.append({"role": "user", "content": msg.data})
    else:
        msg_log = [{"role": "user", "content": msg.data}]

    # print(msg_log)

    global model_engine
    completion = openai.ChatCompletion.create(
        model=model_engine,
        messages=msg_log,

    )

    response = completion["choices"][0]["message"]["content"]
    rospy.logwarn(response)

    if(conversation == True):
        msg_log.append({"role": "assistant", "content": response})

if __name__ == "__main__":
    rospy.init_node("chatgpt_node")

    api_key =  rospy.get_param('~openai/api_key')
    model_engine =  rospy.get_param('~openai/model' , "gpt-3.5-turbo")
    personality =  rospy.get_param('~openai/personality' , "")
    conversation =  rospy.get_param('~openai/conversation' , "False")

    openai.api_key = api_key

    rospy.logwarn("ChatGPT: current used model %s",model_engine)

    question_sub = rospy.Subscriber("/wpr_ask", String, cbQuestion, queue_size=1)

    response_pub = rospy.Publisher("/chatspt_answer", String, queue_size=1)

    msg_log = []

    if(len(personality) > 0):
        msg_log.append({"role": "system", "content": "You are "+personality})
        rospy.logwarn("ChatGPT: I'm "+personality+" I'm ready! Please ask your questions ^_^")
    else:
        rospy.logwarn("ChatGPT: I'm ready! Please ask your questions ^_^")

    rospy.spin()