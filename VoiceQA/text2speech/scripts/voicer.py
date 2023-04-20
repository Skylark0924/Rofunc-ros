#!/usr/bin/env python3

# Copyright (c) 2018, Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
# You may not use this file except in compliance with the License.
# A copy of the License is located at
#
#  http://aws.amazon.com/apache2.0
#
# or in the "license" file accompanying this file. This file is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing
# permissions and limitations under the License.

import sys
import actionlib
import rospy
from rofunc_ros.msg import SpeechAction, SpeechGoal


if __name__ == '__main__':
    rospy.init_node('tts_action_client')
    client = actionlib.SimpleActionClient('tts', SpeechAction)
    client.wait_for_server()

    goal = SpeechGoal()

    print(sys.argv[1])
    goal.text = str(sys.argv[1]) if len(sys.argv) > 1 else 'I got no idea.'
    goal.metadata = str(sys.argv[2]) if len(sys.argv) > 2 else ''

    print(goal)

    client.send_goal(goal)
    client.wait_for_result()
    print('\n' + client.get_result().response)
