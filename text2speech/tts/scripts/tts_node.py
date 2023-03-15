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

"""A very simple Action Server that does TTS.

It is a combination of a synthesizer and a player. Being an action server, it can be used in two different manners.

1. Play and wait for it to finish
---------------------------------

A user can choose to be blocked until the audio playing is done. This is especially useful in interactive scenarios.

Example::

    rospy.init_node('tts_action_client')
    client = actionlib.SimpleActionClient('tts', SpeechAction)
    client.wait_for_server()
    goal = SpeechGoal()
    goal.text = 'Let me ask you a question, please give me your answer.'
    client.send_goal(goal)
    client.wait_for_result()

    # start listening to a response or waiting for some input to continue the interaction

2. Play and forget
------------------

A user can also choose not to wait::

    rospy.init_node('tts_action_client')
    client = actionlib.SimpleActionClient('tts', SpeechAction)
    client.wait_for_server()
    goal = SpeechGoal()
    goal.text = 'Let me talk, you can to something else in the meanwhile.'
    client.send_goal(goal)

This is useful when the robot wants to do stuff while the audio is being played. For example, a robot may start to
read some instructions and immediately get ready for any input.
"""

import json

import actionlib
import rospy
from tts.msg import SpeechAction, SpeechResult
from tts.srv import Synthesizer

from sound_play.libsoundplay import SoundClient


def play(filename):
    """plays the wav or ogg file using sound_play"""
    SoundClient(blocking=True).playWave(filename)


def do_synthesize(goal):
    """calls synthesizer service to do the job"""
    rospy.wait_for_service('synthesizer')
    synthesize = rospy.ServiceProxy('synthesizer', Synthesizer)
    return synthesize(goal.text, goal.metadata)


def finish_with_result(s):
    """responds the client"""
    tts_server_result = SpeechResult(s)
    server.set_succeeded(tts_server_result)
    rospy.loginfo(tts_server_result)


def do_speak(goal):
    """The action handler.

    Note that although it responds to client after the audio play is finished, a client can choose
    not to wait by not calling ``SimpleActionClient.waite_for_result()``.
    """
    rospy.loginfo('speech goal: {}'.format(goal))

    res = do_synthesize(goal)
    rospy.loginfo('synthesizer returns: {}'.format(res))

    try:
        r = json.loads(res.result)
    except Exception as e:
        s = 'Expecting JSON from synthesizer but got {}'.format(res.result)
        rospy.logerr('{}. Exception: {}'.format(s, e))
        finish_with_result(s)
        return

    result = ''

    if 'Audio File' in r:
        audio_file = r['Audio File']
        rospy.loginfo('Will play {}'.format(audio_file))
        play(audio_file)
        result = audio_file

    if 'Exception' in r:
        result = '[ERROR] {}'.format(r)
        rospy.logerr(result)

    finish_with_result(result)


if __name__ == '__main__':
    rospy.init_node('tts_node')
    server = actionlib.SimpleActionServer('tts', SpeechAction, do_speak, False)
    server.start()
    rospy.spin()
