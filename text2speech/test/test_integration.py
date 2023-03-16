#!/usr/bin/env python

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

from __future__ import print_function

import sys
import json
import unittest

import rospy
import rostest

from rofunc_ros.srv import Polly
from rofunc_ros.srv import PollyResponse
from rofunc_ros.srv import Synthesizer
from rofunc_ros.srv import SynthesizerResponse

# import tts which is a relay package, otherwise things don't work
#
#     devel/lib/python2.7/dist-packages/
#     +-- tts
#     |   +-- __init__.py
#     +-- ...
#
# per http://docs.ros.org/api/catkin/html/user_guide/setup_dot_py.html:
#
# A relay package is a folder with an __init__.py folder and nothing else.
# Importing this folder in python will execute the contents of __init__.py,
# which will in turn import the original python modules in the folder in
# the sourcespace using the python exec() function.


PKG = 'tts'
NAME = 'amazonpolly'


class TestPlainText(unittest.TestCase):

    def test_plain_text_to_wav_via_polly_node(self):
        rospy.wait_for_service('polly')
        polly = rospy.ServiceProxy('polly', Polly)

        test_text = 'Mary has a little lamb, little lamb, little lamb.'
        res = polly(polly_action='SynthesizeSpeech', text=test_text)
        self.assertIsNotNone(res)
        self.assertTrue(type(res) is PollyResponse)

        r = json.loads(res.result)
        self.assertIn('Audio Type', r, 'result should contain audio type')
        self.assertIn('Audio File', r, 'result should contain file path')
        self.assertIn('Amazon Polly Response Metadata', r, 'result should contain metadata')

        audio_type = r['Audio Type']
        audio_file = r['Audio File']
        md = r['Amazon Polly Response Metadata']
        self.assertTrue("'HTTPStatusCode': 200," in md)
        self.assertEqual('audio/ogg', audio_type)
        self.assertTrue(audio_file.endswith('.ogg'))

        import subprocess
        o = subprocess.check_output(['file', audio_file], stderr=subprocess.STDOUT)
        import re
        m = re.search(r'.*Ogg data, Vorbis audi.*', o, flags=re.MULTILINE)
        self.assertIsNotNone(m)

    def test_plain_text_using_polly_class(self):
        from tts.amazonpolly import AmazonPolly
        polly = AmazonPolly()
        test_text = 'Mary has a little lamb, little lamb, little lamb.'
        res = polly.synthesize(text=test_text)
        self.assertIsNotNone(res)
        self.assertTrue(type(res) is PollyResponse)

        r = json.loads(res.result)
        self.assertIn('Audio Type', r, 'result should contain audio type')
        self.assertIn('Audio File', r, 'result should contain file path')
        self.assertIn('Amazon Polly Response Metadata', r, 'result should contain metadata')

        audio_type = r['Audio Type']
        audio_file = r['Audio File']
        md = r['Amazon Polly Response Metadata']
        self.assertTrue("'HTTPStatusCode': 200," in md)
        self.assertEqual('audio/ogg', audio_type)
        self.assertTrue(audio_file.endswith('.ogg'))

        import subprocess
        o = subprocess.check_output(['file', audio_file], stderr=subprocess.STDOUT)
        import re
        m = re.search(r'.*Ogg data, Vorbis audi.*', o, flags=re.MULTILINE)
        self.assertIsNotNone(m)

    def test_plain_text_via_synthesizer_node(self):
        rospy.wait_for_service('synthesizer')
        speech_synthesizer = rospy.ServiceProxy('synthesizer', Synthesizer)

        text = 'Mary has a little lamb, little lamb, little lamb.'
        res = speech_synthesizer(text=text)
        self.assertIsNotNone(res)
        self.assertTrue(type(res) is SynthesizerResponse)

        r = json.loads(res.result)
        self.assertIn('Audio Type', r, 'result should contain audio type')
        self.assertIn('Audio File', r, 'result should contain file path')
        self.assertIn('Amazon Polly Response Metadata', r, 'result should contain metadata')

        audio_type = r['Audio Type']
        audio_file = r['Audio File']
        md = r['Amazon Polly Response Metadata']
        self.assertTrue("'HTTPStatusCode': 200," in md)
        self.assertEqual('audio/ogg', audio_type)
        self.assertTrue(audio_file.endswith('.ogg'))

        import subprocess
        o = subprocess.check_output(['file', audio_file], stderr=subprocess.STDOUT)
        import re
        m = re.search(r'.*Ogg data, Vorbis audi.*', o, flags=re.MULTILINE)
        self.assertIsNotNone(m)

    def test_plain_text_to_mp3_via_polly_node(self):
        rospy.wait_for_service('polly')
        polly = rospy.ServiceProxy('polly', Polly)

        test_text = 'Mary has a little lamb, little lamb, little lamb.'
        res = polly(polly_action='SynthesizeSpeech', text=test_text, output_format='mp3')
        self.assertIsNotNone(res)
        self.assertTrue(type(res) is PollyResponse)

        r = json.loads(res.result)
        self.assertIn('Audio Type', r, 'result should contain audio type')
        self.assertIn('Audio File', r, 'result should contain file path')
        self.assertIn('Amazon Polly Response Metadata', r, 'result should contain metadata')

        audio_type = r['Audio Type']
        audio_file = r['Audio File']
        md = r['Amazon Polly Response Metadata']
        self.assertTrue("'HTTPStatusCode': 200," in md)
        self.assertEqual('audio/mpeg', audio_type)
        self.assertTrue(audio_file.endswith('.mp3'))

        import subprocess
        o = subprocess.check_output(['file', audio_file], stderr=subprocess.STDOUT)
        import re
        m = re.search(r'.*MPEG.*layer III.*', o, flags=re.MULTILINE)
        self.assertIsNotNone(m)

    def test_simple_ssml_via_polly_node(self):
        rospy.wait_for_service('polly')
        polly = rospy.ServiceProxy('polly', Polly)

        text = '<speak>Mary has a little lamb, little lamb, little lamb.</speak>'
        res = polly(polly_action='SynthesizeSpeech', text=text, text_type='ssml')
        self.assertIsNotNone(res)
        self.assertTrue(type(res) is PollyResponse)

        r = json.loads(res.result)
        self.assertIn('Audio Type', r, 'result should contain audio type')
        self.assertIn('Audio File', r, 'result should contain file path')
        self.assertIn('Amazon Polly Response Metadata', r, 'result should contain metadata')

        audio_type = r['Audio Type']
        audio_file = r['Audio File']
        md = r['Amazon Polly Response Metadata']
        self.assertTrue("'HTTPStatusCode': 200," in md)
        self.assertEqual('audio/ogg', audio_type)
        self.assertTrue(audio_file.endswith('.ogg'))

        import subprocess
        o = subprocess.check_output(['file', audio_file], stderr=subprocess.STDOUT)
        import re
        m = re.search(r'.*Ogg data, Vorbis audi.*', o, flags=re.MULTILINE)
        self.assertIsNotNone(m)

    def test_simple_ssml_via_synthesizer_node(self):
        rospy.wait_for_service('synthesizer')
        speech_synthesizer = rospy.ServiceProxy('synthesizer', Synthesizer)

        text = '<speak>Mary has a little lamb, little lamb, little lamb.</speak>'
        res = speech_synthesizer(text=text, metadata='''{"text_type":"ssml"}''')
        self.assertIsNotNone(res)
        self.assertTrue(type(res) is SynthesizerResponse)

        r = json.loads(res.result)
        self.assertIn('Audio Type', r, 'result should contain audio type')
        self.assertIn('Audio File', r, 'result should contain file path')
        self.assertIn('Amazon Polly Response Metadata', r, 'result should contain metadata')

        audio_type = r['Audio Type']
        audio_file = r['Audio File']
        md = r['Amazon Polly Response Metadata']
        self.assertTrue("'HTTPStatusCode': 200," in md)
        self.assertEqual('audio/ogg', audio_type)
        self.assertTrue(audio_file.endswith('.ogg'))

        import subprocess
        o = subprocess.check_output(['file', audio_file], stderr=subprocess.STDOUT)
        import re
        m = re.search(r'.*Ogg data, Vorbis audi.*', o, flags=re.MULTILINE)
        self.assertIsNotNone(m)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestPlainText, sys.argv)
