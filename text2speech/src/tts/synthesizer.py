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

import os
import time
import json
import rospy
import hashlib
from optparse import OptionParser
from tts.srv import Synthesizer, SynthesizerResponse


class SpeechSynthesizer:
    """This class serves as a ROS service node that should be an entry point of a TTS task.

    Although the current implementation uses Amazon Polly as the synthesis engine, it is not hard to let it support
    more heterogeneous engines while keeping the API the same.

    In order to support a variety of engines, the SynthesizerRequest was designed with flexibility in mind. It
    has two fields: text and metadata. Both are strings. In most cases, a user can ignore the metadata and call
    the service with some plain text. If the use case needs any control or engine-specific feature, the extra
    information can be put into the JSON-form metadata. This class will use the information when calling the engine.

    The decoupling of the synthesizer and the actual synthesis engine will benefit the users in many ways.

    First, a user will be able to use a unified interface to do the TTS job and have the freedom to use different
    engines available with no or very little change from the client side.

    Second, by applying some design patterns, the synthesizer can choose an engine dynamically. For example, a user
    may prefer to use Amazon Polly but is also OK with an offline solution when network is not reliable.

    Third, engines can be complicated, thus difficult to use. As an example, Amazon Polly supports dozens of parameters
    and is able to accomplish nontrivial synthesis jobs, but majority of the users never need those features. This
    class provides a clean interface with two parameters only, so that it is much easier and pleasant to use. If by
    any chance the advanced features are required, the user can always leverage the metadata field or even go to the
    backend engine directly.

    Also, from an engineering perspective, simple and decoupled modules are easier to maintain.

    This class supports two modes of using polly. It can either call a service node or use AmazonPolly as a library.

    Start the service node::

        $ rosrun tts synthesizer_node.py  # use default configuration
        $ rosrun tts synthesizer_node.py -e POLLY_LIBRARY  # will not call polly service node

    Call the service::

        $ rosservice call /synthesizer 'hello' ''
        $ rosservice call /synthesizer '<speak>hello</speak>' '"{\"text_type\":\"ssml\"}"'
    """

    class PollyViaNode:
        def __init__(self, polly_service_name='polly'):
            self.service_name = polly_service_name

        def __call__(self, **kwargs):
            rospy.loginfo('will call service {}'.format(self.service_name))
            from tts.srv import Polly
            rospy.wait_for_service(self.service_name)
            polly = rospy.ServiceProxy(self.service_name, Polly)
            return polly(polly_action='SynthesizeSpeech', **kwargs)

    class PollyDirect:
        def __init__(self):
            pass

        def __call__(self, **kwargs):
            rospy.loginfo('will import amazonpolly.AmazonPolly')
            from tts.amazonpolly import AmazonPolly
            node = AmazonPolly()
            return node.synthesize(**kwargs)

    ENGINES = {
        'POLLY_SERVICE': PollyViaNode,
        'POLLY_LIBRARY': PollyDirect,
    }

    class BadEngineError(NameError):
        pass

    def __init__(self, engine='POLLY_SERVICE', polly_service_name='polly'):
        if engine not in self.ENGINES:
            msg = 'bad engine {} which is not one of {}'.format(engine, ', '.join(SpeechSynthesizer.ENGINES.keys()))
            raise SpeechSynthesizer.BadEngineError(msg)

        engine_kwargs = {'polly_service_name': polly_service_name} if engine == 'POLLY_SERVICE' else {}
        self.engine = self.ENGINES[engine](**engine_kwargs)

        self.default_text_type = 'text'
        self.default_voice_id = 'Joanna'
        self.default_output_format = 'ogg_vorbis'

    def _call_engine(self, **kw):
        """Call engine to do the job.

        If no output path is found from input, the audio file will be put into /tmp and the file name will have
        a prefix of the md5 hash of the text.

        :param kw: what AmazonPolly needs to synthesize
        :return: response from AmazonPolly
        """
        if 'output_path' not in kw:
            tmp_filename = hashlib.md5(kw['text'].encode("utf-8")).hexdigest()
            tmp_filepath = os.path.join(os.sep, 'tmp', 'voice_{}_{}'.format(tmp_filename, str(time.time())))
            kw['output_path'] = os.path.abspath(tmp_filepath)
        rospy.loginfo('audio will be saved as {}'.format(kw['output_path']))

        return self.engine(**kw)

    def _parse_request_or_raise(self, request):
        """It will raise if request is malformed.

        :param request: an instance of SynthesizerRequest
        :return: a dict
        """
        md = json.loads(request.metadata) if request.metadata else {}

        md['output_format'] = md.get('output_format', self.default_output_format)
        md['voice_id'] = md.get('voice_id', self.default_voice_id)
        md['sample_rate'] = md.get('sample_rate', '16000' if md['output_format'].lower() == 'pcm' else '22050')
        md['text_type'] = md.get('text_type', self.default_text_type)
        md['text'] = request.text

        return md

    def _node_request_handler(self, request):
        """The callback function for processing service request.

        It never raises. If anything unexpected happens, it will return a SynthesizerResponse with the exception.

        :param request: an instance of SynthesizerRequest
        :return: a SynthesizerResponse
        """
        rospy.loginfo(request)
        try:
            kws = self._parse_request_or_raise(request)
            res = self._call_engine(**kws).result

            return SynthesizerResponse(res)
        except Exception as e:
            return SynthesizerResponse('Exception: {}'.format(e))

    def start(self, node_name='synthesizer_node', service_name='synthesizer'):
        """The entry point of a ROS service node.

        :param node_name: name of ROS node
        :param service_name:  name of ROS service
        :return: it doesn't return
        """
        rospy.init_node(node_name)

        service = rospy.Service(service_name, Synthesizer, self._node_request_handler)

        rospy.loginfo('{} running: {}'.format(node_name, service.uri))

        rospy.spin()


def main():
    usage = '''usage: %prog [options]
    '''

    parser = OptionParser(usage)

    parser.add_option("-n", "--node-name", dest="node_name", default='synthesizer_node',
                      help="name of the ROS node",
                      metavar="NODE_NAME")
    parser.add_option("-s", "--service-name", dest="service_name", default='synthesizer',
                      help="name of the ROS service",
                      metavar="SERVICE_NAME")
    parser.add_option("-e", "--engine", dest="engine", default='POLLY_SERVICE',
                      help="name of the synthesis engine",
                      metavar="ENGINE")
    parser.add_option("-p", "--polly-service-name", dest="polly_service_name", default='polly',
                      help="name of the polly service",
                      metavar="POLLY_SERVICE_NAME")

    (options, args) = parser.parse_args()

    node_name = options.node_name
    service_name = options.service_name
    engine = options.engine
    polly_service_name = options.polly_service_name

    if engine == 'POLLY_SERVICE':
        synthesizer = SpeechSynthesizer(engine=engine, polly_service_name=polly_service_name)
    else:
        synthesizer = SpeechSynthesizer(engine=engine)
    synthesizer.start(node_name=node_name, service_name=service_name)


if __name__ == "__main__":
    main()
