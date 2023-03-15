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

import json
import os
import sys
import wave
import traceback
import requests
from boto3 import Session
from botocore.credentials import CredentialProvider, RefreshableCredentials
from botocore.session import get_session
from botocore.exceptions import UnknownServiceError
from contextlib import closing
from optparse import OptionParser

import rospy
from tts.srv import Polly, PollyRequest, PollyResponse


def get_ros_param(param, default=None):
    try:
        key = rospy.search_param(param)
        return default if key is None else rospy.get_param(key, default)
    except Exception as e:
        rospy.logwarn('Failed to get ros param {}, will use default {}. Exception: '.format(param, default, e))
        return default


class AwsIotCredentialProvider(CredentialProvider):
    METHOD = 'aws-iot'
    CANONICAL_NAME = 'customIoTwithCertificate'

    DEFAULT_AUTH_CONNECT_TIMEOUT_MS = 5000
    DEFAULT_AUTH_TOTAL_TIMEOUT_MS = 10000

    def __init__(self):
        super(AwsIotCredentialProvider, self).__init__()
        self.ros_param_prefix = 'iot/'

    def get_param(self, param, default=None):
        return get_ros_param(self.ros_param_prefix + param, default)

    def retrieve_credentials(self):
        try:
            cert_file = self.get_param('certfile')
            key_file = self.get_param('keyfile')
            endpoint = self.get_param('endpoint')
            role_alias = self.get_param('role')
            connect_timeout = self.get_param('connect_timeout_ms', self.DEFAULT_AUTH_CONNECT_TIMEOUT_MS)
            total_timeout = self.get_param('total_timeout_ms', self.DEFAULT_AUTH_TOTAL_TIMEOUT_MS)
            thing_name = self.get_param('thing_name', '')

            if any(v is None for v in (cert_file, key_file, endpoint, role_alias, thing_name)):
                return None

            headers = {'x-amzn-iot-thingname': thing_name} if len(thing_name) > 0 else None
            url = 'https://{}/role-aliases/{}/credentials'.format(endpoint, role_alias)
            timeout = (connect_timeout, total_timeout - connect_timeout)  # see also: urllib3/util/timeout.py

            response = requests.get(url, cert=(cert_file, key_file), headers=headers, timeout=timeout)
            d = response.json()['credentials']

            rospy.loginfo('Credentials expiry time: {}'.format(d['expiration']))

            return {
                'access_key': d['accessKeyId'],
                'secret_key': d['secretAccessKey'],
                'token': d['sessionToken'],
                'expiry_time': d['expiration'],
            }
        except Exception as e:
            rospy.logwarn('Failed to fetch credentials from AWS IoT: {}'.format(e))
            return None

    def load(self):
        return RefreshableCredentials.create_from_metadata(
            self.retrieve_credentials(),
            self.retrieve_credentials,
            'aws-iot-with-certificate'
        )


class AmazonPolly:
    """A TTS engine that can be used in two different ways.

    Usage
    -----

    1. It can run as a ROS service node.

    Start a polly node::

        $ rosrun tts polly_node.py

    Call the service from command line::

        $ rosservice call /polly SynthesizeSpeech 'hello polly' '' '' '' '' '' '' '' '' [] [] 0 '' '' '' '' '' '' false

    Call the service programmatically::

        from tts.srv import Polly
        rospy.wait_for_service('polly')
        polly = rospy.ServiceProxy('polly', Polly)
        res = polly(**kw)

    2. It can also be used as a normal python class::

        AmazonPolly().synthesize(text='hi polly')

    PollyRequest supports many parameters, but the majority of the users can safely ignore most of them and just
    use the vanilla version which involves only one argument, ``text``.

    If in some use cases more control is needed, SSML will come handy. Example::

        AmazonPolly().synthesize(
            text='<speak>Mary has a <amazon:effect name="whispered">little lamb.</amazon:effect></speak>',
            text_type='ssml'
        )

    A user can also control the voice, output format and so on. Example::

        AmazonPolly().synthesize(
            text='<speak>Mary has a <amazon:effect name="whispered">little lamb.</amazon:effect></speak>',
            text_type='ssml',
            voice_id='Joey',
            output_format='mp3',
            output_path='/tmp/blah'
        )


    Parameters
    ----------

    Among the parameters defined in Polly.srv, the following are supported while others are reserved for future.

    * polly_action : currently only ``SynthesizeSpeech`` is supported
    * text : the text to speak
    * text_type : can be either ``text`` (default) or ``ssml``
    * voice_id : any voice id supported by Amazon Polly, default is Joanna
    * output_format : ogg (default), mp3 or pcm
    * output_path : where the audio file is saved
    * sample_rate : default is 16000 for pcm or 22050 for mp3 and ogg

    The following are the reserved ones. Note that ``language_code`` is rarely needed (this may seem counter-intuitive).
    See official Amazon Polly documentation for details (link can be found below).

    * language_code
    * lexicon_content
    * lexicon_name
    * lexicon_names
    * speech_mark_types
    * max_results
    * next_token
    * sns_topic_arn
    * task_id
    * task_status
    * output_s3_bucket_name
    * output_s3_key_prefix
    * include_additional_language_codes


    Links
    -----

    Amazon Polly documentation: https://docs.aws.amazon.com/polly/latest/dg/API_SynthesizeSpeech.html

    """

    def __init__(self, aws_access_key_id=None, aws_secret_access_key=None, aws_session_token=None, region_name=None):
        if region_name is None:
            region_name = get_ros_param('aws_client_configuration/region', default='us-west-2')

        self.polly = self._get_polly_client(aws_access_key_id, aws_secret_access_key, aws_session_token, region_name)
        self.default_text_type = 'text'
        self.default_voice_id = 'Joanna'
        self.default_output_format = 'ogg_vorbis'
        self.default_output_folder = '.'
        self.default_output_file_basename = 'output'

    def _get_polly_client(self, aws_access_key_id=None, aws_secret_access_key=None, aws_session_token=None,
                          region_name=None, with_service_model_patch=False):
        """Note we get a new botocore session each time this function is called.
        This is to avoid potential problems caused by inner state of the session.
        """
        botocore_session = get_session()

        if with_service_model_patch:
            # Older versions of botocore don't have polly. We can possibly fix it by appending
            # extra path with polly service model files to the search path.
            current_dir = os.path.dirname(os.path.abspath(__file__))
            service_model_path = os.path.join(current_dir, 'data', 'models')
            botocore_session.set_config_variable('data_path', service_model_path)
            rospy.loginfo('patching service model data path: {}'.format(service_model_path))

        botocore_session.get_component('credential_provider').insert_after('boto-config', AwsIotCredentialProvider())

        botocore_session.user_agent_extra = self._generate_user_agent_suffix()

        session = Session(aws_access_key_id=aws_access_key_id, aws_secret_access_key=aws_secret_access_key,
                          aws_session_token=aws_session_token, region_name=region_name,
                          botocore_session=botocore_session)

        try:
            return session.client("polly")
        except UnknownServiceError:
            # the first time we reach here, we try to fix the problem
            if not with_service_model_patch:
                return self._get_polly_client(aws_access_key_id, aws_secret_access_key, aws_session_token, region_name,
                                              with_service_model_patch=True)
            else:
                # we have tried our best, time to panic
                rospy.logerr('Amazon Polly is not available. Please install the latest boto3.')
                raise

    def _generate_user_agent_suffix(self):
        exec_env = get_ros_param('exec_env', 'AWS_RoboMaker').strip()
        if 'AWS_RoboMaker' in exec_env:
            ver = get_ros_param('robomaker_version', None)
            if ver:
                exec_env += '-' + ver.strip()
        ros_distro = get_ros_param('rosdistro', 'Unknown_ROS_DISTRO').strip()
        ros_version = get_ros_param('rosversion', 'Unknown_ROS_VERSION').strip()
        return 'exec-env/{} ros-{}/{}'.format(exec_env, ros_distro, ros_version)

    def _pcm2wav(self, audio_data, wav_filename, sample_rate):
        """per Amazon Polly official doc, the pcm in a signed 16-bit, 1 channel (mono), little-endian format."""
        wavf = wave.open(wav_filename, 'w')
        wavf.setframerate(int(sample_rate))
        wavf.setnchannels(1)  # 1 channel
        wavf.setsampwidth(2)  # 2 bytes == 16 bits
        wavf.writeframes(audio_data)
        wavf.close()

    def _make_audio_file_fullpath(self, output_path, output_format):
        """Makes a full path for audio file based on given output path and format.

        If ``output_path`` doesn't have a path, current path is used.

        :param output_path: the output path received
        :param output_format: the audio format, e.g., mp3, ogg_vorbis, pcm
        :return: a full path for the output audio file. File ext will be constructed from audio format.
        """
        head, tail = os.path.split(output_path)
        if not head:
            head = self.default_output_folder
        if not tail:
            tail = self.default_output_file_basename

        file_ext = {'pcm': '.wav', 'mp3': '.mp3', 'ogg_vorbis': '.ogg'}[output_format.lower()]
        if not tail.endswith(file_ext):
            tail += file_ext

        return os.path.realpath(os.path.join(head, tail))

    def _synthesize_speech_and_save(self, request):
        """Calls Amazon Polly and writes the returned audio data to a local file.

        To make it practical, three things will be returned in a JSON form string, which are audio file path,
        audio type and Amazon Polly response metadata.

        If the Amazon Polly call fails, audio file name will be an empty string and audio type will be "N/A".

        Please see https://boto3.readthedocs.io/reference/services/polly.html#Polly.Client.synthesize_speech
        for more details on Amazon Polly API.

        :param request: an instance of PollyRequest
        :return: a string in JSON form with two attributes, "Audio File" and "Amazon Polly Response".
        """
        kws = {
            'LexiconNames': request.lexicon_names if request.lexicon_names else [],
            'OutputFormat': request.output_format if request.output_format else self.default_output_format,
            'SampleRate': request.sample_rate,
            'SpeechMarkTypes': request.speech_mark_types if request.speech_mark_types else [],
            'Text': request.text,
            'TextType': request.text_type if request.text_type else self.default_text_type,
            'VoiceId': request.voice_id if request.voice_id else self.default_voice_id
        }

        if not kws['SampleRate']:
            kws['SampleRate'] = '16000' if kws['OutputFormat'].lower() == 'pcm' else '22050'

        rospy.loginfo('Amazon Polly Request: {}'.format(kws))
        response = self.polly.synthesize_speech(**kws)
        rospy.loginfo('Amazon Polly Response: {}'.format(response))

        if "AudioStream" in response:
            audiofile = self._make_audio_file_fullpath(request.output_path, kws['OutputFormat'])
            rospy.loginfo('will save audio as {}'.format(audiofile))

            with closing(response["AudioStream"]) as stream:
                if kws['OutputFormat'].lower() == 'pcm':
                    self._pcm2wav(stream.read(), audiofile, kws['SampleRate'])
                else:
                    with open(audiofile, "wb") as f:
                        f.write(stream.read())

            audiotype = response['ContentType']
        else:
            audiofile = ''
            audiotype = 'N/A'

        return json.dumps({
            'Audio File': audiofile,
            'Audio Type': audiotype,
            'Amazon Polly Response Metadata': str(response['ResponseMetadata'])
        })

    def _dispatch(self, request):
        """Amazon Polly supports a number of APIs. This will call the right one based on the content of request.

        Currently "SynthesizeSpeech" is the only recognized action. Basically this method just delegates the work
        to ``self._synthesize_speech_and_save`` and returns the result as is. It will simply raise if a different
        action is passed in.

        :param request: an instance of PollyRequest
        :return: whatever returned by the delegate
        """
        actions = {
            'SynthesizeSpeech': self._synthesize_speech_and_save
            # ... more actions could go in here ...
        }

        if request.polly_action not in actions:
            raise RuntimeError('bad or unsupported Amazon Polly action: "' + request.polly_action + '".')

        return actions[request.polly_action](request)

    def _node_request_handler(self, request):
        """The callback function for processing service request.

        It never raises. If anything unexpected happens, it will return a PollyResponse with details of the exception.

        :param request: an instance of PollyRequest
        :return: a PollyResponse
        """
        rospy.loginfo('Amazon Polly Request: {}'.format(request))

        try:
            response = self._dispatch(request)
            rospy.loginfo('will return {}'.format(response))
            return PollyResponse(result=response)
        except Exception as e:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            exc_type = sys.exc_info()[0]

            # not using `issubclass(exc_type, ConnectionError)` for the condition below because some versions
            # of urllib3 raises exception when doing `from requests.exceptions import ConnectionError`
            error_ogg_filename = 'connerror.ogg' if 'ConnectionError' in exc_type.__name__ else 'error.ogg'

            error_details = {
                'Audio File': os.path.join(current_dir, 'data', error_ogg_filename),
                'Audio Type': 'ogg',
                'Exception': {
                    'Type': str(exc_type),
                    'Module': exc_type.__module__,
                    'Name': exc_type.__name__,
                    'Value': str(e),
                },
                'Traceback': traceback.format_exc()
            }

            error_str = json.dumps(error_details)
            rospy.logerr(error_str)
            return PollyResponse(result=error_str)

    def synthesize(self, **kws):
        """Call this method if you want to use polly but don't want to start a node.

        :param kws: input as defined in Polly.srv
        :return: a string in JSON form with detailed information, success or failure
        """
        req = PollyRequest(polly_action='SynthesizeSpeech', **kws)
        return self._node_request_handler(req)

    def start(self, node_name='polly_node', service_name='polly'):
        """The entry point of a ROS service node.

        Details of the service API can be found in Polly.srv.

        :param node_name: name of ROS node
        :param service_name:  name of ROS service
        :return: it doesn't return
        """
        rospy.init_node(node_name)

        service = rospy.Service(service_name, Polly, self._node_request_handler)

        rospy.loginfo('polly running: {}'.format(service.uri))

        rospy.spin()


def main():
    usage = '''usage: %prog [options]
    '''

    parser = OptionParser(usage)

    parser.add_option("-n", "--node-name", dest="node_name", default='polly_node',
                      help="name of the ROS node",
                      metavar="NODE_NAME")
    parser.add_option("-s", "--service-name", dest="service_name", default='polly',
                      help="name of the ROS service",
                      metavar="SERVICE_NAME")

    (options, args) = parser.parse_args()

    node_name = options.node_name
    service_name = options.service_name

    AmazonPolly().start(node_name=node_name, service_name=service_name)


if __name__ == "__main__":
    main()
