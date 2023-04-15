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
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


# ROS PACKAGING
# using distutils : https://docs.python.org/2/distutils
# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        'tts',
    ],
    package_dir={
        '': 'VoiceQA/text2speech/src',
    },
    package_data={
        '': ['data/*.ogg', 'data/models/polly/2016-06-10/*.json']
    },
)
setup(**setup_args)
