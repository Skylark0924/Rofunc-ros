
![](img/logo14.png)

# Rofunc-ros: A Ros Package for Human-centered Intelligent Interactive Humanoid Robots

- [Rofunc-ros: A Ros Package for Human-centered Intelligent Interactive Humanoid Robots](#rofunc-ros-a-ros-package-for-human-centered-intelligent-interactive-humanoid-robots)
  - [Installation](#installation)
    - [System requirements](#system-requirements)
    - [Installation](#installation-1)
  - [Functions](#functions)
    - [Voice Q\&A](#voice-qa)
      - [Speech2text](#speech2text)
      - [Chat](#chat)
      - [Text2speech](#text2speech)
  - [Cite](#cite)
  - [The Team](#the-team)
  - [Related repository: Rofunc](#related-repository-rofunc)
  - [Acknowledge](#acknowledge)


## Installation

### System requirements

The package is only tested by the following configuration.

1. Ubuntu 20.04
2. Ros Neotic

### Installation

**Please refer to the [installation manual](docs/Installation.md).**

## Functions

### Voice Q&A

This function consists of `Speech2text`, `Chat` and `Text2speech`. This pipeline can be activated by calling

English

```
roslaunch rofunc_ros voice_qa_en.launch
```

Chinese

```
roslaunch rofunc_ros voice_qa_cn.launch
```

#### Speech2text

English

```
roslaunch rofunc_ros speech2text_en.launch
```

Chinese

```
roslaunch rofunc_ros speech2text_cn.launch
```

#### Chat

```
roslaunch rofunc_ros start.launch
```

```
rosrun rofunc_ros str_pub.py "Hello"
```

#### Text2speech


```
roslaunch rofunc_ros text2speech.launch
```

```
rosrun rofunc_ros voicer.py 'Hello everyone! I am CURI, a humanoid robot designed by \
      collaborative and versatile robots laboratory. Our lab focuses on the co-evolutionary\
      development of human-centered robotics and AI technologies for advanced robots, such as \
      human-like mobile manipulators, humanoid robots, to perform autonomous, assistive and \
      collaborative tasks by learning and transferring the skills from humans.'
```

## Cite

If you use rofunc-ros in a scientific publication, we would appreciate citations to the following paper:

```
@misc{Rofunc2022,
      author = {Liu, Junjia and Li, Zhihao and Li, Chenzui and Chen, Fei},
      title = {Rofunc: The full process python package for robot learning from demonstration},
      year = {2022},
      publisher = {GitHub},
      journal = {GitHub repository},
      howpublished = {\url{https://github.com/Skylark0924/Rofunc}},
}
```

## The Team
Rofunc-ros is developed and maintained by the [CLOVER Lab (Collaborative and Versatile Robots Laboratory)](https://feichenlab.com/), CUHK.

## Related repository: Rofunc

We also have a python package robot learning from demonstration and robot manipulation (**Rofunc**). 

> **Repository address: https://github.com/Skylark0924/Rofunc**

[![Release](https://img.shields.io/github/v/release/Skylark0924/Rofunc)](https://pypi.org/project/rofunc/)
![License](https://img.shields.io/github/license/Skylark0924/Rofunc?color=blue)
![](https://img.shields.io/github/downloads/skylark0924/Rofunc/total)
[![](https://img.shields.io/github/issues-closed-raw/Skylark0924/Rofunc?color=brightgreen)](https://github.com/Skylark0924/Rofunc/issues?q=is%3Aissue+is%3Aclosed)
[![](https://img.shields.io/github/issues-raw/Skylark0924/Rofunc?color=orange)](https://github.com/Skylark0924/Rofunc/issues?q=is%3Aopen+is%3Aissue)
[![Documentation Status](https://readthedocs.org/projects/rofunc/badge/?version=latest)](https://rofunc.readthedocs.io/en/latest/?badge=latest)
[![Build Status](https://img.shields.io/endpoint.svg?url=https%3A%2F%2Factions-badge.atrox.dev%2FSkylark0924%2FRofunc%2Fbadge%3Fref%3Dmain&style=flat)](https://actions-badge.atrox.dev/Skylark0924/Rofunc/goto?ref=main)
![](img/pipeline.png)



## Acknowledge

We would like to acknowledge the following projects:

1. [wpr_chatgpt](https://github.com/play-with-chatgpt/wpr_chatgpt/)
2. [ros-vosk](https://github.com/alphacep/ros-vosk)
3. [tts-ros1](https://github.com/aws-robotics/tts-ros1)