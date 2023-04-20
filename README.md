
![](img/logo14.png)

# Rofunc-ros: A Ros Package for Human-centered Intelligent Interactive Humanoid Robots

- [Rofunc-ros: A Ros Package for Human-centered Intelligent Interactive Humanoid Robots](#rofunc-ros-a-ros-package-for-human-centered-intelligent-interactive-humanoid-robots)
  - [Installation](#installation)
    - [System requirements](#system-requirements)
    - [Installation](#installation-1)
  - [Functions](#functions)
    - [Voice Q\&A](#voice-qa)
    - [OptiLocation](#optilocation)
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

For this function, we implemented a Voice Q&A robot based on `ChatGPT`. This function consists of `Speech2text`, `Chat` and `Text2speech`. 

![](img/voiceQA_pipeline.png)

This whole pipeline can be activated by calling

**English**

```
roslaunch rofunc_ros voice_qa_en.launch
```

**Chinese**

```
roslaunch rofunc_ros voice_qa_cn.launch
```

### OptiLocation

```
roslaunch rofunc_ros mocap.launch & rosrun rviz rviz
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
