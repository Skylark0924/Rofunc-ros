# Installation manual

> **Note**
> Ubuntu 20.04 is highly recommended, since the initial version of python3 is **python 3.8** (which is necessary for this package).  

- [Installation manual](#installation-manual)
  - [Install Ros Neotic](#install-ros-neotic)
  - [Install Rofunc-ros](#install-rofunc-ros)
    - [Download the package](#download-the-package)
    - [Install requirements](#install-requirements)
    - [Compile](#compile)
  - [Configure APIs](#configure-apis)
    - [OpenAI API](#openai-api)
    - [AWS API](#aws-api)


## Install Ros Neotic

Please follow the [official documentation](http://wiki.ros.org/noetic/Installation/Ubuntu) for Ros Neotic, and also [configure your ros environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).


## Install Rofunc-ros

### Download the package

```
cd ~/catkin_ws/src
git clone https://github.com/Skylark0924/Rofunc-ros.git
```

### Install requirements

```
cd ~/catkin_ws/src/Rofunc-ros/scripts
./install_deps.sh
```

### Compile

```
cd ~/catkin_ws/
catkin_make
```

## Configure APIs

### OpenAI API

1. Copy your OpenAI API from this [website](https://platform.openai.com/account/api-keys)
2. Paste it to `chat/config/api_key.yaml`.

### AWS API

This API is used for text2speech

1. Register the AWS account
2. Create default profiles by following this [documentation](https://docs.aws.amazon.com/cli/latest/userguide/cli-configure-files.html)
   `~/.aws/credentials`
   ```
   [default] 
   aws_access_key_id=AKIAIOSFODNN7EXAMPLE
   aws_secret_access_key=wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY
   ```

   `~/.aws/config`
   ```
   [default]
   region=us-west-2
   output=json
   ```
3. Check your AWS configuration
   ```
   $ aws configure
    AWS Access Key ID [None]: AKIAIOSFODNN7EXAMPLE
    AWS Secret Access Key [None]: wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY
    Default region name [None]: us-west-2
    Default output format [None]: json
   ```
4. Add IAM role permissions for your AWS account
   1. Search `Role` in your account console and enter the `Identity and Access Management (IAM)` page
   2. Find the `Role` in `Access management`
   3. Click `Create role`
   4. Click `AWS account`
   5. Search and add `AmazonPollyFullAccess`
   6. Add a name and then click `Create role`