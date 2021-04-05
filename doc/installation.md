# Installation

It has only one way at this time (build from source in Catkin workspace) to install the packages in this repository.

## Prerequisites

* Ubuntu 18.04.6 LTS
* ROS Melodic Morenia
* (Optional) Nvidia GPU Driver
* realsense D435 driver [link](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)
* pip: `$ sudo apt install python-pip`


## Building from Source

### Retrive the sources

    $ cd $HOME/catkin_ws/src
    $ git clone https://github.com/oorrppp2/zipsa_robot_simulation.git

### Install Dependencies (Intel realsense)
    $ sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
    $ sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
    $ sudo apt-get install librealsense2-dkms
    $ sudo apt-get install librealsense2-utils
    $ sudo apt-get install librealsense2-dev


### Install Dependencies (ROS packages)

    $ cd $HOME/catkin_ws/
    $ wstool init src
    $ wstool merge -t src ./src/zipsa_robot_simulation/doc/living_lab_robot.rosinstall
    $ cd src
    $ wstool update


### Install dependency packages

    $ cd $HOME/catkin_ws/src
    $ rosdep install --from-paths . --ignore-src -r -y
    $ find -name 'requirements.txt' | xargs -L 1 sudo pip install -U -r
    $ sudo apt-get install python3 python-dev python3-dev build-essential libssl-dev libffi-dev libxml2-dev libxslt1-dev libxml2-dev zlib1g-dev libblas-dev libatlas-base-dev ros-melodic-moveit-commander ros-melodic-trac-ik-kinematics-plugin ros-melodic-gazebo* ros-melodic-realsense2-camera* ros-melodic-rgbd-launch -y
    $ find -name 'requirements.txt' | xargs -L 1 sudo pip install -U -r
    
    $ cd $HOME/catkin_ws/src
    $ git clone https://github.com/byeongkyu/gazebo_mimic_joint_plugin.git
    $ rosdep install --from-paths . --ignore-src -r -y
    $ git clone https://github.com/byeongkyu/robotis_gripper.git



### Build

    $ cd $HOME/catkin_ws
    $ catkin build or catkin_make

