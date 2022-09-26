## ROS setup on Laptop

### Install  ROS Noetic:
  http://wiki.ros.org/noetic/Installation/Ubuntu

### Install dependencies:
```
sudo apt install git python3-catkin-tools python3-osrf-pycommon emacs vim tmux tmuxinator python-is-python3

sudo apt install python3-matplotlib python3-numpy libeigen3-dev libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev protobuf-compiler libnlopt-dev libnlopt-cxx-dev libyaml-cpp-dev

sudo apt install ros-${ROS_DISTRO}-octomap-server ros-${ROS_DISTRO}-octomap-ros ros-${ROS_DISTRO}-sophus ros-${ROS_DISTRO}-swri-roscpp ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-camera-calibration-parsers ros-${ROS_DISTRO}-tf2-geometry-msgs ros-${ROS_DISTRO}-random-numbers ros-${ROS_DISTRO}-image-geometry ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-rviz ros-${ROS_DISTRO}-laser-assembler ros-${ROS_DISTRO}-image-proc ros-${ROS_DISTRO}-depth-image-proc

sudo apt install ros-${ROS_DISTRO}-vision-msgs
```

### Install Gazebo:

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt update
sudo apt install gazebo11
sudo apt install ros-${ROS_DISTRO}-gazebo-ros ros-${ROS_DISTRO}-gazebo-plugins

```

### Clone repositories: (Use vcstoll to import all necessary packages)
```
sudo apt install python3-vcstool
pip3 install gdown

mkdir -p ~/ws_kr/src
cd ~/ws_kr/src

git clone https://github.com/KumarRobotics/waypoint_navigation_plugin.git
git clone https://github.com/KumarRobotics/mrsl_quadrotor.git
git clone https://github.com/KumarRobotics/multi_mav_manager.git
git clone https://github.com/KumarRobotics/kr_mav_control.git

git clone https://github.com/ATLFlight/snap_msgs.git
git clone https://github.com/tdinesh/qflight_descriptions.git -b devel_kr
git clone https://github.com/tdinesh/snavquad_interface.git -b devel
```

### Compile:
```
  cd ~/ws_kr/src/kr_mav_control
  git submodule update --init --recursive

  cd ~/ws_kr/src/multi_mav_manager
  git submodule update --init --recursive

  cd ~/ws_kr
  catkin init
  catkin config -DCMAKE_BUILD_TYPE=Release
  catkin build -c
```

### Add workspace to bashrc:

```
echo "source ~/ws_kr/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```