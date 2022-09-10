Get helper scripts

```
cd ~/Downloads
git clone https://github.com/tynguyen/arm64v8_dockers.git
```

Install Docker CE
```
cd ~/Downloads/arm64v8_dockers
bash install_docker_ce.sh
```
To install qemu tools
``` 
sudo apt install binfmt-support
sudo apt install qemu-user-static android-tools-adb android-tools-fastboot
```
---

## Get the arm64v8/noetic-bionic docker

```
docker pull kumarrobotics/voxl:arm64v8-noetic_bionic_mrsl
```

## (Alternate) Build arm64v8/noetic-bionic on an x86 machine
Make sure the you've installed the prerequisites

Following registers new handlers for ELF binaries built for alternative architectures 
```
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

bash build_docker_image.sh
```

This will take a while. Grab a coffee!

---

## Setup common package folder (both for indigo/noetic)
```
mkdir -p ~/voxl_home/common_pkgs
cd ~/voxl_home/common_pkgs

git clone https://github.com/tdinesh/apriltag.git
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/KumarRobotics/jps3d.git
git clone https://github.com/ATLFlight/snap_msgs.git

git clone https://github.com/tdinesh/qflight_descriptions.git -b devel_kr
git clone https://github.com/tdinesh/snavquad_interface.git -b devel_cleanup
git clone https://github.com/tdinesh/snav_replanning.git -b devel_noetic

git clone https://github.com/KumarRobotics/kr_mav_control.git
git clone https://github.com/KumarRobotics/multi_mav_manager.git

cd ~/voxl_home/common_pkgs/apriltag
git submodule update --init

cd ~/voxl_home/common_pkgs/kr_mav_control
git submodule update --init

cd ~/voxl_home/common_pkgs/multi_mav_manager
git submodule update --init
```

## Setup noetic workspace
```
mkdir -p ~/voxl_home/ws_noetic/src
cd ~/voxl_home/ws_noetic/src
```

### Create sym-link to common pkgs (outside docker)
```
cd ~/voxl_home/ws_noetic/src
ln -s ../../common_pkgs common_pkgs
```

### Get noetic packages

git clone https://github.com/tdinesh/ewok.git -b devel_replan_noetic


---

## Run the arm64v8 noetic docker

```
sudo docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

#change the folder location for voxl_home to your own shared folder

```
sudo docker run -it --privileged \
  --net=host --name voxl_noetic_docker \
  -v /dev/ptmx:/opt/ptmx \
  -v /home/voxl_home:/root/voxl_home:rw \
  -w /root/ \
  kumarrobotics/voxl:arm64v8-noetic_focal_mrsl \
  /bin/bash

sudo docker start voxl_noetic_docker
sudo docker exec -it voxl_noetic_docker /bin/bash
```

Compile the noetic workspace in the docker

```
catkin config -DCMAKE_BUILD_TYPE=Release
catkin config --install
```

---

## Setup indigo workspace
```
mkdir -p ~/voxl_home/ws_indigo/src
cd ~/voxl_home/ws_indigo/src
```

### Create sym-link to pkgs

```
cd ~/voxl_home/ws_indigo/src
ln -s ../../common_pkgs/catkin_simple catkin_simple
ln -s ../../common_pkgs/kr_mav_control kr_mav_control
ln -s ../../common_pkgs/qflight_descriptions qflight_descriptions
ln -s ../../common_pkgs/snavquad_interface  snavquad_interface
```

### Get indigo pacakges
```

git clone https://github.com/tdinesh/quadrotor_ukf.git -b devel_tf_imu

git clone https://github.com/ros/roslint.git
git clone https://github.com/ros-perception/image_transport_plugins.git -b indigo-devel
git clone https://github.com/ros-perception/image_common.git -b hydro-devel
git clone https://github.com/ros/xacro.git -b indigo-devel
git clone https://github.com/ros-perception/vision_opencv.git  -b indigo

git clone https://github.com/ros/geometry2.git -b melodic-devel
touch geometry2/geometry2/CATKIN_IGNORE geometry2/test_tf2/CATKIN_IGNORE geometry2/tf2/CATKIN_IGNORE geometry2/tf2_bullet/CATKIN_IGNORE geometry2/tf2_kdl/CATKIN_IGNORE geometry2/tf2_msgs/CATKIN_IGNORE geometry2/tf2_py/CATKIN_IGNORE geometry2/tf2_ros/CATKIN_IGNORE geometry2/tf2_sensor_msgs/CATKIN_IGNORE geometry2/tf2_tools/CATKIN_IGNORE

git clone https://github.com/ros/geometry.git -b indigo-devel
touch geometry/eigen_conversions/CATKIN_IGNORE geometry/geometry/CATKIN_IGNORE geometry/tf/CATKIN_IGNORE

mkdir -p ~/voxl_home/ws_indigo/src/snap
cd ~/voxl_home/ws_indigo/src/snap
ln -s ~/voxl_home/common_pkgs/snap_msgs snap_msgs

git clone https://github.com/ATLFlight/snap_cpa
git clone https://github.com/ATLFlight/snap_vio
git clone https://github.com/tdinesh/snap_imu.git -b fix/publish_rate
git clone https://gitlab.com/tdineshd/voxl-cam-ros.git -b devel_cinfo_path
git clone https://gitlab.com/tdineshd/voxl-hal3-tof-cam-ros.git -b fix/depth_image

cd ~/voxl_home/ws_indigo/src/snap/snav_ros
git submodule update --init
cd ~/voxl_home/ws_indigo/src/snap/voxl_cam_ros
git submodule update --init

cd ~/voxl_home/ws_indigo/
```

## Download and the voxl-emulator docker
```
cd ~/Downloads
git clone https://gitlab.com/voxl-public/voxl-docker.git
cd voxl-docker

sudo install -m 0755 files/voxl-docker.sh /usr/local/bin/voxl-docker

docker pull kumarrobotics/voxl:voxl-emulator-v1.2-mrsl
```

## Run the voxl-emulator
```
sudo voxl-docker -i kumarrobotics/voxl:voxl-emulator-v1.2-mrsl -d ~/voxl_home
```

Compile the indigo workspace in the emulator. Use `catkin build -c`to ignore errors
```
catkin config -DCMAKE_BUILD_TYPE=Release
catkin config --install
source /opt/ros/indigo/setup.bash
catkin build -c
```

###  Note about building packages in voxl emulater

TODO (need to check compile flags for all ROS pacakges running in indigo). Maybe set via catkin config or CXX_FLAGS ?

It is very important to specify correct fpu flags, otherwise the NEON engine will not be used for floating-point computations, significantly slowing down floating-point math.

`-march=armv7-a -mfloat-abi=softfp -mfpu=neon-vfpv4`

### Removing old melodic docker on voxl (Only needed if noetic docker is not setup)

`sudo docker ps -a`

Get the ContainerID corresponding to arm64v8/melodic:bionic-melodic

```
sudo docker stop ContainerID
sudo docker rm ContainerID
sudo docker rmi arm64v8/melodic:bionic-melodic

cd /mnt/sdcard
sudo docker load --input arm64v8-noetic-focal_voxl.tar.gz

sudo docker run -it --privileged \
  --net=host --name voxl_noetic_docker \
  -v /dev/ptmx:/opt/ptmx \
  -v /data/home_linaro:/root/home_linaro:rw \
  -w /root/ \
  kumarrobotics/voxl:arm64v8-noetic-focal_voxl \
  /bin/bash

sudo docker start voxl_noetic_docker
sudo docker exec -it voxl_noetic_docker /bin/bash
```

### Setting the cameras

Check the configurations from
https://docs.modalai.com/camera-connections/#camera-ports

Then ex Tracking + TOF
`voxl-configure-cameras 5`