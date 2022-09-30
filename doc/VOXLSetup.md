# VOXL board setup

[ModalAI Technichal Docs](https://docs.modalai.com/)



### [Flash the board](https://docs.modalai.com/flash-system-image/)

### Setup ADB
  <a href="https://docs.modalai.com/setup-adb/" target="_blank">Opens in new tab</a>

### Get the system image `modalai-2-5-2-1.0.1-b.tar.gz`
```
tar -xvf modalai-2-5-2-1.0.1-b.tar.gz
cd modalai-2-5-2-1.0.1-b
./install.sh
```
  * When asked
        "Do you want this installer to fetch the latest packages, and automatically install them? (y/n)"
    Type n (do not install voxl-suite)

### Install voxl utils and other ipk

```
tar -xvf voxl_working_ipk.tar.gz
cd voxl_working_ipk
./install.sh --adb
```

### Push precompiled ipks to the board and setup files folder to the board
```
tar -xvf snav_setup.tar.gz
tar -xvf ipk_emulator_v1.1.tar.gz
adb push ipk_emulator_v1.1 /home/root
adb push snav_setup /home/root
```

### Change hostname based of vehicle id `$1`
```
adb shell
sudo hostname dragonfly$1
vi /etc/hostname
```
  * replace `apq8096` with: `dragonfly$1`
  * `:wq + Enter` to save and exit vi

### Connect Wifi to AP
```
voxl-wifi station <SSID> <Password>
exit
```

### Reboot adb device and try ssh into the board `$1` assumed to be 30
```
adb reboot && adb wait-for-device
ssh root@dragonfly30
```
  * password `oelinux123`
  * If ssh fails, likely the hostname is not resolving.
    `sudo ln -sf /run/systemd/resolve/resolv.conf /etc/resolv.conf`


### Installing snav and other ipks
```
cd ~/ipk_emulator_v1.1
opkg remove voxl-suite
opkg remove voxl-vision-px4
opkg remove modalai-vl
opkg install snav-modalai_1.4.1_8x96.ipk
opkg install Python_3.6.9_8x96.ipk
opkg install libevent_2.1.11_8x96.ipk
opkg install tmux_2.1_8x96.ipk
opkg install opencv_3.4.6_8x96.ipk
opkg install rsync_3.1.2_8x96.ipk
pip3 install pyyaml rospkg
```

### [Setting cameras](https://docs.modalai.com/configure-cameras/)

Based on the attached camera configurations, choose the correct one
https://docs.modalai.com/voxl-camera-config/

For only the tracking camera use the following
`voxl-configure-cameras 3`

This updates the following files
 * /etc/snav/camera.downward.xml
 * /etc/snav/camera.stereo.xml
 * /etc/modalai/camera_env.sh

### Copy config files
```
cd ~/snav_setup
cp _bashrc_voxl ~/.bashrc
cp _tmux2.1.conf ~/.tmux.conf
cp _bash_aliases ~/.bash_aliases
cp _vimrc ~/.vimrc
```

```
cp snav_params_voxl_230_2s_45deg_MT1306-8.xml /usr/lib/rfsa/adsp/

ln -sf /usr/lib/rfsa/adsp/snav_params_voxl_230_2s_45deg_MT1306-8.xml /usr/lib/rfsa/adsp/snav_params.xml
```
```
systemctl stop snav_vio_app
systemctl stop snav_voa_app
systemctl stop snav_vio_apriltag_app
systemctl stop snav_dronecontroller

ln -sf /usr/lib/rfsa/adsp/snav_params_voxl_230_2s_45deg_MT1306-8.xml /usr/lib/rfsa/adsp/snav_params.xml
ln -sf /etc/snav/mount.downward_voxl_tray.xml /etc/snav/mount.snav_dft_vio_app.xml
ln -sf /etc/snav/mount.downward_voxl_tray.xml /etc/snav/mount.snav_dft_vio_apriltag_app.xml
ln -sf /etc/snav/mount.stereo.thirteen_deg_down.xml /etc/snav/mount.stereo.xml

```

### Set environment variable based on vehicle ID, example 30
```
export MAV_ID=30
echo "export MAV_ID=$MAV_ID" >> ~/.bashrc
echo "export MAV_NAME=dragonfly$MAV_ID" >> ~/.bashrc
echo "export MAV_TYPE=230" >> ~/.bashrc
echo "export MAV_MASS=0.245" >> ~/.bashrc
echo "export MAV_BOARD=sdf_pro_imu1" >> ~/.bashrc
echo "export IMU_1_USED=true" >> ~/.bashrc
echo "export XBEE_BUS=12" >> ~/.bashrc
echo "export USE_LOG_CAMERA_HEIGHT=false" >> ~/.bashrc
echo "export LOG_CAMERA_HEIGHT=-3.912" >> ~/.bashrc
echo "export USE_VIO_MASK=false" >> ~/.bashrc
echo "export TOF_PITCH=-1.570796" >> ~/.bashrc
```

### Calibrate IMU
  * Restart the MAV before IMU calbiration
  * Below command does a static IMU calibration
  * Without IMU calibration the leds will be blinking yellow
  * After calibration the leds will blink blue

```
systemctl stop snav
systemctl start snav
snav_calibration_manager -s

systemctl restart snav
```

```
cd ~/
mkdir -p /data/home_linaro
ln -s /data/home_linaro/ws_indigo/install/share/snavquad_interface/scripts/tmux_voxl.sh tmux_voxl.sh
ln -s ~/ws_ros/install/share/snavquad_interface/scripts/capture/record.sh
```

### Create bashrc for docker, Set environment variable based on vehicle ID, example 30
```
export MAV_ID=30

echo "" > /data/home_linaro/.bashrc_voxl

echo "source ~/home_linaro/ws_noetic/install/setup.bash" >> /data/home_linaro/.bashrc_voxl
echo "export MAV_ID=$MAV_ID" >> /data/home_linaro/.bashrc_voxl
echo "export MAV_NAME=dragonfly$MAV_ID" >> /data/home_linaro/.bashrc_voxl
echo "export MAV_TYPE=230" >> /data/home_linaro/.bashrc_voxl
echo "export MAV_MASS=0.245" >> /data/home_linaro/.bashrc_voxl
echo "export MAV_BOARD=sdf_pro_imu1" >> /data/home_linaro/.bashrc_voxl
echo "export IMU_1_USED=true" >> /data/home_linaro/.bashrc_voxl
echo "export XBEE_BUS=12" >> /data/home_linaro/.bashrc_voxl
echo "export USE_LOG_CAMERA_HEIGHT=false" >> /data/home_linaro/.bashrc_voxl
echo "export LOG_CAMERA_HEIGHT=-3.912" >> /data/home_linaro/.bashrc_voxl
echo "export USE_VIO_MASK=false" >> /data/home_linaro/.bashrc_voxl
echo "export TOF_PITCH=-1.570796" >> /data/home_linaro/.bashrc_voxl
```

### Configure docker and Load Noetic arm64v8-noetic-focal_voxl.tar.gz docker from SD card
  * Docker load will take roughly 10mins

```
voxl-configure-docker-support.sh

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

### Setup docker on laptop and compile packages
[Docker setup on computer](README_arm64v8_docker.md)


### Sync the ROS packages and test the robot 30
  * Assumes voxl indigo and noetic workspaces are compiled under ~/voxl_home
```
roscd snavquad_interface/scripts
./sync_install.sh 30
```
