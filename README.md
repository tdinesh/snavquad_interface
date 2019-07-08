# snavquad_interface

[ATLFlight README](https://github.com/ATLFlight/ATLFlightDocs/blob/master/README.md)

[Get Latest Platform BSP](https://support.intrinsyc.com/projects/snapdragon-flight/wiki/Get_and_install_the_latest_platform_BSP)

```
unzip Flight_3.1.3.1_JFlash.zip
cd Flight_3.1.3_JFlash
./jflash.sh
```

Update DSP firmware
```
unzip Flight_3.1.3.1_qcom_flight_controller_hexagon_sdk_add_on.zip
cd Flight_3.1.3.1_qcom_flight_controller_hexagon_sdk_add_on
./installfcaddon.sh
```

Change the hostname based on vehicle id
```
adb shell
sudo hostname dragon65
vi /etc/hostname
```

Copy snav setup folder onto the board
```
adb push snav_setup /home/linaro/snav_setup

```

Modify /etc/network/interfaces and /etc/wpa_supplicant/wpa_supplicant.conf to connect to WiFi network
```
cd ~/snav_setup
sudo cp interfaces /etc/network/interfaces
sudo cp wpa_supplicant.conf /etc/wpa_supplicant/wpa_supplicant.conf
```

[Put the board in station mode](https://docs.px4.io/en/flight_controller/snapdragon_flight_advanced.html#wifi-settings)
```
/usr/local/qr-linux/wificonfig.sh -s station
echo "wireless-power off" >> /etc/network/interfaces.d/.qca6234.cfg.station
```

Reboot and try ssh 
```
ssh linaro@dragon65
```

Update dpkg
```
cd ~/
wget http://launchpadlibrarian.net/359699571/dpkg_1.17.5ubuntu5.8_armhf.deb
sudo dpkg -i dpkg_1.17.5ubuntu5.8_armhf.deb
```

[ROS Installation](https://github.com/ATLFlight/ATLFlightDocs/blob/master/SnapdragonROSInstallation.md)
```
sudo apt-get install -f
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
```

If you get a NO_PUBKEY error with a alphanumeric key, run:

```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys <key>
```

ros-indigo-opencv3 has a conflict
```
sudo apt purge --auto-remove libxcb1 libfreetype6
sudo apt-get install -y libcairo-gobject2 libcairo2 libxft2 libfontconfig1 libharfbuzz0b libfreetype6 libxcursor1 libxdamage1 libxcb-render0 libxcb-shm0 libxfixes3 libxcomposite1 libxrandr2 xauth libxext6:armhf libxmuu1:armhf  libxrender1 libx11-6:armhf libxcb1 libgtkglextmm-x11-1.2-0 libgtk-3-0
```

```
sudo apt-get install ros-indigo-opencv3 ros-indigo-ros-base
sudo apt-get -f -o Dpkg::Options::="--force-overwrite" install
cd /home
sudo chown -R linaro:linaro linaro
cd ~/
echo "export HOME=/home/linaro/" >> /home/linaro/.bashrc
echo "source /opt/ros/indigo/setup.bash" >> /home/linaro/.bashrc
sudo rosdep init
rosdep update
sudo dpkg -r libhwloc-plugins
sudo dpkg -r ocl-icd-libopencl1:armhf
sudo apt-get install ros-indigo-tf2-ros ros-indigo-tf2-geometry-msgs ros-indigo-geometry python-catkin-tools ros-indigo-camera-info-manager
mkdir -p ws_ros/src
cd ws_ros
catkin init
catkin config -j1 -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-std=c++11 -DCMAKE_C_FLAGS=-std=gnu99
```

Setup Machine Vision SDK and Snapdragon Navigator SDK
```
cd ~/snav_setup
sudo dpkg -i mv_1.1.8_8x74.deb
sudo dpkg -i snav_1.2.58_8x74.deb
sudo dpkg -i snav-esc_1.2.0.deb
sudo cp snapdragon-flight-license.bin /usr/lib
sudo mkdir -p /opt/qcom-licenses/
sudo cp snapdragon-flight-license.bin /opt/qcom-licenses/
sudo cp snav_params.xml /usr/share/data/adsp/
```

If ESC Update is needed `force_firmware_update` and `force_config_update` have to be set in /usr/share/data/adsp/snav_params.xml and rebooted. On reboot boards updates ESC
Reset params to 0 to disable update on every boot

```
cd /etc/snav
sudo ./configure_vio.sh
```

Static and dynamic calibration (Vehicle will takeoff!!!)
```
sudo stop snav
sudo start snav
sudo snav_calibration_manager -s
sudo snav_calibration_manager -d
```

To check vio and other status
```
snav_inspector
```

Disable apps
```
cd /etc/snav
sudo ./disable_apps.sh
```
Reboot and proceed

Setup other snav ros packages
```
cd ~/ws_ros/src
mkdir snav
cd snav
git clone https://github.com/ATLFlight/snav_ros
git clone https://github.com/ATLFlight/snav_msgs
git clone https://github.com/ATLFlight/snap_vio
git clone https://github.com/ATLFlight/snap_imu
git clone https://github.com/ATLFlight/snap_cpa
git clone https://github.com/ATLFlight/snap_cam_ros
git clone https://github.com/ATLFlight/qflight_descriptions
git clone https://github.com/ATLFlight/snap_msgs
git clone https://github.com/ATLFlight/dfs-ros-example
cd ~/ws_ros/src/snav/snap_cam_ros
git submodule init
git submodule update
cd ~/ws_ros/src/snav/snav_ros
git submodule init
git submodule update
```


```
sudo apt-get install libeigen3-dev htop
cd ~/ws_ros/src
git clone https://github.com/ros-perception/image_common
cd ~/ws_ros/src/image_common/
touch CATKIN_IGNORE
cd ~/ws_ros/src/
git clone https://github.com/ros-perception/vision_opencv.git
git clone https://github.com/ros-perception/image_transport_plugins.git
cd ~/ws_ros/src/vision_opencv
git checkout origin/indigo
cd ~/ws_ros/src
git clone https://github.com/tdinesh/apriltag.git
git checkout -t origin/devel_snav

```

OpenCV3 deb file (Not needed) https://github.com/PX4/snap_cam/blob/master/README.md
```
cd ~/Downloads
wget http://px4-tools.s3.amazonaws.com/opencv3_20160222-1_armhf.deb
```

```
roslaunch ddk_launch snav_interface.launch
```
