# snavquad_interface

Flash the base SnapDragon Flight (SDF)
  * Request access to `snav_setup` folder (contains platform BSP, firmware, etc)
  * Apply wall power to vehicle.
  * Connect Micro-USB 2.0 cable to SDF to use as the Android Debug Bridge (ADB).
  * Confirm blue "breathing" LED on SDF.

On your laptop add correct rules to /etc/udev/rules.d by creating a file, e.g. /etc/udev/rules.d/98-flight-pro.rule, with the following lines:

```
SUBSYSTEM=="usb", ATTR{idVendor}=="05c6", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="18d1", MODE="0666", GROUP="plugdev"
```

Restart udev and install platform BSP
```
udevadm control --reload-rules && udevadm trigger
sudo apt-get install zip unzip default-jdk android-tools-adb
cd snav_setup
unzip Flight_3.1.3.1_JFlash.zip
cd Flight_3.1.3_JFlash
./jflash.sh
cd ..
```
  * After flashing is complete: SDF led will begin pulsing green. Wait until SDF led is pulsing blue before continuing
  * If led is not pulsing blue (likely green), power cycle the vehicle. Disconnect both usb and power. First plug in the power and then the micro usb.

DSP firmware
```
unzip Flight_3.1.3.1_qcom_flight_controller_hexagon_sdk_add_on.zip
cd Flight_3.1.3.1_qcom_flight_controller_hexagon_sdk_add_on
./installfcaddon.sh
```
  * If the above fails, run `./installfcaddon.sh` again

Copy `snav_setup` folder onto the board
```
adb push snav_setup /home/linaro/snav_setup
```

Setup Networking
  * vehicle id is represented by "$1"...replace all "$1" text with actual vehicle id
```
adb shell
export HOME=/home/linaro/
sudo hostname dragonfly$1
vi /etc/hostname
```
  * replace 'linaro-developer' with: dragonfly$1

Update dpkg
```
cd ~/snav_setup
sudo dpkg -i dpkg_1.17.5ubuntu5.8_armhf.deb
```

Steps for using Ubiquiti Bullet router which resolves hostname to IPs
```
cd ~/snav_setup
cp interfaces-mast /etc/network/interfaces
cp wpa_supplicant.conf-mast /etc/wpa_supplicant/wpa_supplicant.conf
/usr/local/qr-linux/wificonfig.sh -s station
echo "wireless-power off" >> /etc/network/interfaces.d/.qca6234.cfg.station
sudo halt
```
  * make sure to unplug both power and adb line
  * some bullet routers are set with mac filtering. ask admin to add the mac address to bullet router settings

Check SSH
  * unpower the vehicle and remove ADB cable
  * repower the vehicle
  * (password: linaro)
```
ssh linaro@dragonfly$1
sudo -s
```
  * If ssh fails, likely the hostname is not resolving.
  * Only for 18.04 (on laptop)
```
sudo ln -sf /run/systemd/resolve/resolv.conf /etc/resolv.conf
```

Set the vehicle in performance mode (enables 2 additional cores)
```
cd ~/snav_setup
chmod +x setperfmode.sh
./setperfmode.sh
```
  * ignore "Invalid argument" write error

Install machine vision, snav, snav-esc
```
sudo dpkg -i mv_1.1.8_8x74.deb
sudo dpkg -i snav_1.2.58_8x74.deb
sudo dpkg -i snav-esc_1.2.0.deb
```

Install Qualcomm license
```
sudo mkdir -p /opt/qcom-licenses/
sudo cp snapdragon-flight-license.bin /opt/qcom-licenses/
```

 * Power the board through ESC (likely needs a battery)
Flash the ESC
```
sudo cp flash_esc_firmware.xml /usr/share/data/adsp/snav_params.xml
sudo stop snav
sudo snav
```
  * Once the ESC flash is complete, the low power warning (fast double beeps) will start. Then type - Ctrl+C

For DragonDDK vehicle
```
sudo cp dragonddk.xml /usr/share/data/adsp/snav_params.xml
```
For dragonfly vehicle
```
sudo cp snav_params.xml /usr/share/data/adsp/snav_params.xml
```

```
sudo stop snav
sudo start snav
sudo snav_calibration_manager -s
```
* Above command does a static IMU calibration
* Update vehicle specific parameters...vehicle length dx, dy, basethrust in /usr/share/data/adsp/snav_params.xml
* Vehicle should be flyable with RC at this point

Disable snav apps
```
cd /etc/snav
./disable_apps.sh
```

Update package listing
  * This step assumes the vehicle has an internet connection!!!
```
sudo apt-get update
```
  * If you get a NO_PUBKEY error with a alphanumeric key, run:
```
sudo apt-key adv * keyserver keyserver.ubuntu.com * recv-keys <key>
```

Install ROS
  * This step assumes the vehicle has an internet connection!!!
```
chmod +x ros_setup_snap.sh
./ros_setup_snap.sh $1
```

Copy config files
```
ssh linaro@dragonfly$1
cd snav_setup
mv _tmux.conf ~/.tmux_conf
mv _bash_aliases ~/.bash_aliases
mv _vimrc ~/.vimrc
```
  * basic installation complete

Create ros workspace
```
mkdir -p ~/ws_ros/src
cd ~/ws_ros
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
echo "source /home/linaro/ws_ros/devel/setup.bash" >> /home/linaro/.bashrc
```

Clone ATLFlight repos
```
cd ~/ws_ros/src
git clone https://github.com/ATLFlight/snav_msgs
git clone https://github.com/ATLFlight/snap_vio
git clone https://github.com/ATLFlight/snap_imu
git clone https://github.com/ATLFlight/snap_cpa
git clone https://github.com/ATLFlight/snap_cam_ros
git clone https://github.com/ATLFlight/qflight_descriptions
git clone https://github.com/ATLFlight/snap_msgs
git clone https://github.com/ATLFlight/dfs-ros-example
git clone https://github.com/ATLFlight/snav_ros
```

```
cd ~/ws_ros/src/snap_cam_ros
git submodule init
git submodule update
cd ~/ws_ros/src/snav_ros
git submodule init
git submodule update
```

Clone opencv/image_transport repos
  * Make sure to get right branch
```
cd ~/ws_ros/src
git clone https://github.com/ros-perception/vision_opencv.git
cd vision_opencv
git checkout origin/indigo
cd ..
git clone https://github.com/ros-perception/image_transport_plugins.git
git clone https://github.com/ros-perception/image_common.git
```

Clone KumarRobotics repos
```
git clone https://github.com/KumarRobotics/quadrotor_control.git
```

Clone `vio_qc` packages. If you dont have git access skip to next step
```
git clone https://github.com/loiannog/vio_qc.git
git checkout version_12
```

Clone/get ukf_packages.zip in `~/ws_ros/src` folder
```
unzip ukf_packages.zip
```

Clone launch files repo
```
cd ~/ws_ros/src
git clone https://github.com/tdinesh/snavquad_interface.git
git checkout devel_fixes
```

Compile ros packages
```
cd ~/ws_ros
catkin build
```
 * Grab a cup of coffee or two. This will take some time

Once compiled run the following launch files in `tmux`
```
roslaunch snavquad_interface state_control.launch
roslaunch snavquad_interface snav_tf_pub.launch
```
 * vehicle ready to fly with rqt_mav_manager. Refer to quadrotor_control for further instructions
