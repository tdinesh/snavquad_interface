Flash the base SnapDragon Flight (SDF)
  * Request access to `snapdragon_flight_setup` folder (contains platform BSP, firmware, etc)
  * Apply wall power to vehicle.
  * Connect Micro-USB 2.0 cable to SDF to use as the Android Debug Bridge (ADB).
  * Confirm blue "breathing" LED on SDF.

On your laptop add correct rules to `/etc/udev/rules.d` by creating a file, e.g. `/etc/udev/rules.d/98-flight-pro.rules`, with the following lines:

```
SUBSYSTEM=="usb", ATTR{idVendor}=="05c6", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="18d1", MODE="0666", GROUP="plugdev"
```

Restart udev and install platform BSP
```
sudo udevadm control --reload-rules && udevadm trigger
sudo apt install zip unzip default-jdk android-tools-adb
cd snapdragon_flight_setup
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

Copy `snav_setup` & `snav_debs` folder onto the board
```
adb push snav_setup /home/linaro/snav_setup
adb push snav_debs /home/linaro/snav_debs
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
cd ~/snav_debs
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
```
  * If ssh fails, likely the hostname is not resolving.
  * Only for 18.04 (on laptop)
```
sudo ln -sf /run/systemd/resolve/resolv.conf /etc/resolv.conf
```

Another issue with ssh `ssh Connection closed by port 22`
```
adb shell
ssh-keygen -t dsa -f /etc/ssh/ssh_host_dsa_key
ssh-keygen -t rsa -f /etc/ssh/ssh_host_rsa_key
```

Set the vehicle in performance mode (enables 2 additional cores)
```
sudo chown -R linaro:linaro ~/snav_setup
cd ~/snav_setup
chmod +x setperfmode.sh
sudo ./setperfmode.sh
```
  * ignore "Invalid argument" write error

Install machine vision, snav, snav-esc
```
cd ~/snav_debs
sudo dpkg -i mv_1.1.9_8x74.deb
sudo dpkg -i snav_1.2.59_8x74.deb
sudo dpkg -i snav-esc_1.2.0.deb
```

Install Qualcomm license
```
cd ~/snav_setup
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
sudo cp snav_params_ddk.xml /usr/share/data/adsp/snav_params.xml
```
For dragonfly (230) vehicle
```
sudo cp snav_params_230.xml /usr/share/data/adsp/snav_params.xml
```
For tiercel vehicle
```
sudo cp snav_params_tiercel.xml /usr/share/data/adsp/snav_params.xml
```

Following enables Snapdragon VIO app with downward facing camera. Without the `downward` flag, downward camera mounted with 45 deg tilt is used.
```
cd /etc/snav
sudo ./configure_vio.sh -c downward
```

```
sudo stop snav
sudo start snav
sudo snav_calibration_manager -s
```
* Above command does a static IMU calibration
* Update vehicle specific parameters...vehicle length dx, dy, basethrust in /usr/share/data/adsp/snav_params.xml
* Vehicle should be flyable with RC at this point

If needed perform dynamic calibration with props mounted.
```
sudo snav_calibration_manager -d
```
 * Warning!! Vehicle will automatically takeoff and land after few mins. Make sure you have copied the correct `snav_params` files.

Disable snav apps (`snav_dft`, `snav_dft_vio`, `snav_dft_vio_apriltag`, `snav_voa`). This allows external ROS interfaces we use to have acess to the camera/imu. (Note, dynamic calibration won't work after these apps are disabled)
```
cd /etc/snav
sudo ./disable_apps.sh
```

Update package listing
  * This step assumes the vehicle has an internet connection!!!
```
sudo apt-get update
```
  * If you get a NO_PUBKEY error with a alphanumeric key, run:
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys <key>
```

Install ROS
  * This step assumes the vehicle has an internet connection!!!
```
cd ~/snav_setup
chmod +x ros_setup_snap.sh
./ros_setup_snap.sh $1
```

Copy config files
```
ssh linaro@dragonfly$1
cd ~/snav_setup
cp _tmux1.8.conf ~/.tmux.conf
cp _bash_aliases ~/.bash_aliases
cp _vimrc ~/.vimrc
```
  * basic installation complete

Create ros workspace (Make sure you are not `sudo`)
```
source ~/.bashrc
mkdir -p ~/ws_ros/src
cd ~/ws_ros
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release -j3
catkin build
echo "source /home/linaro/ws_ros/devel/setup.bash" >> /home/linaro/.bashrc
```

Clone ATLFlight repos
```
mkdir -p ~/ws_ros/src/snap
cd ~/ws_ros/src/snap
git clone https://github.com/ATLFlight/snap_msgs
git clone https://github.com/ATLFlight/snav_msgs
git clone https://github.com/ATLFlight/snap_vio
git clone https://github.com/ATLFlight/snap_cpa
git clone https://github.com/ATLFlight/qflight_descriptions
git clone https://github.com/ATLFlight/dfs-ros-example
git clone https://github.com/ATLFlight/snav_ros
git clone https://github.com/ATLFlight/snav_fci
git clone https://github.com/ATLFlight/snap_cam_ros
git clone -b fix/publish_rate https://github.com/tdinesh/snap_imu
```

```
cd ~/ws_ros/src/snap/snap_cam_ros
git submodule update --init
cd ~/ws_ros/src/snap/snav_ros
git submodule update --init
```

Clone opencv/image_transport repos
  * Clone and select correct branch (currently Indigo if available).
```
cd ~/ws_ros/src
git clone -b indigo https://github.com/ros-perception/vision_opencv.git
git clone -b indigo-devel https://github.com/ros-perception/image_transport_plugins.git
git clone -b hydro-devel https://github.com/ros-perception/image_common.git
```

Clone KumarRobotics repos
```
git clone https://github.com/KumarRobotics/quadrotor_control.git
cd ~/ws_ros/src/quadrotor_control
git submodule update --init
```

Clone `vio_qc` packages. If you dont have git access skip to next step
```
cd ~/ws_ros/src
git clone -b version_12 https://github.com/loiannog/vio_qc.git
```

Copy ukf_packages.zip into `~/ws_ros/src` folder and unzip.
  * For example, from your laptop copy over the file.
```
scp ukf_packages.zip linaro@dragonfly$1:~/ws_ros/src/
```
  * Then from the device unzip and clean up.
```
unzip ukf_packages.zip
rm ukf_packages.zip
```

Clone launch files repo
```
cd ~/ws_ros/src
git clone https://github.com/tdinesh/snavquad_interface.git
```

Assign swap space.
 * If you compiling large templated libraries, sometimes the board will run out of memory.
 * Use following command to assign swap space to be used as virtual memory

```
sudo fallocate -l 1G /mnt/1GB.swap
sudo chmod 600 /mnt/1GB.swap
sudo mkswap /mnt/1GB.swap
sudo swapon /mnt/1GB.swap
# TO Make swap permanent
sudo cp /etc/fstab /etc/fstab.bak
echo '/mnt/1GB.swap none swap sw 0 0' | sudo tee -a /etc/fstab
```

Compile ROS Packages
  * Run a fan while compiling (onboard or offboard) to avoid damaging the board.
```
cd ~/ws_ros
catkin build -c
```
 * Grab a cup of coffee; this will take about 40 minutes.

Setup Camera IDs

```
echo "export HIRES_CAM_ID=0" >> ~/.bashrc
echo "export TRACKING_CAM_ID=1" >> ~/.bashrc
echo "export STEREO_CAM_ID=3" >> ~/.bashrc
```

Setup MAV properties type, mass and board_type

 * MAV_TYPE can be `230`, `ddk` or `tiercel` based on the frame.
 * Set `MAV_MASS=0.24` for dragonfly(230) platform, `MAV_MASS=0.45` for ddk platform, `MAV_MASS=0.191` for Tiercel.
 * Tray type can be `sdf_tray`, `sdf_tray_45`, `sdf_pro`

For `dragonfly`

```
echo "export MAV_TYPE=230" >> ~/.bashrc
echo "export MAV_MASS=0.245" >> ~/.bashrc
echo "export MAV_BOARD=sdf_tray" >> ~/.bashrc
echo "export IMU_1_USED=false" >> ~/.bashrc
```

For `ddk`

```
echo "export MAV_TYPE=ddk" >> ~/.bashrc
echo "export MAV_MASS=0.45" >> ~/.bashrc
echo "export MAV_BOARD=sdf_tray" >> ~/.bashrc
echo "export IMU_1_USED=false" >> ~/.bashrc
```

For `Tiercel`

```
echo "export MAV_TYPE=tiercel" >> ~/.bashrc
echo "export MAV_MASS=0.191" >> ~/.bashrc
echo "export MAV_BOARD=sdf_tray" >> ~/.bashrc
echo "export IMU_1_USED=false" >> ~/.bashrc
```

Internal `VIO` and other apps have to be disabled to allow access to camera/imu.
```
cd /etc/snav
sudo ./disable_apps.sh
```

Always stop/start Snapdragon Navigator `snav` before starting a new flight. There is a bash alias `restart_snav` that combines the two commands. This command also attempts to start any other processes that are configured to autostart.
```
restart_snav
```

Use helper tmux scripts to launch necessary launch files. Create symbolic links in home folder
```
cd ~/
ln -s ws_ros/src/snavquad_interface/scripts/tmux_snav.sh tmux_snav.sh
```
