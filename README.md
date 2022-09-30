# Snapdragon Flight Documentation/Interface

These pages provide some information about the [VOXL](https://www.modalai.com/pages/development-kits) platform including how to setup the build environment, interface with [kr_mav_control](https://github.com/KumarRobotics/kr_mav_control) and more.

User Guides:
* [Tools & Resources](https://docs.modalai.com/voxl/)
* Common FAQs are addressed in the [forums](https://forum.modalai.com/).

Our board setup is slightly different than the [official documentation](https://docs.modalai.com/). Following instructions have been pooled together from above guides, forums, etc.

We have three hardware platforms available in the lab for use.
* [Dragon Drone Development Kit](https://worldsway.com/product/dragon-drone-development-kit/) DDK. [Assembly Instructions](https://worldsway.com/wp-content/uploads/2017/08/DragonDDK-End-User-Assembly-Instructions_V3.pdf).
* Dragonfly/230
* Starling

## Board setup

[Snapdragon Flight (deprecated old board)](doc/SnapFlightSetup.md)

[VOXL Board](doc/VOXLSetup.md)

## Setup packages on Laptop

[Setup ROS](doc/ROSSetup.md)

## Flying with the VOXL board and 230/DDK (platform).
```
ssh root@dragonflyX
```
  * Here X is the ID of the MAV

This automatically restarts snav, launches necessary nodes.
```
./tmux_voxl.sh
```
  * Switch to the `Kill` tab in the tmux window and enter the session name `tmux_snav`, press Enter to close everything

There are helper scripts for ground station computer that sets up `ROS_MASTER_URI` and launches necessary nodes. This assumes `kr_mav_control`  and `snavquad_interface` is compiled in your laptop workspace. For example `dragonfly4` platform Enter the vehicle number accordingly.

```
roscd snavquad_interface/scripts
./tmux_ground_station 4
```

There are helper scripts for recording bag file in the `snavquad_interface/scripts/capture` folder. On the mav tmux session, switch to the `Aux` tab, there should be a pane that already has the following keys. Just hit `Enter` to start recording the bag. If the mav has micro-SD card it will bag it to `/media/sdcard`, else to the home folder `~/`.
```
roscd snavquad_interface/scripts/capture
./record/sh 4
```

## Calibrating the cameras.
[Calibrate](doc/CameraCalib.md)

## Calibrating the IMU
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
