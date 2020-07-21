# VOXL board setup

[ModalAI Technichal Docs](https://docs.modalai.com/)

## [Flash the board](https://docs.modalai.com/flash-system-image/)
### Get the system image `modalai-2-2-0.tar.gz`
```
tar -xvf modalai-2-2-0.tar.gz
cd modalai-2-2-0
python flash_build_app.py
```

## [Install Software Bundles](https://docs.modalai.com/install-software-bundles/)
```
tar -xvf voxl-factory-bundle_0.0.4.tar.gz
tar -xvf voxl-software-bundle_0.0.4.tar.gz

cd voxl-factory-bundle_0.0.4
./install.sh --adb

cd ../voxl-software-bundle_0.0.4
./install.sh --adb
```

## Push precompiled ipks to the board and setup files folder to the board
```
adb push ipk_emulator_v1.1 /home/root`
adb push snav_setup /home/root
```

## Change hostname based of vehicle id `$1`
```
adb shell
sudo hostname dragonfly$1
vi /etc/hostname
```
  * replace `apq8096` with: `dragonfly$1`

## Connect Wifi to AP
```
voxl-wifi station <SSID> <Password>
exit
```

## Reboot adb device and try ssh into the board
```
adb reboot && adb wait-for-device
ssh root@dragonfly30
```
 * password `oelinux123`


## Installing snav and other ipks
```
cd ~/ipk_emulator_v1.1
opkg remove voxl-suite
opkg remove voxl-vision-px4
opkg remove modalai-vl
opkg install snav-modalai_1.3.0_8x96.ipk
opkg install Python_3.6.9_8x96.ipk
opkg install libevent_2.1.11_8x96.ipk
opkg install tmux_2.1_8x96.ipk
opkg install opencv_3.4.6_8x96.ipk
opkg install rsync_3.1.2_8x96.ipk
```

## [Setting cameras](https://docs.modalai.com/camera-connections/)
`voxl-configure-cameras 3`

This updates the following files
 * /etc/snav/camera.downward.xml
 * /etc/snav/camera.stereo.xml
 * /etc/modalai/camera_env.sh

## Copy config files
```
cd snav_setup
cp _bashrc_voxl ~/.bashrc
cp _tmux2.1.conf ~/.tmux.conf
cp _bash_aliases ~/.bash_aliases
cp _vimrc ~/.vimrc
```

## Set environment variable based on vehicle ID, example 30
```
export MAV_ID=30
echo "export MAV_ID=$MAV_ID" >> ~/.bashrc
echo "export MAV_NAME=dragonfly$MAV_ID" >> ~/.bashrc
echo "export IMU_1_USED=true" >> ~/.bashrc
```