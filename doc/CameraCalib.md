### Camera-IMU Calibration

## Install Kalibr

Clone kalibr into a ROS workspace, build, and source.

```
git clone git@github.com:ethz-asl/kalibr.git
```

See [their wiki](https://github.com/ethz-asl/kalibr/wiki/installation) for more information about installation and dependencies (unfortunately dependency information is only provided for ROS indigo/14.04, but it works on melodic/18.04).
At the least, you probably need

```
sudo apt-get install flex
pip2 install python-igraph
```
Installing python-igraph take a long time.

Go to workspace and build as release

```
catkin build -DCMAKE_BUILD_TYPE=Release
```

## Take a ROS bag using the bagging scripts

Use an April tag or checkerboard grid that we have at PERCH, and take a bag of the camera pointing at it.

- Try to excite all IMU axes (rotation and translation)
- Avoid shocks, especially at the beginning/end when you pick up the sensor
- Good illumination
- Keep the whole grid in the frame (watch playback on RViz)

In the home directory of the drone, run

```
sudo -s
./tmux_calibration.sh
```

Once the tmux loads, get the drone in position and manually start the bagging script by hitting `Enter` in the `Bagging` tab.

## Processing the bag in Kalibr

```
roscd snavquad_interface/config/kalibr
kalibr_calibrate_cameras \
--bag [your bag path] \
--topics MVSampleVISLAMEagle/snap_cam_output_image \
--models pinhole-radtan \
--target aprilgrid.yaml
```

This will output `camchain-%BAGNAME%.yaml`: Results in YAML format. This file is used as an input for the camera-imu calibrator.

```
roscd snavquad_interface/config/kalibr
kalibr_calibrate_imu_camera --bag [your bag path]  \
--cam camchain-[your_bag_name].yaml \
--imu snap_imu.yaml \
--target aprilgrid.yaml
```

See [Kalibr wiki](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration#calibration-target) for more information if it's not working.
