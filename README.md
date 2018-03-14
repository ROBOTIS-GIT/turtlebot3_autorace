![](https://github.com/ROBOTIS-GIT/ROBOTIS-Documents/blob/master/wiki-images/AutoRace/autorace_pics/autorace_rbiz_challenge_2017_robots_1.png)

# TurtleBot3 Auto on AutoRace 2017 - Source codes
Sources for TurtleBot3 Auto - AutoRace 2017

<!--
![Picture of Pi Camera mounted TurtleBot3 Burger]
-->

This source code is for AutoRace 2017. 

<!--
![](https://youtu.be/sp02Q4FHOWo)
-->

## 1. Preparations

### 1.1 Requirements

* TurtleBot3 Burger
  * ROS and dependent ROS packages needs to be installed in the robot
  * All functions of TurtleBot3 Burger which is described in [TurtleBot3 E-Manual](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) needs to be tested before running TurtleBot3 Auto source code

* Remote PC (Laptop, Desktop, etc.)
  * ROS and dependent ROS packages needs to be installed in the computer
  * All functions of TurtleBot3 Burger which is described in [TurtleBot3 E-Manual](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) needs to be tested before running TurtleBot3 Auto source code

* Add-ons on TurtleBot3 Burger
  * Raspberry Pi Camera Type G (Fisheye Lens) : Available [Here](https://www.waveshare.com/rpi-camera-g.htm)
    * See `Features of 4 screw holes` in the page very carefully before mounting on the frame of any conductive materials
  * Raspberry Pi Camera Mount

<!--
![Picture of mounted Pi Camera]
-->

* Track structure and Accessories, such as Traffic Signs, Traffic Lights, and other objects.
  * Get Sources of AutoRace Referee system from [autorace_referee](https://github.com/ROBOTIS-GIT/autorace_referee)
  * Get 3D CAD model data of the race track from [autorace_track](https://github.com/ROBOTIS-GIT/autorace_track)

### 1.2 Install Additional Dependent Packages

[Remote PC & TurtleBot SBC] Open new terminal, then enter

``` bash
$ sudo apt-get install ros-kinetic-image-transport ros-kinetic-cv-bridge ros-kinetic-vision-opencv python-opencv libopencv-dev ros-kinetic-image-proc
```

### 1.3 Calibration

#### 1.3.1 Camera Imaging Calibration

- 1. [Remote PC] Open new terminal, then enter

``` bash
$ roscore
```

- 2. [TurtleBot SBC] Open new terminal, then enter

``` bash
$ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_camera_pi.launch
```

- 3. [Remote PC] Open new terminal, then enter

```
$ rqt_image_view
```

then, click `/camera/image/compressed` or `/camera/image/` topic in the select box. If everything works fine, the screen should show you the view from the robot.

- 4. [Remote PC] Open new terminal, then enter

``` bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

then, click `camera`, adjust the parameter value that makes the camera show clean, enough bright image to you. After that, overwrite each values on to the `turtlebot3_autorace_camera/calibration/camera_calibration/camera.yaml`. This will make the camera set its parameters as you set here from next launching.

#### 1.3.2 Intrinsic Camera Calibration

- 1. [Remote PC] Open new terminal, then enter

``` bash
$ roscore
```

- 2. [TurtleBot SBC] Open new terminal, then enter

``` bash
$ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_camera_pi.launch
```

- 3. [Remote PC] Print checkerboard for camera calibration on A4 size paper. The checkerboard is in `turtlebot3_autorace_camera/data/checkerboard_for_calibration.pdf`. See [Calibration manual](http://wiki.ros.org/camera_calibration) and modify the parameter value written in `turtlebot3_autorace_camera/launch/turtlebot3_autorace_intrinsic_camera_calibration.launch`.

- 4. [Remote PC] Open new terminal, then enter

``` bash
$ export AUTO_IN_CALIB=calibration
$ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
```

- 5. [Remote PC] After finishing the calibration, intrinsic camera calibration file will be saved in `turtlebot3_autorace_camera/calibration/intrinsic_calibration/camerav2_320x240_30fps.yaml`. 

#### 1.3.3 Extrinsic Camera Calibration

- 1. [Remote PC] Open new terminal, then enter

``` bash
$ roscore
```

- 2. [TurtleBot SBC] Open new terminal, then enter

``` bash
$ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_camera_pi.launch
```

- 3. [TurtleBot SBC] Open new terminal, then enter

``` bash
$ export AUTO_IN_CALIB=action
$ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
```

- 4. [Remote PC] Open new terminal, then enter

``` bash
$ export AUTO_EX_CALIB=calibration
$ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_extrinsic_camera_calibration.launch
```

- 5. [Remote PC] Open new terminal, then enter

```
$ rqt
```

clicking <kbd>plugins</kbd> -> <kbd>visualization</kbd> -> <kbd>Image view</kbd> on the top of the screen will make extra monnitor for camera view. Make 2 extra monitor in the rqt plate by following it. Then, choose `/camera/image_extrinsic_calib/compressed` and `/camera/image_projected_compensated` topics on each of extra monitors. If everything works fine, one of the screen will show the image with red rectangle, and other one will show the ground projected view (bird's eye view) which is based on the one another.

- 6. [Remote PC] Open new terminal, then enter

``` bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

then, adjust the parameter value in `/camera/image_projection` and `/camera/image_compensation_projection` that carries out visual modifications on the image. The parameter `image_projection` will change the shape of the red rectangle of `/camera/image_extrinsic_calib/compressed` image. Intrinsic camera calibration will transform the image surrounded by the red rectangle, and will show the image that looks from over the lane. After that, overwrite each values on to the `yaml` files in  `turtlebot3_autorace_camera/calibration/extrinsic_calibration/`. This will make the camera set its parameters as you set here from next launching.

#### 1.3.4 Settings for Recognition

Until now, all the preprocess of image must have been tested. 

- 1. [Remote PC] Open new terminal, then enter

``` bash
$ roscore
```

- 2. [TurtleBot SBC] Open new terminal, then enter

``` bash
$ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_camera_pi.launch
```

- 3. [TurtleBot SBC] Open new terminal, then enter

``` bash
$ export AUTO_IN_CALIB=action
$ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
```

- 4. [Remote PC] Open new terminal, then enter

``` bash
$ export AUTO_EX_CALIB=action
$ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_extrinsic_camera_calibration.launch
```

From now, the following descriptions will mainly adjust `feature detector / color filter` for object recognition. Every adjustment after here is independent to each other's process. However, to make sure, if you want to adjust each parameters in series, finish every adjustment perfectly, then continue to next.

##### 1.3.4.1 Lane Detection

- 1. Put the robot on the lane. If you placed the robot correctly, `yellow line` should be placed on the left side of the robot, and of course, `white line` should be placed on the right side of the robot. Make sure that `turtlebot3_robot` node of `turtlebot3_bringup` package is not yet launched. If it is on running, the robot will suddenly runs on the track. 


- 2. [Remote PC] Open new terminal, then enter

``` bash
$ export AUTO_DT_CALIB=calibration
$ roslaunch turtlebot3_autorace_detect turtlebot3_autorace_detect_lane.launch
```

- 3. [Remote PC] Open new terminal, then enter

```
$ rqt
```

clicking <kbd>plugins</kbd> -> <kbd>visualization</kbd> -> <kbd>Image view</kbd> on the top of the screen will make extra monnitor for camera view. Make 3 extra monitor in the rqt plate by following it. Then, choose `/detect/image_yellow_lane_marker/compressed`, `/detect/image_lane/compressed` and `/detect/image_white_lane_marker/compressed` topics on each of extra monitors. If everything works fine, left and right screen will show the filtered image of the yellow line and the white line, and the center screen will show the lane of where the robot should go. In the calibration mode, left and right screen will show white, and the center screen may show abnormal result. From here, you should adjust the filter parameters to show up correct lines and the direction.

- 4. [Remote PC] Open new terminal, then enter

``` bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

then, adjust the parameter value in `/camera/image_projection` and `/camera/image_compensation_projection` that carries out visual modifications on the image. The parameter `image_projection` will change the shape of the red rectangle of `/camera/image_extrinsic_calib/compressed` image. Intrinsic camera calibration will transform the image surrounded by the red rectangle, and will show the image that looks from over the lane. After that, overwrite value on to the `lane.yaml` file in  `turtlebot3_autorace_detect/param/lane/`. This will make the camera set its parameters as you set here from next launching.

Tip: calibration process of line color filtering is sometimes so-so difficult because of your physical environment which includes the luminance of light in the room, etc. Hence, you should have patience to carry out this procedure. To make everything quickly, put the value of `turtlebot3_autorace_detect/param/lane/lane.yaml` on the reconfiguration parameter, then start calibration. Calibrate hue low - high value at first. (1) Hue value means the color, and every colors, like `yellow`, `white`, have their own region of hue value (refer to hsv map). Then calibrate saturation low - high value. (2) Every colors have also their own field of saturation. Finally, calibrate the lightness low - high value. (3) In the source code, however, have auto-adjustment function, so calibrating lightness low value is meaningless. Just put the lightness high value to 255. Clearly filtered line image will give you clear result of the lane. 

- 5. [Remote PC] After overwriting the calibration file, close `rqt_rconfigure` and `turtlebot3_autorace_detect_lane`, then enter

``` bash
$ export AUTO_DT_CALIB=action
$ roslaunch turtlebot3_autorace_detect turtlebot3_autorace_detect_lane.launch
```

- 6. Check if the results come out well by entering 

[Remote PC] 
``` bash
$ roslaunch turtlebot3_autorace_control turtlebot3_autorace_control_lane.launch
```

[TurtleBot SBC] 
``` bash
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

After entering these commands, the robot will kick-off to run. 

##### 1.3.4.2 Traffic Sign

- 1. Traffic sign detection needs some pictures of the traffic sign. Take their pictures by using `rqt_image_view` node and edit their size, shape by any of `photo editor` available in linux. The node finds the traffic sign with `SIFT algorithm`, so if you want to use your customized traffic signs ( which is not introduced in the `autorace_track`), just be aware of `More edges in the traffic sign gives better recognition results from SIFT`. 

- 2. Put the robot on the lane. At this time, the traffic sign should be placed to where the robot can see it easily. Make sure that `turtlebot3_robot` node of `turtlebot3_bringup` package is not yet launched. If it is on run, the robot may suddenly run on the track.

- 3. [Remote PC] Open new terminal, then enter

```
$ rqt_image_view
```

then, click `/camera/image_compensated` topic in the select box. If everything works fine, the screen should show you the view from the robot.

- 4. [Remote PC] Take the picture by <kbd>alt</kbd> + <kbd>print screen</kbd>, edit the captured with your preferred photo editor. After that, place the picture to `[where the turtlebot3_autorace package you've placed]/turtlebot3_autorace/turtlebot3_autorace_detect/file/detect_sign/` and rename it as you want. (Although, you should change the file name written in the source `detect_sign.py`, if you want to change the default file names.)


- 5. [Remote PC] Open new terminal, then enter

``` bash
$ roslaunch turtlebot3_autorace_detect turtlebot3_autorace_detect_sign.launch
```


- 6. [Remote PC] Open new terminal, then enter

```
$ rqt_image_view
```

then, click `/detect/image_traffic_sign/compressed` topic in the select box. If everything works fine, the screen will show the result of traffic sign detection, if it succeeds to recognize it.

##### 1.3.4.3 Traffic Light

- 1. Put the robot on the lane. If you placed the robot correctly, `yellow line` should be placed on the left side of the robot, and of course, `white line` should be placed on the right side of the robot. Make sure that `turtlebot3_robot` node of `turtlebot3_bringup` package is not yet launched. If it is on running, the robot will suddenly runs on the track. 

- 2. [Remote PC] Open new terminal, then enter

``` bash
$ export AUTO_DT_CALIB=calibration
$ roslaunch turtlebot3_autorace_detect turtlebot3_autorace_detect_traffic_light.launch
```

- 3. [Remote PC] Open new terminal, then enter

```
$ rqt
```

clicking <kbd>plugins</kbd> -> <kbd>visualization</kbd> -> <kbd>Image view</kbd> on the top of the screen will make extra monnitor for camera view. Make 3 extra monitor in the rqt plate by following it. Then, choose `/detect/image_yellow_light`, `/detect/image_yellow_light`, `/detect/image_yellow_light` and `/detect/image_traffic_light` topics on each of extra monitors. If everything works fine, three screen will show the filtered image of the red / yellow / green light, and the other one will show the recognized color with short string. In the calibration mode, three screen will show white, and the other screen may show plain result. From here, you should adjust the filter parameters to show up correct lines and the direction.

- 4. [Remote PC] Open new terminal, then enter

``` bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

then, adjust the parameter value in `/detect_traffic_light`. Changing the value of color filter will show the change of filtered view on each color's screen. After that, overwrite value on to the `traffic_light.yaml` file in  `turtlebot3_autorace_detect/param/traffic_light/`. This will set its parameters as you set here from next launching.

Tip: same as 1.3.4.1

- 5. [Remote PC] After overwriting the calibration file, close `rqt_rconfigure` and `turtlebot3_autorace_detect_traffic_light`, then enter

``` bash
$ export AUTO_DT_CALIB=action
$ roslaunch turtlebot3_autorace_detect turtlebot3_autorace_detect_traffic_light.launch
```

- 6. Use `rqt_image_view` node, and check if the results come out well 

##### 1.3.4.4 Parking Lot

- 1. `Parking` needs only one preparation, traffic sign recognition. 

- 2. Place the dummy robot on either of parking lot. 

- 3. Place the robot on the lane appropriately.

##### 1.3.4.5 Level Crossing

- 1. Level Crossing finds 3 red rectangles on the level, and calculates whether the level is opened or closed, and how much near the robot is come. 

- 2. Put the robot on the lane correctly. Then, bring the robot in front of closed level. 

- 3. [Remote PC] Open new terminal, then enter

``` bash
$ export AUTO_DT_CALIB=calibration
$ roslaunch turtlebot3_autorace_detect turtlebot3_autorace_detect_level.launch
```

- 4. [Remote PC] Open new terminal, then enter

```
$ rqt
```

clicking <kbd>plugins</kbd> -> <kbd>visualization</kbd> -> <kbd>Image view</kbd> on the top of the screen will make extra monnitor for camera view. Make 3 extra monitor in the rqt plate by following it. Then, choose `/detect/image_level_color_filtered` and `/detect/image_level` topics on each of extra monitors. If everything works fine, three screen will show the filtered image of the red rectangles, and another one will draw a line which connects the rectangles. In the calibration mode, a screen will show white, and the other screen may show plain result. From here, you should adjust the filter parameters to show up correct lines and the direction.

- 5. [Remote PC] Open new terminal, then enter

``` bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

then, adjust the parameter value in `/detect_level`. Changing the value of color filter will show the change of filtered view on each color's screen. After that, overwrite value on to the `level.yaml` file in  `turtlebot3_autorace_detect/param/level/`. This will set its parameters as you set here from next launching.

Tip: same as 1.3.4.1

- 6. [Remote PC] After overwriting the calibration file, close `rqt_rconfigure` and `turtlebot3_autorace_detect_level`, then enter

``` bash
$ export AUTO_DT_CALIB=action
$ roslaunch turtlebot3_autorace_detect turtlebot3_autorace_detect_level.launch
```

- 7. Use `rqt_image_view` node, and check if the results come out well 

##### 1.3.4.6 Tunnel

- 1. Tunnel node will bring you from the entrance to the exit by using turtlebot3 navigation package. What you should calibrate is mapping the tunnel (or if you are using the autorace track as it is, you don't need to modify it by yourself) and check the `pose` of how the robot should be posed right before it comes out from tunnel (this is also unnecessary when you are using the default map).  

- 2. [Remote PC] Check the `pose` of `exit` on RViz, while the `SLAM` or `Navigation` package is running. After that, overwrite value on to the `detect_tunnel.py` file `line 144`

#### 1.3.5 Run autonomous driving

- 1. From now, all the related nodes will be run in `action mode`. Close all `ROS-related programs` and `terminals` on `Remote PC` and `TurtleBot SBC`, if some were not closed yet. Then, put the robot on the lane correctly.

- 2. [Remote PC] Open new terminal, then enter

``` bash
$ roscore
```

- 3. [TurtleBot SBC] Open new terminal, then enter

``` bash
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

- 4. [TurtleBot SBC] Open new terminal, then enter

``` bash
$ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_camera_pi.launch
```

- 5. [TurtleBot SBC] Open new terminal, then enter

``` bash
$ export AUTO_IN_CALIB=action
$ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
```

- 6. [Remote PC] Open new terminal, then enter

``` bash
$ export AUTO_EX_CALIB=action
$ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_extrinsic_camera_calibration.launch
```

- 7. [Remote PC] Open new terminal, then enter

``` bash
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch
```
- 8. [Remote PC] Open new terminal, then enter

``` bash
$ rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2" 
```

turtlebot3_autorace_core will control all system in the package (open and close the launch, nodes in the package.)

## 2. TO DO

### 2.1 turtlebot3_autorace_msgs

### 2.2 turtlebot3_autorace_controls

### 2.3 turtlebot3_autorace_cores