# depth_nav_tools

The set of software tools dedicated to a mobile robot autonomous navigation with a depth sensor, for example Kinect.

The metapackage depth_nav_tools contains following packages:

- **laserscan_kinect** -- It converts the depth image to a laser scanner format (LaserScan).
The node finds the smallest value of distance in each column of the depth image
and converts it to polar coordinates. The package provides features like:
  - removing a ground plane from the output data,
  - a sensor tilt compenstaion.

  However, the sensor position (heigh and tilt angle) must be known for a correct data processing.
  Parameters should be calculated in a frame of the ground plane.

- **depth_sensor_pose** -- It detects the ground plane on the depth image and estimates the height and the tilt angle of the depth sensor relative to ground.
The procedure of the ground plane detection based on the RANSAC algorithm and ranges of acceptable parameters.

- **cliff_detector** -- This tool detects negative objects like cliffs or downstairs.
It uses a known sensor pose to determine obstacles placed below the ground plane.

- **nav_layer_from_points** -- It creates navigation costmap layer based on received points, for example from the *cliff_detector*.

## Additional documentation

A full documentation is available at the [ROS wiki](http://wiki.ros.org/depth_nav_tools) and in the publication "[A set of depth sensor processing ROS tools for wheeled mobile robot navigation"(PDF)](http://www.jamris.org/images/ISSUES/ISSUE-2017-02/48_56%20Drwiega.pdf) by M. Drwięga and J. Jakubiak (Journal of Automation, Mobile Robotics & Intelligent Systems, 2017).

BibTeX:
```
@ARTICLE{drwiega17jamris,
  author = {Michał Drwięga and Janusz Jakubiak},
  title = {A set of depth sensor processing {ROS} tools for wheeled mobile robot navigation},
  journal = {Journal of Automation, Mobile Robotics & Intelligent Systems (JAMRIS)},
  year = 2017,
  doi = {10.14313/JAMRIS_2-2017/16},
  note = {Software available at \url{http://github.com/mdrwiega/depth_nav_tools}}
}
```

## laserscan_kinect

### The example of obstacles detection by the laserscan_kinect

The picture shows comparison between a laser scan based on the converted depth image from a Microsoft Kinect (blue points) and a laser scan from a scanner Hokuyo URG-04LX-UG01 (black points).
![Laserscan Kinect detection](http://wiki.ros.org/laserscan_kinect?action=AttachFile&do=get&target=laserscan_kinect_detection.jpg)

### Tuning

During the tuning process additional debug image can be used. It contains lines that represent the lower and upper bounds of the detection area. Also, closest points in each image column are visible.
![laserscan_kinect_dbg](https://user-images.githubusercontent.com/8460945/107285398-62629f80-6a5f-11eb-8d7b-2c23f7247566.png)


### Usage
To start a node laserscan_kinect it can be used a following command
`roslaunch laserscan_kinect laserscan.launch`

### Subscribed topics
- */image* ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)) - depth image which will be converted to laserscan.
- */camera_info* ([sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html)) - additional information about the depth sensor.

### Published topics
- */scan* ([sensor_msgs/LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html)) - the converted depth image in form of laser scan. It contains information about the robot surrounding in a planar scan.

### Parameters
The file /config/params.yaml contains default parameters values.

- *~output_frame_id* (str) - frame id for the output laserscan message.
- *~range_min* (double) - minimum sensor range (in meters). Pixels in depth image with values smaller than this parameter are ignored in processing.
- *~range_max* (double) - maximum sensor range (in meters). Pixels in depth image with values greater than this parameter are ignored in processing.
- *~depth_img_row_step* (int) - Row step in depth image processing. Increasing this parameter we decrease computational complexity of algorithm but some of data are lost.
- *~scan_height* (int) - height of used part of depth image (in pixels).
- *~cam_model_update* (bool) - determines if continuously camera model data update is neccessary. If it's true, then camera model (sensor_msgs/CameraInfo) from topic camera_info is updated with each new depth image message. Otherwise, camera model and parameters associated with it are updated only at the start of node or when node parameter are changed by dynamic_reconfigure.

- *~ground_remove_en* (bool) - determines if ground remove from output scan feature is enabled. The ground removing method to work needs a correctly values of parameters like a sensor_tilt_angle and sensor_mount_height.
- *~sensor_mount_height* (double) - height of depth sensor optical center mount (in meters). Parameter is necessary for the ground removing feature. It should be measured from ground to the optical center of depth sensor.
- *~sensor_tilt_angle* (double) - depth sensor tilt angle (in degrees). If the sensor is leaning towards the ground the tilt angle should be positive. Otherwise, the value of angle should be negative.
- *~ground_margin* (double) - margin in ground removing feature (in meters).
- *~tilt_compensation_en* (bool) - parameter determines if sensor tilt angle compensation is enabled.


## depth_sensor_pose
### Usage
To start a node laserscan_kinect it can be used a following command
`roslaunch depth_sensor_pose depth_sensor_pose.launch.launch`

### Subscribed topics
- */image* ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)) - depth image which will be converted to laserscan.
- */camera_info* ([sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html)) - additional information about the depth sensor.

### Published topics
- */height* (double) - the sensor height (the distance from the ground to the center of optical sensor)
- */tilt_angle* (double) - the sensor tilt angle (in deg)
- */debug_image* ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)) - the debug image to check which points are used in the ground plane estimation, enabled only if *publish_dbg_info* parameter is set to *true*.

### Parameters
- *~rate* (dobule) - Data processing frequency  (Hz)
- *~range_min* (double) - minimum sensor range (in meters). Pixels in depth image with values smaller than this parameter are ignored in processing.
- *~range_max* (double) - maximum sensor range (in meters). Pixels in depth image with values greater than this parameter are ignored in processing.
- *~mount_height_min* (double) - minimum height of the depth sensor (m)
- *~mount_height_max* (double) - maximum height of the depth sensor (m)
- *~tilt_angle_min* (double) - minimum sensor tilt angle (degrees)
- *~tilt_angle_max* (double) - maximum sensor tilt angle (degrees)
- *~cam_model_update* (bool) - determines if continuously camera model data update is neccessary. If it's true, then camera model (sensor_msgs/CameraInfo) from topic camera_info is updated with each new depth image message. Otherwise, camera model and parameters associated with it are updated only at the start of node or when node parameter are changed by dynamic_reconfigure
- *~used_depth_height* (int) - used depth height from img bottom (px)
- *~depth_img_step_row* (int) - rows step in depth processing (px)
- *~depth_img_step_col* (int) - columns step in depth processing (px)

- *~ground_max_points* (int) - max ground points in selection stage
- *~ransac_max_iter* (int) - max number of RANSAC iterations
- *~ransac_dist_thresh* (double) - RANSAC distance threshold

- *~publish_dbg_info* (bool) - determines if debug image should be published

## Building

`colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`

## Tests
Currently unit tests are implemented only for the **laserscan_kinect** package.
- `catkin_make run_tests_laserscan_kinect`
