## depth_nav_tools
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

- **nav_layer_from_points** -- It creates navigation costmap layer based on received points, for example from `cliff_detector`.

- **depth_nav_msgs** -- Custom messages for other depth nav related packages.

## Documentation
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

### The example of obstacles detection by laserscan_kinect
![Laserscan Kinect detection](http://wiki.ros.org/laserscan_kinect?action=AttachFile&do=get&target=laserscan_kinect_detection.jpg)

The picture shows comparison between a laser scan based on the converted depth image from a Microsoft Kinect (blue points) and a laser scan from a scanner Hokuyo URG-04LX-UG01 (black points).
