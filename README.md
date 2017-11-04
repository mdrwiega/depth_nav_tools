## depth_nav_tools
The set of software tools dedicated to mobile robot autonomous navigation with depth sensor, for example Microsoft Kinect.

The metapackage depth_nav_tools contains following packages:

- **laserscan_kinect** -- It converts depth image to laser scanner format (LaserScan). 
Firstly, the node finds the smallest value of distance in each column of depth image 
and converts it to polar coordinates. The package provides features like:
  - removing ground plane from output data,
  - sensor tilt compenstaion.

  However, height of the sensor position and the tilt angle must be known for correct data processing.
  Parameters should be calculated in the frame of the ground plane.

- **depth_sensor_pose** -- It detects the ground plane on depth image and estimates height and tilt angle of depth sensor.
The procedure of the ground plane detection based on the RANSAC algorithm and acceptable parameters ranges. 

- **cliff_detector** -- This tool detects negative objects like a cliffs or downstairs.
It uses known sensor pose to determine obstables placed below ground plane.

- **nav_layer_from_points** -- It creates navigation costmap layer based on received points (i.e. from cliff_detector).

- **depth_nav_msgs** -- Specific messages for other packages.

## Documentation 
General documentation is available at the [ROS wiki](http://wiki.ros.org/depth_nav_tools) and in the publication "[A set of depth sensor processing ROS tools for wheeled mobile robot navigation"(PDF)](http://www.jamris.org/images/ISSUES/ISSUE-2017-02/48_56%20Drwiega.pdf) by M. Drwięga and J. Jakubiak (Journal of Automation, Mobile Robotics & Intelligent Systems, 2017).

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

The picture shows comparison between laser scan based on converted depth image from Microsoft Kinect (blue points) and laser scan from scanner Hokuyo URG-04LX-UG01 (black points).
