# depth_nav_tools
Set of tools for mobile robot navigation with depth sensor, for example 
Microsoft Kinect.

Actually depth_nav_tools contains only package laserscan_kinect which converts
depth image to laser scan (LaserScan). It allows to use Microsoft Kinect sensor
for navigation purposes. Package laserscan_kinect finds smallest value of
distance in each column of depth image and converts it to polar coordinates.

Moreover, package provides features like ground removing from scan and sensor
tilt compensation in distance values, but it is necessary to know height of
sensor optical center and tilt angle in frame of ground.
