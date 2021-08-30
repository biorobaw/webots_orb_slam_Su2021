# webots_orb_slam_Su2021
A non-real-time integration of Webots robot simulator with the ORBSLAM2 library using ROS2 for environment localization and mapping with the intent of being extended to a fully real-time system.

Outside of the contents of this repository, this system requires a few other third-party software packages and systems. Installation of some of these packages are covered in more detail in the documentation. However, to touch on a few of the main ones, the ORBSLAM2 library is required for this system to run. The library must be installed and built before running it on the dataset generated by the Webots/ROS2 portion of the system. That being said, ROS2 must already be installed as well as the 'webots_ros2' repository, which must be present and built in the 'ws/src' directory before the simulator is run with the "wanderer" controller package. More information is present in the documentation present in the 'docs' folder. 

Along with the source code, configuration files, and other resources, this repository also contains a test dataset in the 'sequence' folder that users can use to test the system.
