==============================
Navigation Stack Configuration
==============================

.. raw:: html

    <a href="https://github.com/Interbotix/interbotix_ros_rovers/tree/rolling/interbotix_ros_xslocobots/interbotix_xslocobot_nav"
        class="docs-view-on-github-button"
        target="_blank">
        <img src="../_static/GitHub-Mark-Light-32px.png"
            class="docs-view-on-github-button-gh-logo">
        View Package on GitHub
    </a>

Overview
========

This package configures the ROS 2 Navigation Stack needed to give any X-Series Interbotix LoCoBot
platform the ability to perform simultaneous localization and mapping (SLAM), navigation, or just
localization. It can be used with just the `Intel RealSense D435`_ camera or with both the camera
and the `A2 RPLidar`_ laser scanner. Localization and mapping is done using the `rtabmap_ros`_ or
(in some distributions) the `slam_toolbox`_ ROS packages while navigation is accomplished via the
`Nav2`_ stack.

.. note::

    For best results, this package should be run with the robot in an indoor, uncluttered
    environment that does not contain too much sunlight and has minimal reflective surfaces.

.. _`Intel RealSense D435`: https://www.intelrealsense.com/depth-camera-d435/
.. _`A2 RPLidar`: https://www.slamtec.com/en/Lidar/A2
.. _`rtabmap_ros`: https://github.com/introlab/rtabmap_ros/tree/ros2
.. _`slam_toolbox`: https://github.com/SteveMacenski/slam_toolbox
.. _`Nav2`: https://navigation.ros.org/

.. Structure
.. =========

Usage
=====

SLAM From Scratch
-----------------

To get SLAM up and running using RTAB-Map, type the following in a terminal on the robot computer:

.. code-block:: console

    $ ros2 launch interbotix_xslocobot_nav xslocobot_rtabmap.launch.py robot_model:=locobot_wx200 use_lidar:=true slam_mode:=mapping rtabmap_args:=-d

Localization
------------

Once you've finished mapping your desired environment, the next step is to have the robot uses its
sensors to just localize itself within the map while navigating. To do so, type the following in a
terminal on the robot computer:

.. code-block:: console

    $ ros2 launch interbotix_xslocobot_nav xslocobot_rtabmap.launch.py robot_model:=locobot_wx200 use_lidar:=true slam_mode:=localization

Troubleshooting
===============

Time out waiting for transform...
---------------------------------

When starting the Nav Stack (either when continuing a map or just doing localization) on your
robot, you may see some warnings appear in the terminal. For example...

.. code-block:: console

    Timed out waiting for transform from locobot_wx200/base_footprint to map to become available before running costmap, tf error: canTransform: target_frame map does not exist.. canTransform returned after 0.100567 timeout was 0.1

The reason this appears is because no map is being supplied to the navigation stack. The reason for
that is because it takes RTAB-Map a few seconds to generate the map from its database (which could
be hundreds of megabytes). As such, this warning can be safely ignored assuming it stops once
RTAB-Map gets the map out.

Rejected Loop Closure
---------------------

When starting the Nav stack or during mapping, you may see the following warning appear (or
similar) in the terminal...

.. code-block:: console

    Rtabmap.cpp:2533::process() Rejected loop closure 694 -> 773: Not enough inliers 0/20 (matches=0) between 694 and 772

Similar to the first warning, this can be ignored if it only shows up a few times at node startup.
It just means that RTAB-Map has failed to determine where the robot is in the map. If you're
mapping too quickly, this warning can also appear, so slow down a bit.

Robot does not go to same location after map reset
--------------------------------------------------

If using the Create® 3 base, the robot keeps its odometry through sessions and power cycles. You
may need to use the /reset_pose service with each new map to ensure the robot goes to the same
location.
