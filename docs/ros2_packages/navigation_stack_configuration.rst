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

This package configures the ROS Navigation Stack needed to give any X-Series Interbotix LoCoBot
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

Structure
=========

Usage
=====

Troubleshooting
===============

Video Tutorials
===============
