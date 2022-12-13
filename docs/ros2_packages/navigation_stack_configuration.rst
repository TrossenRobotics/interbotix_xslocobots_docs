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
and the `A2M8 RPLidar`_ laser scanner. The localization and mapping part is done using the
`rtabmap_ros`_ ROS package while the navigation part is accomplished via the `move_base`_ ROS
package.

.. note::

    For best results, this package should be run with the robot in an indoor, uncluttered
    environment that does not contain too much sunlight and has minimal reflective surfaces.

.. _`Intel RealSense D435`: https://www.intelrealsense.com/depth-camera-d435/
.. _`A2M8 RPLidar`: https://www.slamtec.com/en/Lidar/A2
.. _`rtabmap_ros`: http://wiki.ros.org/rtabmap_ros
.. _`move_base`: http://wiki.ros.org/move_base

Structure
=========

Usage
=====

Troubleshooting
===============

Video Tutorials
===============
