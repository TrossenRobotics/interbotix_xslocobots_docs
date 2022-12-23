===============
LoCoBot Control
===============

.. raw:: html

    <a href="https://github.com/Interbotix/interbotix_ros_rovers/tree/rolling/interbotix_ros_xslocobots/interbotix_xslocobot_control"
        class="docs-view-on-github-button"
        target="_blank">
        <img src="../_static/GitHub-Mark-Light-32px.png"
            class="docs-view-on-github-button-gh-logo">
        View Package on GitHub
    </a>

Overview
========

This package contains the configuration and launch files necessary to easily start the X-Series
LoCoBot platform. This includes launching the **xs_sdk** node responsible for driving the DYNAMIXEL
motors on the robot, loading the URDF to the ``robot_description`` launch configuration, starting
the mobile base nodes, and activating the depth camera and 2D lidar. Essentially, this package is
what all 'downstream' ROS packages should reference to get the robot up and running.

Structure
=========

.. image:: images/xslocobot_control_flowchart_ros2.png
    :align: center

As shown in the image above, the *interbotix_xslocobot_control* package builds on top of the
*interbotix_xslocobot_descriptions* and *interbotix_xs_sdk* packages among many others. To get
familiar with the nodes not described below, just hop over to the package documentation that
launches them.

-   **rplidar_composition**: responsible for starting the `RPlidar A2M8`_ sensor and publishing
    `LaserScan`_ messages on the ``/<robot_name>/scan`` topic. This node is only launched if the
    ``use_lidar`` launch configuration is set to ``true``. See the `RPLidar ROS Wiki`_ for
    parameter descriptions. All parameters were left at their default values except for the
    following:

    -   *frame_id*: ``/<robot_name>/laser_frame_link``
    -   *serial_port*: ``/dev/rplidar``
    -   *angle_compensate*: ``true``

-   **kobuki nodes**: a group of three nodes responsible for starting the Kobuki base. See the
    `Kobuki ROS Wiki`_ for parameter descriptions. These nodes are only launched if the
    ``base_type`` launch configuration is set to ``kobuki`` and the ``use_base`` launch
    configuration is set to ``true``. All parameters were left at their default values except for
    the following:

    -   *odom_frame*: ``/<robot_name>/odom``
    -   *base_frame*: ``/<robot_name>/base_footprint``
    -   *acceleration_limiter*: ``true``

-   **create3 nodes**: a group of nodes responsible for controlling the Create® 3 mobile base.
    These nodes are managed onboard the Create® 3 and will always be running if the base is turned
    on. The parameters can be changed using its :doc:`webserver
    </getting_started/create3_configuration>`.

-   **tf_rebroadcaster**: rebroadcasts TFs from the Create® 3's namespaces TF topic to the root TF
    topic. This node is only launched if the the ``use_base_odom_tf`` and ``use_base`` launch
    configurations are set to ``true`` and if the ``base_type`` launch configuration is set to
    ``create3``. As of the time of writing these packages, it was not possible to tell the Create®
    3's nodes to publish the TF in the root TF namespace so this node serves as a workaround. The
    node is configured using the `tf_rebroadcaster.yaml configuration file`_.

-   **realsense2_camera_node**: responsible for running the `RealSense D435`_ camera and publishing
    a variety of image topics. See the `realsense_ros`_ repository for parameter
    descriptions. See the `rs_camera.yaml configuration file`_ for detailed parameter information.

You will also notice a `config`_ directory containing many YAML files. Each file (beside the
modes.yaml one and the others mentioned above) specifies the names and initial register values for
all the motors that make up a specific locobot. There is also some 'meta-info' like names of joint
groups, the desired joint-topic name and publishing frequency, etc. For a full explanation of each
of these parameters, check out the `Motor Config file template`_. The other file located in that
directory is the Mode Config one (a.k.a mode.yaml). The parameters in there define the desired
operating modes for either a group of joints or single joints, and whether or not they should be
torqued on/off at node startup. See more by referencing the `Mode Config file template`_.
Typically, the Motor Config file is only defined here while the Mode Config file is also defined in
any 'downstream' ROS package. This makes it easy for users to configure their desired motor
operating modes depending on their project.

.. _`RPlidar A2M8`: https://www.slamtec.com/en/Lidar/A2
.. _`LaserScan`: http://docs.ros.org/latest/api/sensor_msgs/html/msg/LaserScan.html
.. _`RPLidar ROS Wiki`: http://wiki.ros.org/rplidar
.. _`Kobuki ROS Wiki`: http://wiki.ros.org/kobuki_node
.. _`tf_rebroadcaster.yaml configuration file`: https://github.com/Interbotix/interbotix_ros_rovers/blob/rolling/interbotix_ros_xslocobots/interbotix_xslocobot_control/config/tf_rebroadcaster.yaml
.. _`RealSense D435`: https://www.intelrealsense.com/depth-camera-d435/
.. _`realsense_ros`: https://github.com/IntelRealSense/realsense-ros/tree/ros2-development
.. _`config`: https://github.com/Interbotix/interbotix_ros_rovers/blob/rolling/interbotix_ros_xslocobots/interbotix_xslocobot_control/config
.. _`rs_camera.yaml configuration file`: https://github.com/Interbotix/interbotix_ros_rovers/blob/rolling/interbotix_ros_xslocobots/interbotix_xslocobot_control/config/rs_camera.yaml
.. _`Motor Config file template`: https://github.com/Interbotix/interbotix_ros_core/blob/rolling/interbotix_ros_xseries/interbotix_xs_sdk/config/motor_configs_template.yaml
.. _`Mode Config file template`: https://github.com/Interbotix/interbotix_ros_core/blob/rolling/interbotix_ros_xseries/interbotix_xs_sdk/config/mode_configs_template.yaml

Usage
=====

To run this package on the physical robot, type the line below in a terminal (assuming the
``locobot_wx200`` is being launched with the onboard lidar and camera).

.. code-block:: console

    $ ros2 launch interbotix_xslocobot_control xslocobot_control.launch.py robot_model:=locobot_wx200 use_base:=true use_lidar:=true use_camera:=true

If you are working from a remote computer and would like to visualize the robot using RViz, open a
terminal on your remote and run:

.. code-block:: console

    $ ros2 launch interbotix_xslocobot_descriptions remote_view.launch.py

Note that in order for this to work, you must first run the :doc:`remote installation
</ros_interface/ros2/software_setup>` script on your remote computer.

To further customize the launch file at run-time, refer to the table below.

.. csv-table::
    :file: ../_data/xslocobot_control_ros2.csv
    :header-rows: 1

Video Tutorials
===============

ROS 2 Control
-------------

.. youtube:: A1-PpKyBbzw
    :align: center
    :width: 70%
