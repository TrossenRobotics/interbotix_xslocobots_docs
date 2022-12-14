====================
LoCoBot Descriptions
====================

.. raw:: html

    <a href="https://github.com/Interbotix/interbotix_ros_rovers/tree/rolling/interbotix_ros_xslocobots/interbotix_xslocobot_descriptions"
        class="docs-view-on-github-button"
        target="_blank">
        <img src="../_static/GitHub-Mark-Light-32px.png"
            class="docs-view-on-github-button-gh-logo">
        View Package on GitHub
    </a>

Overview
========

This package contains the URDFs and meshes for the robots in the Interbotix X-Series LoCoBot
Family. The STL files for each robot are located in a unique folder inside the `meshes directory`_.
Next, the URDFs for the robot are located in the `urdf directory`_. They are written in 'xacro'
format so that users have the ability to customize what parts of the URDF get loaded to the
parameter server (see the 'Usage' section below for details). Note that all the other ROS packages
in the sub-repo reference this package to launch the robot description.

.. _`meshes directory`: https://github.com/Interbotix/interbotix_ros_rovers/blob/rolling/interbotix_ros_xslocobots/interbotix_xslocobot_descriptions/meshes
.. _`interbotix_black.png`: https://github.com/Interbotix/interbotix_ros_rovers/blob/rolling/interbotix_ros_xslocobots/interbotix_xslocobot_descriptions/meshes/interbotix_black.png
.. _`urdf directory`: https://github.com/Interbotix/interbotix_ros_rovers/blob/rolling/interbotix_ros_xslocobots/interbotix_xslocobot_descriptions/urdf

Structure
=========

.. image:: images/xslocobot_descriptions_flowchart_ros2.png
    :align: center

This package contains the xslocobot_description.launch file responsible for loading parts or all of
the robot model. It launches up to four nodes as described below:

-   **joint_state_publisher** - responsible for parsing the 'robot_description' parameter to find
    all non-fixed joints and publish a JointState message with those joints defined.
-   **joint_state_publisher_gui** - does the same thing as the 'joint_state_publisher' node but
    with a GUI that allows a user to easily manipulate the joints.
-   **robot_state_publisher** - uses the URDF specified by the parameter robot_description and the
    joint positions from the joint_states topic to calculate the forward kinematics of the robot
    and publish the results via tf.
-   **rviz** - displays the virtual robot model using the transforms in the 'tf' topic.

Usage
=====

This package contains two launch files:

-   xslocobot_description.launch.py - should be included in any launch tree in which you want to
    load and parse the URDF, or can be launched to view any robot model's URDF in RViz.
-   remote_view.launch.py which simply loads RViz in the proper namespace.

xslocobot_description.launch.py
-------------------------------

To run this package, type the line below in a terminal. Note that the ``robot_model`` argument must
be specified as the name of one of the four locobot models. For example, to launch a LoCoBot with a
WidowX-200 arm, type:

.. code-block:: console

    $ ros2 launch interbotix_xslocobot_descriptions xslocobot_description.launch.py robot_model:=locobot_wx200 use_joint_pub_gui:=true

This is the bare minimum needed to get up and running. Take a look at the table below to see how to
further customize with other launch file arguments.

.. csv-table::
    :file: ../_data/xslocobot_description_ros2.csv
    :header-rows: 1

remote_view.launch.py
---------------------

This launch file should be run on a networked ROS computer to visualize the robot in RViz in real
time. See more in the :doc:`ROS 2 Interface Quickstart <../ros_interface/ros2/quickstart>`.

To run this launch file, enter the command below in a terminal.

.. code-block:: console

    $ ros2 launch interbotix_xslocobot_descriptions remote_view.launch.py

.. csv-table::
    :file: ../_data/remote_view_ros2.csv
    :header-rows: 1
