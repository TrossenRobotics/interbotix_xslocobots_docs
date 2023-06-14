======================
ROS 2 Quickstart Guide
======================

.. note::

    This guide assumes that you have completed the :ref:`ROS 2 Remote Install
    <ros2-software-setup-remote-install-label>` on your remote computer.

This guide is intended to get the user familiar with the basic functions and interfaces of the ROS
2 Interface. Make sure that you have completed and are familiar with the steps taken in the
:doc:`Basic Usage Guide </getting_started/user_guide>` and the :doc:`ROS 2 Software Setup Guide
<./software_setup>`.

.. , and the :doc:`ROS 2 RMW Configuration Guide <rmw_configuration>`.

1.  Get familiar with the physical robot rover (let's say... a LoCoBot WidowX-250 6DOF with lidar!)
    by executing the following command in a terminal ssh'ed into the LoCoBot:

    .. code-block:: console

        $ ros2 launch interbotix_xslocobot_control xslocobot_control.launch.py robot_model:=locobot_wx250s use_base:=true use_camera:=true use_lidar:=true

2.  Now, in a terminal on your remote computer (not via SSH), type...

    .. code-block:: console

        $ ros2 launch interbotix_xslocobot_descriptions remote_view.launch.py

3.  RViz should appear on your remote computer and display a virtual real-time representation of
    the robot!

.. image:: images/rviz_remote.png
    :align: center
    :width: 70%

4.  By default, all the DYNAMIXEL motors in the robot are torqued on so it will be very difficult
    to manually manipulate them. To torque off all the motors, execute the command below in another
    terminal (either via SSH or on your remote computer).

    .. warning::

        This command will cause the robot arm (if present) to collapse (if it's not already
        resting) so manually hold or secure it before executing.

    .. code-block:: console

        $ ros2 service call /locobot/torque_enable "{cmd_type: 'group', name: 'all', enable: false}"

    .. note::

        The command above torques off every motor in the ``all`` group. This is a special group
        that includes every DYNAMIXEL motor. To only torque off the arm motors, change the name
        from ``all`` to ``arm``. Likewise, to only torque off the motors controlling the camera,
        change the name from ``all`` to ``camera``.

        .. list-table::
            :header-rows: 1
            :align: center
            :widths: 10 40

            * - Group Name
              - Servos in Group
            * - ``all``
              - Every DYNAMIXEL servo on the robot
            * - ``arm``
              - All DYNAMIXEL servos on the arm excluding the gripper
            * - ``camera``
              - All DYNAMIXEL servos in the camera pan & tilt mechanism

5.  Now you should be able to freely manipulate the arm, gripper, and pan/tilt mechanism. Take note
    of how the RViz model accurately mimics the real robot. To make the robot hold a certain pose,
    manually hold the arm in the desired pose and execute the following command:

    .. code-block:: console

        $ ros2 service call /locobot/torque_enable "{cmd_type: 'group', name: 'all', enable: true}"

6.  You can now let go and observe how the arm and pan/tilt mechanism stay in place.

7.  Now let's visualize the sensor output! In the RViz window, check the **Camera** and
    **LaserScan** (if you have a lidar) displays and adjust the topic names as necessary. You
    should see image data from the camera streaming in the lower left corner of the window, and
    small red pixels being displayed in the RViz grid from the lidar.

8.  To move the base with a translational velocity of 0.5 m/s and angular velocity of 0.3 rad/s,
    type the following in another terminal...

    .. warning::

        This command will cause the base to drive forward and to its left for around three seconds.
        Make sure there is nothing in the robot's path before running it.

    .. code-block:: console

        # create 3
        $ ros2 topic pub --once /locobot/mobile_base/cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'

9.  You can stop the base's movement at any time by pressing :kbd:`Ctrl` + :kbd:`C` the publisher's
    terminal.

10. Shutdown all nodes by pressing :kbd:`Ctrl` + :kbd:`C` in the terminal where you started the
    launch file.

11. Another way to check if all functions work is to launch the :doc:`Joystick Control package
    <../../ros2_packages/joystick_control>`. This package allows you to control your LoCoBot using a
    Bluetooth controller. Check the package's :ref:`usage section
    <ros-joystick-control-usage-label>` for more details.

That ends the quickstart tutorial. To get familiar with the architecture and launch file arguments,
refer to the documentation of the core packages. Start with the :doc:`LoCoBot Descriptions
<../../ros2_packages/locobot_descriptions>` package, then the :doc:`LoCoBot Control
<../../ros2_packages/locobot_control>` package. Next, look at the :doc:`Gazebo Configuration
<../../ros2_packages/simulation_configuration>` package followed by the :doc:`ROS Controllers
Configuration <../../ros2_packages/ros_control>` and :doc:`MoveIt Configuration
<../../ros2_packages/moveit_motion_planning_configuration>` packages. This is the most logical approach
to take to gain a better understanding of how they relate to each other.

Afterwards, feel free to check out the demo projects like :doc:`Joystick Control
<../../ros2_packages/joystick_control>`, or any of the other :doc:`ROS Open Source Packages
<../../ros2_packages>`.
