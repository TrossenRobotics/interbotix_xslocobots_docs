===============
Create® 3 Setup
===============

.. contents::
  :local:

Setup
=====

This section details the setup for your Create® 3 base. Follow Phases 1 and 3 in `iRobot's Create®
3 Setup Guide`_. You should not need to follow Phase 2 since we will provide the latest stable
firmware for the base.

.. _`iRobot's Create® 3 Setup Guide`: https://edu.irobot.com/create3-setup

Configuration
=============

This section presents the default Interbotix configuration of the LoCoBot's Create® 3 base. This
can be found on the Create® 3 webserver accessed through the LoCoBot's computer's web browser
(Firefox, etc.) at ``192.168.186.2/ros-config`` if connected over the base's USB-Ethernet
interface.

.. warning::

    Make sure that your Create® 3 base is configured to this default, otherwise it may not work
    properly or you may experience unexpected behavior.

.. note::

    If you modify any of these configurations, make sure to save the changes and restart the
    application for them to take effect.

Main configuration
------------------

.. tabs::

    .. group-tab:: ROS 1

        .. list-table::
            :header-rows: 1
            :widths: 20 20 60

            * - Field
              - Interbotix Default Value
              - Notes
            * - ROS 2 Domain ID
              - ``0``
              - This value is typically left as the default if you do not have multiple ROS domains
                on the same network. See `ROS 2 Domain ID Documentation`_.
            * - ROS 2 Namespace
              - ``/mobile_base``
              - The value dictates the namespace under which all Create® 3 nodes, topics, services,
                actions, and parameters will be, i.e. the ``/tf`` topic published by the base will
                be ``/mobile_base/tf``.
            * - RMW_IMPLEMENTATION
              - ``rmw_fastrtps_cpp``
              - This value tells which RMW implementation to use. See `ROS 2 RMW Implementation
                Documentation`_.
            * - Enable Fast DDS discovery server?
              -
              - Left blank as the ROS 1 implementation does not use the Fast DDS discovery server.
            * - Address and port of Fast DDS discovery server
              -
              - Left blank as the ROS 1 implementation does not use the Fast DDS discovery server.

        .. You can set the Main Configuration parameters via the command line using curl if connected
        .. to the Create® 3 base. The command below will set the configuration to the required
        .. parameters if using the ROS 1 Interface.

        .. .. code-block:: console

        ..     $ curl -X POST -d "ros_domain_id=0&ros_namespace=/mobile_base&rmw_implementation=rmw_fastrtps_cpp" "http://192.168.186.2/ros-config-save-main"

        .. attention::

            The application still needs to restart to take effect. To do this, you can either
            reboot the robot using the physical power button or its RobotPower service, or use a
            web browser to reset the application manually.

    .. group-tab:: ROS 2

        .. list-table::
            :header-rows: 1
            :widths: 20 20 60

            * - Field
              - Interbotix Default Value
              - Notes
            * - ROS 2 Domain ID
              - ``0``
              - This value is typically left as the default if you do not have multiple ROS domains
                on the same network. See `ROS 2 Domain ID Documentation`_.
            * - ROS 2 Namespace
              - ``/locobot/mobile_base``
              - The value dictates the namespace under which all Create® 3 nodes, topics, services,
                actions, and parameters will be, i.e. the ``/tf`` topic published by the base will be
                ``/locobot/mobile_base/tf``.
            * - RMW_IMPLEMENTATION
              - ``rmw_fastrtps_cpp``
              - This value tells which RMW implementation to use. See `ROS 2 RMW Implementation
                Documentation`_.
            * - Enable Fast DDS discovery server?
              - ``✔️``
              - Enables the Base to search for a Fast DDS discovery server.
            * - Address and port of Fast DDS discovery server
              - ``192.168.186.3:11811``
              - Tells Fast DDS the address and port of the Fast DDS discovery server. This is the
                IP address of the LoCoBot computer's Ethernet interface.

        .. You can set the Main Configuration parameters via the command line using curl if connected
        .. to the Create® 3 base. The command below will set the configuration to the required
        .. parameters if using the ROS 2 Interface.

        .. .. code-block:: console

        ..     $ curl -X POST -d "ros_domain_id=0&ros_namespace=/locobot/mobile_base&rmw_implementation=rmw_fastrtps_cpp" "http://192.168.186.2/ros-config-save-main"

        .. attention::

            The application still needs to restart to take effect. To do this, you can either
            reboot the robot using the physical power button or its RobotPower service, or use a
            web browser to reset the application manually.

.. _`ROS 2 Domain ID Documentation`: https://docs.ros.org/en/rolling/Concepts/About-Domain-ID.html
.. _`ROS 2 RMW Implementation Documentation`: https://docs.ros.org/en/rolling/Concepts/About-Different-Middleware-Vendors.html

Application ROS 2 Parameters File
---------------------------------

.. tabs::

    .. group-tab:: ROS 1

        .. code-block:: yaml

            /mobile_base/motion_control:
              ros__parameters:
                safety_override: "full"
                reflexes_enabled: false

        .. list-table::
            :header-rows: 1
            :widths: 35 10 10 45

            * - Parameter
              - Type
              - Interbotix Default Value
              - Notes
            * - /mobile_base/motion_control safety_override
              - String
              - ``"full"``
              - Turns off all safety features. See `Create® 3 Safety Overrides`_ for more information.
            * - /mobile_base/motion_control reflexes_enabled
              - Boolean
              - ``false``
              - Turns off all reflexes. See `Create® 3 Reflexes`_ for more information.

    .. group-tab:: ROS 2

        .. code-block:: yaml

            /locobot/mobile_base/motion_control:
              ros__parameters:
                safety_override: "full"
                reflexes_enabled: false

        .. list-table::
            :header-rows: 1
            :widths: 35 10 10 45

            * - Parameter
              - Type
              - Interbotix Default Value
              - Notes
            * - /locobot/mobile_base/motion_control safety_override
              - String
              - ``"full"``
              - Turns off all safety features. See `Create® 3 Safety Overrides`_ for more information.
            * - /locobot/mobile_base/motion_control reflexes_enabled
              - Boolean
              - ``false``
              - Turns off all reflexes. See `Create® 3 Reflexes`_ for more information.

.. _`Create® 3 Safety Overrides`: https://iroboteducation.github.io/create3_docs/api/safety/
.. _`Create® 3 Reflexes`: https://iroboteducation.github.io/create3_docs/api/reflexes/

See the `Create® 3's ROS 2 Parameters documentation`_ for a list of all possible configurable
parameters.

.. _`Create® 3's ROS 2 Parameters documentation`: https://iroboteducation.github.io/create3_docs/api/ros2/#ros-2-parameters
