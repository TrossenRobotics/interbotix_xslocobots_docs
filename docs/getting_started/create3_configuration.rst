=======================
Create® 3 Configuration
=======================

This page presents the default Interbotix configuration of the LoCoBot's Create® 3 base. This can
be found on the Create® 3 webserver accessed through the LoCoBot's computer's web browser (Firefox,
etc.) at ``192.168.186.2/ros-config``.

.. warning::

    Make sure that your Create® 3 base is configured to this default, otherwise it may not work
    properly or you may experience unexpected behavior.

.. note::

    If you modify any of these configurations, make sure to save the changes and restart the
    application for them to take effect.

Configuration
=============

Main configuration
------------------

.. list-table::
    :header-rows: 1
    :widths: 20 20 60

    * - Field
      - Interbotix Default Value
      - Notes
    * - ROS 2 Domain ID
      - ``0``
      - This value is typically left as the default if you do not have multiple ROS domains on the
        same network. See `ROS 2 Domain ID Documentation`_.
    * - ROS 2 Namespace
      - ``/mobile_base``
      - The value dictates the namespace under which all Create® 3 nodes, topics, services,
        actions, and parameters will be, i.e. the ``/tf`` topic published by the base will be
        ``/mobile_base/tf``.
    * - RMW_IMPLEMENTATION
      - ``rmw_fastrtps_cpp``
      - This value tells which RMW implementation to use. See `ROS 2 RMW Implementation
        Documentation`_.

.. _`ROS 2 Domain ID Documentation`: https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html
.. _`ROS 2 RMW Implementation Documentation`: https://docs.ros.org/en/foxy/Concepts/About-Different-Middleware-Vendors.html

Application ROS 2 Parameters File
---------------------------------

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

.. _`Create® 3 Safety Overrides`: https://iroboteducation.github.io/create3_docs/api/safety/
.. _`Create® 3 Reflexes`: https://iroboteducation.github.io/create3_docs/api/reflexes/

See the `Create® 3's ROS 2 Parameters documentation`_ for a list of all possible configurable
parameters.

.. _`Create® 3's ROS 2 Parameters documentation`: https://iroboteducation.github.io/create3_docs/api/ros2/#ros-2-parameters
