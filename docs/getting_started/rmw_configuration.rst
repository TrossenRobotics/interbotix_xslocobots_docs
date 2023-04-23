=================
RMW Configuration
=================

This guide details how to configure the RMW implementation for the first time if using the ROS
Interface.

ROS 1 Interface & Create® 3
===========================

There is no need to configure the RMW on ROS 1. This is because the ROS 1 Interface uses
``rmw_fastrtps_cpp`` which is great at working with multi-device networks. We also use ros1_bridge
on the LoCoBot's computer which allows the remote computer to see the ROS 2 topics/services/actions
from the Create® 3.

ROS 2 Interface & Create® 3
===========================

.. note::

    This guide assumes that you have run the :doc:`ROS 2 Software Setup
    <../ros_interface/ros2/software_setup>` for both your LoCoBot and remote computers.

The ROS 2 Interface uses ``rmw_fastrtps_cpp`` as its RMW and the `Fast-DDS Discovery Server`_.

.. _`Fast-DDS Discovery Server`: https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html

Create® 3
---------

The Create® 3 has the following modifications:

*   RMW_IMPLEMENTATION is set to ``rmw_fastrtps_cpp``.
*   Fast DDS discovery server is enabled.
*   Address and port of Fast DDS discovery server are set to ``192.168.186.3:11811``.

You can find a list of all modifications the :ref:`Create 3 Software Setup's
<label-create3-configuration-main-configuration>` ROS 2 Main Configuration.

LoCoBot Computer
----------------

.. note::

    These modifications are done by Trossen Robotics and the user does not have to worry about
    modifying these unless they are setting up a robot themselves, or if they are dealing with a
    unique network setup.

The LoCoBot computer has the following modifications:

*   The ``RMW_IMPLEMENTATION`` environment variable is set to ``rmw_fastrtps_cpp``.

    .. code:: bash

        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

*   The ``ROS_DISCOVERY_SERVER`` environment variable is set to ``127.0.0.1:11811``, the localhost
    address using port ``11811``.

    .. code:: bash

        export ROS_DISCOVERY_SERVER=127.0.0.1:11811

*   All participants are configured to act as a Super Client via the XML configuration file,
    setting the UDP locator to address ``127.0.0.1`` and port ``11811``.

    .. code:: xml

        <?xml version="1.0" encoding="UTF-8" ?>
        <dds>
            <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
                <participant profile_name="super_client_profile" is_default_profile="true">
                    <rtps>
                        <builtin>
                            <discovery_config>
                                <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
                                <discoveryServersList>
                                    <RemoteServer prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41">
                                        <metatrafficUnicastLocatorList>
                                            <locator>
                                                <udpv4>
                                                    <address>127.0.0.1</address>
                                                    <port>11811</port>
                                                </udpv4>
                                            </locator>
                                        </metatrafficUnicastLocatorList>
                                    </RemoteServer>
                                </discoveryServersList>
                            </discovery_config>
                        </builtin>
                    </rtps>
                </participant>
            </profiles>
        </dds>

*   The ``FASTRTPS_DEFAULT_PROFILES_FILE`` environment variable is set to the location of the
    Fast-DDS XML configuration file. By default, this is at
    ``~/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/install/resources/super_client_configuration_file.xml``.

    .. code:: bash

        export FASTRTPS_DEFAULT_PROFILES_FILE=~/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/install/resources/super_client_configuration_file.xml

*   IP forwarding is enabled by setting the value of ``net.ipv4.ip_forward=1`` in
    ``/etc/sysctl.conf``. See `this guide`_ on IP forwarding from OpenVPN for more information.
*   Fast-DDS Discovery Server running as service ``fastdds_disc_server.service`` at startup.

    .. code-block:: bash

        #!/bin/bash
        source /opt/ros/${ROS_DISTRO}/setup.bash
        fastdds discovery -i 0 &
        exit 0

    *   The status of this service can be checked with the command:

        .. code-block:: bash

            $ systemctl status fastdds_disc_server.service

            ● fastdds_disc_server.service - FastDDS discovery server
                Loaded: loaded (/lib/systemd/system/fastdds_disc_server.service; enabled; vendor preset: enabled)
                Active: active (running)
                Process: 1349 ExecStart=/bin/bash -e /home/locobot/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/install/resources/service/fastdds_disc_server.sh (code=exited, status=0/SUCCESS)
            Main PID: 1393 (bash)
                Tasks: 10 (limit: 9105)
                Memory: 13.8M
                CGroup: /system.slice/fastdds_disc_server.service
                        ├─1393 /bin/bash -e /home/locobot/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/install/resources/service/fastdds_disc_server.sh
                        ├─1395 python3 /opt/ros/galactic/bin/../tools/fastdds/fastdds.py discovery -i 0
                        └─1397 /opt/ros/galactic/bin/fast-discovery-server -i 0

                            locobot systemd[1]: Starting FastDDS discovery server...
                            locobot systemd[1]: Started FastDDS discovery server.
                            locobot bash[1397]: ### Server is running ###
                            locobot bash[1397]:   Participant Type:   SERVER
                            locobot bash[1397]:   Server ID:          0
                            locobot bash[1397]:   Server GUID prefix: 44.53.00.5f.45.50.52.4f.53.49.4d.41
                            locobot bash[1397]:   Server Addresses:   UDPv4:[0.0.0.0]:11811

.. _`this guide`: https://openvpn.net/faq/what-is-and-how-do-i-enable-ip-forwarding-on-linux/

Remote Computer
---------------

The remote computer has the following modifications, done by the remote installation script:

*   The ``RMW_IMPLEMENTATION`` environment variable is set to ``rmw_fastrtps_cpp``.

    .. code:: bash

        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

*   The ``ROS_DISCOVERY_SERVER`` environment variable is set to the LoCoBot's IP address and port
    ``11811``.

    .. code:: bash

        export ROS_DISCOVERY_SERVER=${LOCOBOT_IP}:11811

*   All participants are configured to act as a Super Client via the XML configuration file,
    setting the UDP locator to the address of the LoCoBot's eno1 interface, ``192.168.186.3``, and
    port ``11811``.

    .. code:: xml

        <?xml version="1.0" encoding="UTF-8" ?>
        <dds>
            <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
                <participant profile_name="super_client_profile" is_default_profile="true">
                    <rtps>
                        <builtin>
                            <discovery_config>
                                <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
                                <discoveryServersList>
                                    <RemoteServer prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41">
                                        <metatrafficUnicastLocatorList>
                                            <locator>
                                                <udpv4>
                                                    <address>192.168.186.3</address>
                                                    <port>11811</port>
                                                </udpv4>
                                            </locator>
                                        </metatrafficUnicastLocatorList>
                                    </RemoteServer>
                                </discoveryServersList>
                            </discovery_config>
                        </builtin>
                    </rtps>
                </participant>
            </profiles>
        </dds>

*   The ``FASTRTPS_DEFAULT_PROFILES_FILE`` environment variable is set to the location of the
    Fast-DDS XML configuration file. By default, this is
    ``~/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/install/resources/super_client_configuration_file.xml``.

    .. code:: bash

        export FASTRTPS_DEFAULT_PROFILES_FILE=~/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/install/resources/super_client_configuration_file.xml

*   A route is added to IP subnet ``192.168.186.0/24`` via the LoCoBot's wireless network interface's
    IP address via a service running on startup.

    .. code:: bash

        ip route add 192.168.186.0/24 via ${LOCOBOT_IP}
