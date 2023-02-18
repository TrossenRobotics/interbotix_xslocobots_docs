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

    This section is under construction.

.. .. note::

..     This guide assumes that you have run the ROS 2 Software Setup for both your LoCoBot and remote
..     computers.

.. The ROS 2 Interface uses ``rmw_fastrtps_cpp`` as its RMW and requires a small amount of manual
.. configuration to get started. The process is as follows:

.. LoCoBot Computer
.. ----------------

.. 1.  Log into the LoCoBot's computer using SSH or by directly plug in a mouse and keyboard.

.. 2.  Open a terminal on your LoCoBot's computer by pressing :kbd:`Ctrl` + :kbd:`Alt` + :kbd:`T`.

.. 3.  Get the name of your wireless network connection with the command:

..     .. code-block:: console

..         $ ifconfig | grep wl | cut -d ":" -f1

..     .. note::

..         The name of wireless network interfaces typically start with the characters "wl". If the
..         command returns nothing, use the full ``ifconfig`` command and look for the Wireless
..         connection.

.. 4.  If from Trossen Robotics, or if the installation script was run for a ROS 2 installation, the
..     LoCoBot cyclonedds config file should be in the LoCoBot's home directory. Open it in an editor:

..     .. code-block:: console

..         $ nano ~/cyclonedds_config_locobot.xml

..     The config file will look something like:

..     .. code-block:: xml

..         <CycloneDDS
..             xmlns="https://cdds.io/config"
..             xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
..             xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
..             <Domain>
..                 <General>
..                     <NetworkInterfaceAddress>eno1,${WIRELESS_INTERFACE}</NetworkInterfaceAddress>
..                 </General>
..             </Domain>
..         </CycloneDDS>

.. 5.  Change ``${WIRELESS_INTERFACE}`` to the name of your wireless network interface. For example,
..     if your wireless network interface's name is ``wlan0``, the configuration file should look
..     like:

..     .. code-block:: xml

..         <CycloneDDS
..             xmlns="https://cdds.io/config"
..             xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
..             xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
..             <Domain>
..                 <General>
..                     <NetworkInterfaceAddress>eno1,wlan0</NetworkInterfaceAddress>
..                 </General>
..             </Domain>
..         </CycloneDDS>

.. 6.  Restart the ROS 2 daemon on your LoCoBot's computer (or restart the computer).

..     .. code-block::

..         $ ros2 daemon stop
..         The daemon has been stopped
..         $ ros2 daemon start
..         The daemon has been started

.. Remote Computer
.. ---------------

.. 1.  Open a terminal on your remote computer by pressing :kbd:`Ctrl` + :kbd:`Alt` + :kbd:`T`.

.. 2.  Get the name of your wireless network connection with the command:

..     .. code-block:: console

..         $ ifconfig | grep wl | cut -d ":" -f1

..     .. note::

..         The name of wireless network interfaces typically start with the characters "wl". If the
..         command returns nothing, use the full ``ifconfig`` command and look for the Wireless
..         connection.

.. 3.  If the remote software setup script was run specifying a ROS 2 installation, the remote
..     cyclonedds config file should be in your home directory. Open the cyclonedds config file in an
..     editor:

..     .. code-block:: console

..         $ nano ~/cyclonedds_config_remote.xml

..     The config file will look something like:

..     .. code-block:: xml

..         <CycloneDDS
..             xmlns="https://cdds.io/config"
..             xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
..             xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
..             <Domain>
..                 <General>
..                     <DontRoute>true</DontRoute>
..                     <NetworkInterfaceAddress>${WIRELESS_INTERFACE}</NetworkInterfaceAddress>
..                 </General>
..             </Domain>
..         </CycloneDDS>

.. 4.  Change ``${WIRELESS_INTERFACE}`` to the name of your wireless network interface. For example,
..     if your wireless network interface's name is ``wlan0``, the configuration file should look
..     like:

..     .. code-block:: xml

..         <CycloneDDS
..             xmlns="https://cdds.io/config"
..             xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
..             xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
..             <Domain>
..                 <General>
..                     <DontRoute>true</DontRoute>
..                     <NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress>
..                 </General>
..             </Domain>
..         </CycloneDDS>

.. 5.  Restart the ROS 2 daemon on your remote computer (or restart the computer).

..     .. code-block::

..         $ ros2 daemon stop
..         The daemon has been stopped
..         $ ros2 daemon start
..         The daemon has been started
