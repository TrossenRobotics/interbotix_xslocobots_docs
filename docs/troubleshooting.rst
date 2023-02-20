===============
Troubleshooting
===============

This guide walks a user through possible issues that can occur when using an Interbotix LoCoBot and
how to fix them. If, after following this guide, the issues persists, take a look at the
`interbotix_ros_rovers bug tracker`_. If you can't find an answer there, feel free to `open your
own issue`_.

.. note::

    For troubleshooting tips on the mobile X-Series Arm or the pan-and-tilt camera mount, see the
    `X-Series Arms Troubleshooting documentation`_.


.. _`interbotix_ros_rovers bug tracker`: https://github.com/Interbotix/interbotix_ros_rovers/issues?q=is%3Aissue
.. _`open your own issue`: https://github.com/Interbotix/interbotix_ros_rovers/issues/new/choose
.. _`X-Series Arms Troubleshooting documentation`: https://www.trossenrobotics.com/docs/interbotix_xsarms/troubleshooting/index.html

Common Issues
=============

Create® 3 base stutters or reflexed are triggered when driven
-------------------------------------------------------------

If the robot stops, turns, or reacts in an unexpected way when moving, it's possible that the
reflexes are not turned off on the Create® 3 base. To fix this, you can either set the
``<base_namespace>/motion_control reflexes_enabled`` parameter to ``False`` using the ``ros2 param
set`` command line tool, or set it to ``false`` in the Application ROS 2 Parameters File located in
the Create® 3 webserver's ROS 2 Configuration page. If using Interbotix's default network
configuration, this will be at ``192.168.186.2/ros-config``, assuming that the Create® 3 is turned
on. See the :doc:`Create® 3 Configuration page<../getting_started/create3_configuration>` for more
information.

Less Common Issues
==================

Create® 3 base clock is not synchronized
----------------------------------------

If transforms require a significant extrapolation into the past, it is possible that your Create® 3
base's clock is not in sync with the rest of your system. Errors like the one below may indicate
that this is the case.

.. container:: code-wrap

    .. code-block::

        Error="Lookup would require extrapolation -27793617.761847734s into the future. Requested time 1667324891.111343861 but the latest data is at time 1639531273.349496126, when looking up transform from frame [locobot/base_footprint] to frame [locobot/odom]. canTransform returned after 0.200389 timeout was 0.2."

To solve this, check iRobot's `Setup NTP on compute board to serve time to Create® 3`_ guide. In
short, the steps are as follows:

1.  On your LoCoBot's computer, install chrony NTP server package

    .. code-block:: console

        $ sudo apt install chrony

2.  Edit the config file

    .. code-block:: console

        $ sudo nano /etc/chrony/chrony.conf

    .. note::

        To close nano with your changes, press :kbd:`Ctrl` + :kbd:`S` to save, and :kbd:`Ctrl` +
        :kbd:`X` to exit.

3.  Add the following lines after the ``pool #.ubuntu.pool.ntp.org iburst maxsources #`` block

    .. code-block::

        server 192.168.186.2 presend 0 minpoll 0 maxpoll 0 iburst  prefer trust
        # Enable serving time to ntp clients on 192.168.186.0 subnet.
        allow 192.168.186.0/24

4.  Optionally add the following lines immediately afterward if your LoCoBot's computer will not
    have a connection to a reference clock (i.e., the Internet)

    .. code-block::

        # Serve time even if not synchronized to a time source
        local stratum 10

5.  Restart chrony

    .. code-block:: console

        $ sudo service chrony restart

6.  Verify compute NTP server is talking to the Create® 3

    .. code-block:: console

        $ sudo chronyc clients

7.  Confirm ``192.168.182.2`` shows non 0 number in NTP column

    .. code-block::

        Hostname                      NTP   Drop Int IntL Last     Cmd   Drop Int  Last
        ===============================================================================
        192.168.186.2                  51      0   5   -    32       0      0   -     -
        localhost                       0      0   -   -     -      31      0   7     4

8.  Note that if there is a large jump in the time, the Create® 3 may not accept it until its next
    reboot. This can be verified by checking the Create® 3 robot's log for a line like

    .. code-block::

        user.notice ntpd: ntpd: reply from 192.168.186.3: delay ### is too high, ignoring

    If this happens, simply restart the base (not just the application) via the webserver over the
    USB network connection.

.. _`Setup NTP on compute board to serve time to Create® 3`: https://iroboteducation.github.io/create3_docs/setup/compute-ntp/

.. note::

    Sometimes it helps to disconnect both the Create® 3 and the LoCoBot's computer from the
    Internet and just have them run on an isolated network.

.. note::

    As of Create® 3 firmware versions G.4.3 and H.1.0, you can use its webserver to restart the
    ntpd from the Beta menu. This attempts to force the base to resync its clock. Because this is a
    beta feature, it may not be stable and improper use may result in an inoperable robot. See
    `Restart ntpd`_ for details.

.. _`Restart ntpd`: https://iroboteducation.github.io/create3_docs/webserver/restart-ntpd/

.. _troubleshooting-cant-read-topics-label:

Can't see/echo topics published by LoCoBot on remote using ROS 1
----------------------------------------------------------------

Your network may be configured incorrectly. See the `ROS Multiple Machines Tutorial`_ and the `ROS
Network Setup Tutorial`_. This may be as simple as changing the value of the ``ROS_IP`` environment
variable. For example, your ``ROS_IP`` may be set to the value of the IP assigned in the connection
to the Create® 3 base via the Ethernet connection, ``192.168.186.3``. To fix this, you can
explicitly assign the variable to the IP assigned in the connection to the wireless network through
the following steps:

1.  Find all IP addresses assigned to your LoCoBot computer. You should get two addresses: one for
    the connection over the Ethernet network, and one for the connection over the wireless network.

    .. code-block:: console

        $ hostname -I
        192.168.186.3 192.168.0.171

    If properly connected to the Ethernet network, that IP address will be ``192.168.186.3``. The
    address for the wireless network will be the other one, in this case, ``192.168.0.171``.

2.  Edit the LoCoBot's .bashrc file

    .. code-block:: console

        $ nano ~/.bashrc

3.  Find the ``ROS_IP`` section of the Interbotix Configurations block

    .. code-block:: bash

        export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)
        if [ -z "$ROS_IP" ]; then
               export ROS_IP=127.0.0.1
        fi

4.  Comment this block out and add a line exporting the IP address for the wireless network

    .. code-block:: bash

        #export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)
        #if [ -z "$ROS_IP" ]; then
        #       export ROS_IP=127.0.0.1
        #fi
        export ROS_IP=192.168.0.171

    .. note::

        To close nano with your changes, press :kbd:`Ctrl` + :kbd:`S` to save, and :kbd:`Ctrl` +
        :kbd:`X` to exit.

5.  Source your .bashrc file

    .. code-block:: console

        $ source ~/.bashrc

6.  Your ``ROS_IP`` environment variable should now be properly assigned.

    .. code-block:: console

        $ echo $ROS_IP
        192.168.0.171

.. _`ROS Multiple Machines Tutorial`: http://wiki.ros.org/ROS/Tutorials/MultipleMachines
.. _`ROS Network Setup Tutorial`: http://wiki.ros.org/ROS/NetworkSetup

Can't see RViz displays on remote view using ROS 1
--------------------------------------------------

See :ref:`Can't see/echo topics published by LoCoBot on
remote<troubleshooting-cant-read-topics-label>`.

``run_id on parameter server does not match declared run_id`` when launching on remote using ROS 1
--------------------------------------------------------------------------------------------------

This occurs because roslaunch can't detect the roscore before starting up, leading to the run_id
mismatch. To solve this, simply append the ``--wait`` argument when running roslaunch. This tells
roslaunch to delay the launch until a roscore is detected.

.. code-block:: console

    $ roslaunch interbotix_xslocobot_descriptions remote_view.launch --wait
    #                                                                ^^^^^^
