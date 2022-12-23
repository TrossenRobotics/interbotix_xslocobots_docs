===========
Basic Usage
===========

The Basic Usage Guide details the basics of using your LoCoBot including how to turn it on, how to
charge it, and how to develop remotely.

.. contents:: Contents
    :local:

Charging Your LoCoBot
=====================

.. tabs::

    .. group-tab:: Create® 3 Version


        Your Create® 3-base LoCoBot should come one AC adapter power supply and the Create® 3
        charging dock:

            -   The iRobot Create® 3 charging dock and its cable
            -   A 16.8V 2.5A output for the external battery

        1.  Plug both of these into wall power. Plug the external battery's power supply into the
            external battery.

        2.  Place the Create® 3 on its charging dock and wait for a chime to play. The LED status
            light on the base will indicate the charge level:

        .. list-table::
            :header-rows: 1
            :align: center

            * - Create® 3 Status LED Color
              - Create® 3 Charge Level
            * - Spinning White
              - Robot is booting up
            * - Partial White
              - Robot is charging, the solid arc of the ring indicates the charge level.
            * - Solid White
              - Robot is 100% charged
            * - Pulsing Red
              - Battery < 10%

        3.  Check the 4 LEDs next to the external battery's power button. They should light up
            according to its charge level:

        .. list-table::
            :header-rows: 1
            :align: center

            * - External Battery LED Status
              - Charge Level
            * - Flashing Single LED
              - Low Voltage
            * - Solid Single LED
              - 0%-25% Charge
            * - Solid Two LEDs
              - 25%-50% Charge
            * - Solid Three LEDs
              - 50%-75% Charge
            * - Solid Four LEDs
              - 75%-100% Charge

        4.  The Create® 3 should take about 1.5 hours to charge. If on, it will play a sound to
            indicate that it is fully charged.

        5.  The external battery should take about 6-8 hours to charge.

    .. group-tab:: Kobuki Version

        Your Kobuki-base LoCoBot should come with two AC adapter power supplies:

            -   A 19V 3.16A output for the Kobuki Base
            -   A 16.8V 2.5A output for the external battery

        1.  Plug both of these into wall power and their respective port. The ports are sized
            differently and it is impossible to plug in the wrong charger.

        2.  Turn on the Kobuki and check the Status LED on the back of the Kobuki base. It should
            light up according to its charge level.

        .. list-table::
            :header-rows: 1
            :align: center

            * - Kobuki Status LED Color
              - Kobuki Charge Level
            * - Solid Green
              - Fully Charged
            * - Blinking Green
              - Charging
            * - Orange
              - Low Charge

        3.  Check the 4 LEDs next to the external battery's power button. They should light up
            according to its charge level:

        .. list-table::
            :header-rows: 1
            :align: center

            * - External Battery LED Status
              - Charge Level
            * - Flashing Single LED
              - Low Voltage
            * - Solid Single LED
              - 0%-25% Charge
            * - Solid Two LEDs
              - 25%-50% Charge
            * - Solid Three LEDs
              - 50%-75% Charge
            * - Solid Four LEDs
              - 75%-100% Charge

        4.  The Kobuki should take about 1.5 hours to charge. If on, it will play a sound to
            indicate that it is fully charged.

        5.  The external battery should take about 6-8 hours to charge.

        .. note::

            You are able to use the Kobuki base while it is charging, though it is not recommended
            to move it around. The Kobuki base publishes data necessary for some ROS programs.

.. note::

    You are able to use the devices on the robot while the external battery is charging.
    This includes things like developing and running programs on the NUC, using the camera,
    and using the lidar.


Turning On Your LoCoBot
=======================

.. tabs::

    .. group-tab:: Create® 3 Version

        1.  Press the button on the side of the external battery. The LEDs next to the battery will
            light up, indicating the external battery's charge level. The battery will stay on
            while any connected device draws a load. Otherwise, it will automatically turn off
            after 30 seconds.

        2.  Press the power button on the side of the NUC to turn it on. The NUC's power button
            should light up blue.

        3.  Place the Create® 3 base on its charging dock and wait a few seconds for it to boot up.
            The Status LED will light up and the base will play a sound.

        4.  Connect a monitor, mouse, and keyboard to the NUC. It is okay to unplug any peripherals
            so you have enough ports for the necessary devices.

    .. group-tab:: Kobuki Version

        1.  Press the button on the side of the external battery. The LEDs next to the battery will
            light up, indicating the external battery's charge level. The battery will stay on
            while any connected device draws a load. Otherwise, it will automatically turn off
            after 30 seconds.

        2.  Press the power button on the side of the NUC to turn it on. The NUC's power button
            should light up blue.

        3.  Flick the power switch on the rear of the Kobuki base to On. The Status LED will light
            up and the base will play a sound.

        4.  Connect a monitor, mouse, and keyboard to the NUC. It is okay to unplug any peripherals
            so you have enough ports for the necessary devices.

Control Software Installation
=============================

Install the software that you plan to use, such as the ROS Interface. The quickstart commands for
it are below for a Create® 3 version LoCoBot running ROS 1 Noetic. See the :doc:`ROS 1 Interface
Software Setup <../ros_interface/ros1/software_setup>` or :doc:`ROS 2 Interface Software Setup
<../ros_interface/ros2/software_setup>` for more information.

    .. code:: console

        $ sudo apt install curl
        $ curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_rovers/main/interbotix_ros_xslocobots/install/amd64/xslocobot_amd64_install.sh' > xslocobot_amd64_install.sh
        $ chmod +x xslocobot_amd64_install.sh
        $ ./xslocobot_amd64_install.sh -b create3 -d noetic

Network Configuration
=====================

.. note::

    Terminology:

    - "Remote" - Your own personal computer (desktop, laptop, etc.)
    - "Robot" or "LoCoBot" - The NUC computer on the LoCoBot

LoCoBot Hostname
----------------

.. note::

    This section assumes that you are interacting with your LoCoBot's NUC computer using a monitor,
    mouse, and keyboard.

1.  Log into the ``locobot`` user on the LoCoBot using its default password, ``locobot``.

2.  Open a terminal by pressing :kbd:`Ctrl` + :kbd:`Alt` + :kbd:`T`.

3.  Type the command ``hostname`` to retrieve the robot's hostname. This is the domain name of the
    computer that you will use when you SSH into it. This is typically ``locobot``.

.. note::

    If you are using multiple LoCoBots, you should make each hostname unique, i.e. ``locobot1``,
    ``locobot2``, etc.

    .. code:: console

        $ hostnamectl set-hostname <unique_hostname>

4.  Connect the robot to the same WiFi that your remote computer is connected to.

5.  Ping the robot from your remote computer using the command below. If the robot's hostname is
    different than ``locobot``, use that instead. You should see output similar to below.

    .. code:: console

        $ ping -c 3 locobot.local
        PING locobot.local (192.168.0.171) 56(84) bytes of data.
        64 bytes from locobot (192.168.0.171): icmp_seq=1 ttl=64 time=203 ms
        64 bytes from locobot (192.168.0.171): icmp_seq=2 ttl=64 time=22.5 ms
        64 bytes from locobot (192.168.0.171): icmp_seq=3 ttl=64 time=351 ms

        --- locobot.local ping statistics ---
        3 packets transmitted, 3 received, 0% packet loss, time 2003ms
        rtt min/avg/max/mdev = 22.540/192.306/351.244/134.411 ms

RMW Configuration
-----------------

See the :doc:`RMW Configuration Guide <./rmw_configuration>` for details.

ROS Network Testing
===================

If using ROS, you should now test ROS communication between your LoCoBot and your remote computer
by running the basic talker/listener tutorial. This ensures that we have bi-directional
communication between the two machines.

.. tabs::

    .. tab:: ROS 1

        1.  If not already on both machines, install the rospy_tutorials package on the LoCoBot
            computer and your remote computer.

            .. code-block:: console

                $ sudo apt-get install ros-$ROS_DISTRO-rospy-tutorials

        2.  Start a roscore on your LoCoBot computer.

            .. code-block:: console

                # LoCoBot Computer
                $ roscore

        3.  Open a new terminal on the LoCoBot and run the listener script.

            .. code-block:: console

                # LoCoBot Computer
                $ rosrun rospy_tutorials listener.py

        4.  Open a new terminal on the remote computer and run the talker.

            .. code-block:: console

                # Remote Computer
                $ rosrun rospy_tutorials talker.py

        5.  You should see something like the output below on the terminal on the LoCoBot running
            the listener.

            .. code-block::

                # LoCoBot Computer
                [INFO] [1666885421.836186]: /listener_1841_1666885406149I heard hello world 1666885421.8021505
                [INFO] [1666885421.934443]: /listener_1841_1666885406149I heard hello world 1666885421.9020953
                [INFO] [1666885422.034559]: /listener_1841_1666885406149I heard hello world 1666885422.0021284
                [INFO] [1666885422.134659]: /listener_1841_1666885406149I heard hello world 1666885422.1021016
                [INFO] [1666885422.233945]: /listener_1841_1666885406149I heard hello world 1666885422.2021453

        6.  End the talker and listener processes using :kbd:`Ctrl` + :kbd:`C` in their respective
            terminals.

        7.  Run the listener script on your remote computer.

            .. code-block:: console

                # Remote Computer
                $ rosrun rospy_tutorials listener.py

        8.  Run the talker on your LoCoBot computer.

            .. code-block:: console

                # LoCoBot Computer
                $ rosrun rospy_tutorials talker.py

        9.  You should see something like the output below on the terminal on the remote computer
            running the listener.

            .. code-block::

                # Remote Computer
                [INFO] [1666885695.850287]: /listener_17518_1666885691489I heard hello world 1666885695.8616695
                [INFO] [1666885695.950195]: /listener_17518_1666885691489I heard hello world 1666885695.9616487
                [INFO] [1666885696.050392]: /listener_17518_1666885691489I heard hello world 1666885696.061647
                [INFO] [1666885696.150613]: /listener_17518_1666885691489I heard hello world 1666885696.1614935
                [INFO] [1666885696.250500]: /listener_17518_1666885691489I heard hello world 1666885696.2614782

        10. End the talker and listener processes using :kbd:`Ctrl` + :kbd:`C` in their respective
            terminals.

    .. tab:: ROS 2

        1.  If not already on both machines, install the example minimal publisher and minimal
            subscriber on the LoCoBot computer and your remote computer.

            .. code-block:: console

                $ sudo apt-get install ros-$ROS_DISTRO-examples-rclpy-minimal-publisher ros-$ROS_DISTRO-examples-rclpy-minimal-subscriber

        2.  Open a new terminal on the LoCoBot and run the minimal subscriber.

            .. code-block:: console

                # LoCoBot Computer
                $ ros2 run examples_rclpy_minimal_subscriber subscriber_member_function

        3.  Open a new terminal on the remote computer and run the minimal publisher.

            .. code-block:: console

                # Remote Computer
                $ ros2 run examples_rclpy_minimal_publisher publisher_local_function

        4.  You should see something like the output below on the terminal on the LoCoBot running
            the subscriber.

            .. code-block::

                # LoCoBot Computer
                [INFO] [minimal_subscriber]: I heard: "Hello World: 0"
                [INFO] [minimal_subscriber]: I heard: "Hello World: 1"
                [INFO] [minimal_subscriber]: I heard: "Hello World: 2"
                [INFO] [minimal_subscriber]: I heard: "Hello World: 3"
                [INFO] [minimal_subscriber]: I heard: "Hello World: 4"

        5.  End the publisher and subscriber processes using :kbd:`Ctrl` + :kbd:`C` in their
            respective terminals.

        6.  Run the subscriber script on your remote computer.

            .. code-block:: console

                # Remote Computer
                $ ros2 run examples_rclpy_minimal_subscriber subscriber_member_function

        7.  Run the publisher on your LoCoBot computer.

            .. code-block:: console

                # LoCoBot Computer
                $ ros2 run examples_rclpy_minimal_publisher publisher_local_function

        8.  You should see something like the output below on the terminal on the remote computer
            running the subscriber.

            .. code-block::

                # Remote Computer
                [INFO] [minimal_subscriber]: I heard: "Hello World: 0"
                [INFO] [minimal_subscriber]: I heard: "Hello World: 1"
                [INFO] [minimal_subscriber]: I heard: "Hello World: 2"
                [INFO] [minimal_subscriber]: I heard: "Hello World: 3"
                [INFO] [minimal_subscriber]: I heard: "Hello World: 4"

        9.  End the publisher and subscriber processes using :kbd:`Ctrl` + :kbd:`C` in their
            respective terminals.

.. note::

    If you run into any ROS network related issue, see our :doc:`Troubleshooting
    Guide<../troubleshooting>` for potential solutions.

Remote Development
==================

.. note::

    Terminology:

    -   "Remote" - Your own personal computer (desktop, laptop, etc.)
    -   "Robot" or "LoCoBot" - The NUC computer on the LoCoBot

General Remote Development
--------------------------

.. _ssh-into-the-locobot:

SSH Into the LoCoBot
~~~~~~~~~~~~~~~~~~~~

1.  On your remote computer, install the OpenSSH Client software to allow for a secure shell
    connection between your remote computer and the LoCoBot computer.

    .. code:: console

        $ sudo apt install openssh-client

2.  SSH into the LoCoBot from your remote computer with the display forwarding flag ``-X``. If the
    robot's username or hostname are different than ``locobot``, use that instead.

    .. code:: console

        # ssh -X username@hostname.local
        $ ssh -X locobot@locobot.local

    .. note::

        The ``-X`` flag indicates to OpenSSH that we want to do display forwarding. This means that
        OpenSSH will forward graphical application to the client from the server.

3.  If prompted, enter the password ``locobot`` and accept the SSH key.

4.  Once logged in to the LoCoBot, you can open multiple SSH'ed terminals using the command below.

    .. code:: console

        $ gnome-terminal &

    -  Sometimes, this doesn't work. In that case, use the command from `this Ask Ubuntu answer`_.

        .. code:: console

            $ /usr/bin/dbus-launch /usr/bin/gnome-terminal &

.. _`this Ask Ubuntu answer`: https://askubuntu.com/questions/608330/problem-with-gnome-terminal-on-gnome-3-12-2/1235679#1235679

VSCode Remote Development
~~~~~~~~~~~~~~~~~~~~~~~~~

1.  At Trossen Robotics, we use Microsoft's VSCode and its Remote - SSH extension (also developed
    by Microsoft) for simple remote development on the LoCoBot.

   -   `Install VSCode`_ for Ubuntu.

   -    Open VSCode, Press :kbd:`Ctrl` + :kbd:`P` to launch the Quick Open Menu, and run the
        following command.

    .. code::

        ext install ms-vscode-remote.remote-ssh

.. _`Install VSCode`: https://code.visualstudio.com/download

2.  In VSCode, press **F1** and run the ``Remote-SSH: Open SSH Host...`` command and select the
    ``Add New SSH Host`` option. Enter the same ``username@hostname.local`` combination you used
    when opening the SSH connection between your remote computer and the LoCoBot like ``ssh
    locobot@locobot.local``. If prompted, enter the password ``locobot``.

3.  Once connected, use **File > Open Folder**, and select the directory you wish to operate in,
    i.e. the ``~/interbotix_ws`` directory if using the ROS Interface.

4.  Your instance of VSCode is now attached to the LoCoBot and is open to your development
    workspace.

5.  You can open terminals in VSCode by pressing :kbd:`Ctrl` + :kbd:`Shift` + :kbd:`\`` or by using
    **Terminal > New Terminal**.

.. note::

   It is not simple to configure display forwarding using the Remote-SSH extension at the time of
   writing this guide. To get around this, you can either follow some of the recommendations in
   `this GitHub Issue`_, or just `ssh into the locobot`_ to launch programs with GUIs.

.. _`this GitHub Issue`: https://github.com/microsoft/vscode-remote-release/issues/267
.. _`ssh into the locobot`: `ssh-into-the-locobot`_

ROS Interface Development
-------------------------

If using the ROS Interface, we provide a remote workspace installation script. See the below
commands for the remote installation quickstart commands for a Create® 3 version LoCoBot running
ROS 1 Noetic. Run these commands on your remote computer.

.. code:: console

    $ sudo apt install curl
    $ curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_rovers/main/interbotix_ros_xslocobots/install/xslocobot_remote_install.sh' > xslocobot_remote_install.sh
    $ chmod +x xslocobot_remote_install.sh
    $ ./xslocobot_remote_install.sh -b create3 -d noetic

Alternatively, you can check the :ref:`Remote Install section of the ROS Interface Software Setup
<ros-software-setup-remote-install-label>` for more information.

.. _basic-usage-turning-off-your-locobot-label:

Turning Off Your LoCoBot
========================

.. tabs::

    .. group-tab:: Create® 3 Version

        1.  It is a good idea to cleanly turn off the NUC when you are finishing using it. To do
            this, type ``sudo poweroff`` in its terminal and enter your password.

            .. note::

                This will kill any program currently running on the NUC. Make sure the arm is in
                its cradle and that ending programs will not harm the robot.

        2.  Press and hold the external battery's power button for two (2) seconds until the LEDs
            power off.

        3.  Hold the base's center button (marked with a power symbol) for about seven (7) seconds
            until it plays a chime and the LEDs behind the button turn off.

    .. group-tab:: Kobuki Version

        1.  It is a good idea to cleanly turn off the NUC when you are finishing using it. To do
            this, type ``sudo poweroff`` in its terminal and enter your password.

            .. note::

                This will kill any program currently running on the NUC. Make sure the arm is in
                its cradle and that ending programs will not harm the robot.

        2.  Press and hold the external battery's power button for two (2) seconds until the LEDs
            power off.

        3.  Flick the switch at the back of the Kobuki to power it off. The Status LED should turn
            off.

Video Tutorials
===============

Create® 3 Version Basic Usage Guide
-----------------------------------

.. youtube:: S7ciWI7d67Q
    :width: 70%
    :align: center
