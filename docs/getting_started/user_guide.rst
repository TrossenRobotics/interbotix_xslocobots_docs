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
