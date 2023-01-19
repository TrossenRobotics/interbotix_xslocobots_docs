=============================
ROS 1 Standard Software Setup
=============================

.. contents::
    :local:

Compatible Models
=================

The ROS Interface packages can be used with any of the Interbotix LoCoBot kits listed below. Next
to each name is the codename used to describe it in software (specifically for the ``robot_model``
argument in launch files) and a description. There are up to four parts in a name. The first word
``locobot`` specifies that the robot is a type of rover. The next two letters represent model type
(ex. ``wx`` for 'WidowX'). The number afterwards (ex. ``200``) corresponds to the length of both
the arm's forearm and upper-arm links in millimeters. Finally, the ``s`` after some numbers
signifies if that arm has six degrees of freedom. If the robot has ``base`` in it, that means that
it has no arm.

.. list-table:: **Models**
    :align: center
    :header-rows: 1
    :widths: 20 20 10

    * - Model Name and Store Link
      - Robot Documentation
      - Codename
    * - `LoCoBot Base`_
      - :doc:`LoCoBot Base Documentation <../../specifications/locobot_base>`
      - ``locobot_base``
    * - `LoCoBot PX100`_
      - :doc:`LoCoBot PX100 Documentation <../../specifications/locobot_px100>`
      - ``locobot_px100``
    * - `LoCoBot WX200`_
      - :doc:`LoCoBot WX200 Documentation <../../specifications/locobot_wx200>`
      - ``locobot_wx200``
    * - `LoCoBot WX250 6DOF`_
      - :doc:`LoCoBot WX250 6DOF Documentation <../../specifications/locobot_wx250s>`
      - ``locobot_wx250s``

.. _`LoCoBot Base`: https://www.trossenrobotics.com/locobot-base.aspx
.. _`LoCoBot PX100`: https://www.trossenrobotics.com/locobot-px100.aspx
.. _`LoCoBot WX200`: https://www.trossenrobotics.com/locobot-wx200.aspx
.. _`LoCoBot WX250 6DOF`: https://www.trossenrobotics.com/locobot-wx250-6-degree-of-freedom.aspx

-   **LoCoBot Base Robot Rover** (``locobot_base``): equipped with a :ref:`Kobuki
    <specifications-kobuki-label>` or :ref:`Create® 3<specifications-create3-label>` mobile base, a
    :ref:`RealSense D435 <specifications-realsense-label>` Depth camera on a :ref:`DYNAMIXEL
    pan-tilt servo <specifications-pan-and-tilt-label>`, and a :ref:`NUC
    <specifications-nuc-label>` Intel Computer, this rover is ready to perform some serious
    navigation and mapping tasks. For even more flexibility, the :ref:`RPLidar A2M8
    <specifications-rplidar-label>` laser scanner can be added as well.
-   **LoCoBot PincherX 100 Robot Rover** (``locobot_px100``): with all the features of the
    locobot_base platform, this robot steps it up a notch by including the 4DOF :ref:`PincherX-100
    <specifications-px100-label>` Interbotix Arm. Now officially a mobile-manipulator, this rover
    can perform manipulation tasks in addition to navigating/mapping an environment.
-   **LoCoBot WidowX 200 Robot Rover** (``locobot_wx200``): similar in structure to the
    locobot_px100 robot, this platform substitutes the PincherX 100 arm with the 5DOF
    :ref:`WidowX-200 <specifications-wx200-label>` Interbotix Arm. With longer range, a higher
    payload capacity, and an extra degree of freedom, this rover makes your manipulation tasks
    easier to perform.
-   **LoCoBot WidowX 250 6DOF Robot Rover** (``locobot_wx250s``): similar in structure to the
    locobot_wx200 rover, this platform substitutes the WidowX 200 arm with the 6DOF
    :ref:`WidowX-250 6DOF <specifications-wx250s-label>` Interbotix Arm. With even longer range, a
    higher payload capacity, and yet another degree of freedom, this platform raises the bar on
    research mobile-manipulators.

Requirements
============

Below is a list of the hardware you will need to get started:

-   Keyboard, mouse, HDMI monitor, and HDMI cable
-   One of the X-Series LoCoBot Kits mentioned above
-   Computer running Ubuntu Linux 18.04 or 20.04

Software Installation
=====================

.. note::

    Terminology:

    -   "Remote" - Your own personal computer (desktop, laptop, etc.)
    -   "Robot" or "LoCoBot" - The NUC computer on the LoCoBot

To get all the code setup, refer to the computer platform types below (currently only one option,
but this may change in the future) and run the appropriate installation script. Afterwards,
continue with the :ref:`Installation Checks <ros-software-setup-installation-checks-label>`
sub-section.

AMD64 Architecture
------------------

.. attention::

    If you purchased a NUC-based ROS LoCoBot from Trossen Robotics or their distributors, note that
    it will come pre-installed with all required software already on it and you should **not** have
    to run the software installation yourself.

.. collapse:: Click here to reveal installation steps, acknowledging the note above.

    After powering on the NUC, a login screen should appear with ``locobot`` as the user name.
    Conveniently, the password is the same as the user name so type ``locobot`` and hit **Enter**.
    Next, update the computer by performing the following steps.

    1.  Connect to the Internet. This is as simple as clicking the Wifi icon on the top right of
        the Desktop and selecting your desired network.

    2.  Press :kbd:`Ctrl` + :kbd:`Alt` + :kbd:`T` to open a terminal screen, and type ``sudo apt
        update``.

    3.  After the process completes, type ``sudo apt -y upgrade``. It might take a few minutes for
        the computer to upgrade.

    4.  Finally, type ``sudo apt -y autoremove`` to get rid of unnecessary software packages. Then
        close out of the terminal and reboot the computer.

    5.  Once rebooted, login and open up a terminal as before. Instead of manually installing all
        the software needed for the robot, you will download and run an installation script. Follow
        the commands below to get started! Note that no sensors or actuators (i.e. camera, lidar,
        U2D2, etc.) needs to be connected to the computer for the installation to work. Specify the
        version of ROS that you want to install using the ``-d`` flag followed by the
        distribution's codename. Alternatively, you can run it without the ``-d`` flag and the
        script will install packages for the ROS 1 distribution supported by the version of Ubuntu,
        or the latest stable release of ROS 2 if using Ubuntu version 22.04 or later. See the `list
        of currently supported distributions`_. You will also need to specify the base type using
        the ``-b`` flag followed by ``create3`` if using the Create® 3 base, or ``kobuki`` if using
        the Kobuki base. The commands below demonstrate the process of running the installation
        script for ROS 1 Noetic and a Create® 3 base.

        .. code-block:: console

            $ sudo apt install curl
            $ curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_rovers/main/interbotix_ros_xslocobots/install/amd64/xslocobot_amd64_install.sh' > xslocobot_amd64_install.sh
            $ chmod +x xslocobot_amd64_install.sh
            $ ./xslocobot_amd64_install.sh -d noetic -b create3

        .. note::

            The install script provides more in-depth control of some installation options. Append
            the ``-h`` flag to see the help document like below:

            .. code-block:: console

                $ ./xslocobot_amd64_install.sh -h
                USAGE: ./xslocobot_amd64_install.sh [-h][-d DISTRO][-p PATH][-b BASE_TYPE][-n]

                ...

    6.  Once the script is done, shutdown the computer, and remove the HDMI cable, keyboard, and
        mouse. Replug any sensors into the computer that were unplugged initially. Then turn the
        computer on again by pressing the power button.

.. _`list of currently supported distributions`: https://github.com/Interbotix/interbotix_ros_rovers/security/policy#supported-versions

.. _ros-software-setup-remote-install-label:

Remote Install
--------------

For some robotic projects, you may want to run your robot in a 'headless' state on some computer
(like a NUC or Raspberry Pi), and monitor the robot's state (in RViz for example) on your personal
(a.k.a remote) computer over a local network. For this to work, run the installation script below
on your personal computer running Linux Ubuntu 18.04 or 20.04. As an FYI, the script will prompt
you to insert the hostname of the robot (NOT the remote) computer. As an example, if you wanted to
monitor the state of a NUC-based locobot, you would set the hostname to ``locobot``. To find out
the hostname of the robot computer, just open a terminal and type ``hostname``. Specify the version
of ROS that you want to install using the ``-d`` flag followed by the distribution's codename.
Alternatively, you can run it without the ``-d`` flag and the script will install packages for the
ROS 1 distribution supported by the version of Ubuntu, or the latest stable release of ROS 2 if
using Ubuntu version 22.04 or later. See the `list of currently supported distributions`_. You will
also need to specify the base type using the ``-b`` flag followed by ``create3`` if using the
Create® 3 base, or ``kobuki`` if using the Kobuki base. The commands below demonstrate the process
of running the installation script for ROS 1 Noetic and a Create® 3 base.

.. note::

    ROS and RViz must already be installed on your local machine for the remote install to be
    successful.

.. code-block:: console

    $ sudo apt install curl
    $ curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_rovers/main/interbotix_ros_xslocobots/install/xslocobot_remote_install.sh' > xslocobot_remote_install.sh
    $ chmod +x xslocobot_remote_install.sh
    $ ./xslocobot_remote_install.sh -d noetic -b create3

.. note::

    The install script provides more in-depth control of some installation options. Append the
    ``-h`` flag to see the help document like below:

    .. code-block:: console

        $ ./xslocobot_amd64_install.sh -h
        USAGE: ./xslocobot_remote_install.sh [-h][-d DISTRO][-p PATH][-b BASE_TYPE][-r HOSTNAME]

        ...

Be aware that the installation script will export the ``ROS_MASTER_URI`` environment variable in
your personal computer's ``~/.bashrc`` file to ``http://<hostname>.local:11311``. Make sure to
comment out this line when done monitoring or your personal computer will complain about not being
able to find its ROS Master.

To SSH from your remote to the robot computer, first connect your personal Linux computer to the
same network to which the locobot is connected. Then open a terminal and SSH into the locobot by
typing (assuming a NUC-based locobot)...

.. code-block:: console

    $ ssh -X locobot@locobot.local

You will be prompted for a password - just type ``locobot`` and you should be in!

The ``-X`` flag in the command above allows window forwarding. This means that it's possible to
open small graphical applications on the locobot computer which will be forwarded to your personal
computer. Let's open the terminal application by...

.. code-block:: console

    $ gnome-terminal &

.. note::

    Sometimes the command above doesn't work to open new terminals. An alternate solution is to use
    the command found in `this StackExchange answer`_:

    .. code-block:: console

        $ /usr/bin/dbus-launch /usr/bin/gnome-terminal &

.. _`this StackExchange answer`: https://askubuntu.com/questions/608330/problem-with-gnome-terminal-on-gnome-3-12-2/1235679#1235679

Now, we can open up new terminals (via :kbd:`Ctrl` + :kbd:`Alt` + :kbd:`T`) on the LoCoBot computer
without having to SSH each time. Note that unless otherwise stated, all the following commands
should be executed in the new terminal window that pops up.

.. _ros1-software-setup-installation-checks-label:

Installation Checks
===================

.. note::

    These sensors should be plugged back in and turned on at this point if they're not already.

After running the installation script on the robot computer, verify that it was successful in
finding the U2D2, Kobuki (if applicable), and Lidar (if applicable) by checking that the port names
show up as ``ttyDXL``, ``kobuki`` (if applicable), and ``rplidar`` (if applicable) respectively.

.. code-block:: console

    $ ls /dev | grep ttyDXL
    ttyDXL
    $ ls /dev | grep rplidar    # if applicable
    rplidar
    $ ls /dev | grep kobuki     # if applicable (robot uses the Kobuki as its base)
    kobuki

If using the Create® 3 as a base, use the below command to ping the base to ensure that your
network is properly configured to use the 192.168.186.2 to connect to the base. The base must be
turned on and its Ethernet cable should be plugged into the NUC.

.. code-block:: console

    $ ping -c 2 192.168.186.2   # if applicable (robot uses the Create® 3 as its base)
    PING 192.168.186.2 56(84) bytes of data.
    64 bytes from 192.168.186.2: icmp_seq=1 ttl=64 time=0.040 ms
    64 bytes from 192.168.186.2: icmp_seq=2 ttl=64 time=0.035 ms

    --- localhost ping statistics ---
    2 packets transmitted, 2 received, 0% packet loss, time 1012ms
    rtt min/avg/max/mdev = 0.035/0.037/0.040/0.002 ms

Verify that the RealSense camera can be found by running the command below:

.. code-block:: console

    $ roslaunch realsense2_camera rs_camera.launch

As long as the console's log does not include the repeating warning message ``No RealSense devices
were found!``, you will be able to connect to the camera using ROS.

Next Steps
==========

If the ROS Interface installed properly, you can continue on to the :doc:`ROS Interface Quickstart
Guide <./quickstart>`.

Troubleshooting
===============

Refer to the :doc:`X-Series Troubleshooting Guide <../../troubleshooting>` to try to solve your
problem. If you still need help, feel free to `open an Issue`_ on the ros_rovers repo. We strongly
recommend the latter option though so that other people who may be facing the same difficulty can
benefit. This repository is actively maintained and any open Issues will be addressed as soon as
possible.

.. _open an Issue: https://github.com/Interbotix/interbotix_ros_rovers/issues

Video Tutorials
===============

LoCoBot ROS Installation Guide
------------------------------

.. youtube:: 0lnbw6n6vs4
   :align: center
   :width: 70%
