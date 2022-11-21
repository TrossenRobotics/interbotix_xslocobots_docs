=======================
Pairing Your Controller
=======================

This guide walks you through setting up a Bluetooth joystick controller to control your LoCoBot.

.. note::

    The processes below should be done on the LoCoBot's NUC computer.

.. note::

    The Interbotix software only supports the usage of SONY PS4 and PS3 controllers.

SONY PS4 Controller (Recommended)
=================================

1.  Click the **Bluetooth** icon on the top right of your screen, followed by **Bluetooth
    Settings**.

2.  Press and hold the **Share** button on the PS4 controller (see image below for reference).

3.  While holding the **Share** button, press and hold the **PS Button**. After a few seconds, the
    triangular shaped LED located between the **L2** and **R2** buttons should start rapidly
    flashing white (about twice a second) at which point you can let go.

4.  On the computer, click the **+** icon in the Bluetooth settings window.

5.  Wait until you see **Wireless Controller** pop up, select it, and click **Next** on the bottom
    right of the window.

6.  A message should display saying **successfully set up new device 'Wireless Controller'** and
    the LED should turn blue. This means the controller is connected to the computer.

7.  To disconnect, hold down the **PS Button** for about 10 seconds until the LED turns off.

8.  To reconnect, just press the **PS Button** (no need to hold it down). After blinking white a
    few times, the LED should turn blue.

.. image:: /_images/ps4.jpg
    :width: 70%
    :align: center

SONY PS3 Controller
===================

1.  Get an original SONY PS3 controller and it's accompanying USB cable.

2.  Open up a terminal by pressing :kbd:`Ctrl` + :kbd:`Alt` + :kbd:`T`, and enter the following
    commands.

    .. code-block:: console

        $ sudo bluetoothctl
        [bluetooth]# power on
        [bluetooth]# agent on
        [bluetooth]# scan on

3.  Plug the PS3 controller into the Linux Laptop. At this point, a message should pop up in the
    terminal that looks something like the following (with a different MAC address):

    .. code-block:: console

        [NEW] Device FC:62:B9:3F:79:E7 PLAYSTATION(R)3 Controller

4.  When it appears, type:

    .. code-block:: console

        [bluetooth]# trust <MAC-address>

5.  Unplug the PS3 controller and press the **PS Button**. The four red LEDs at the front of the
    controller should flash a few times, eventually leaving just one LED on by the '1'. This means
    that the joystick paired successfully.

.. note::

    Sometimes, the joystick might cause the cursor of the computer mouse to go crazy. To fix this,
    add the following line to the ``~/.bashrc`` file:

    .. code-block:: console

        alias joy_stop='xinput set-prop "PLAYSTATION(R)3 Controller" "Device Enabled" 0'

    Now, whenever the PS3 joystick is paired to the computer, just type ``joy_stop`` in the
    terminal to stop it messing with the mouse.

Troubleshooting
===============

Controller Isn't Turning On
---------------------------

Your controller might be dead. Try charing it for an hour or plug it into your computer.

PS4 Controller LEDs Blink A Few Times and Turn Off
--------------------------------------------------

Your controller may have a low charge level and needs to be charged. Try charing it for an hour or
plug it into your computer.
