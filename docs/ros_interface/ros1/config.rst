====================
ROS 1 Configurations
====================

This page describes the different configurations present when using the ROS 1 Interface.

.. contents::
    :local:

Environment Variables
=====================

INTERBOTIX_XSLOCOBOT_BASE_TYPE
------------------------------

.. code-block:: bash

    export INTERBOTIX_XSLOCOBOT_BASE_TYPE=${BASE_TYPE}

ROS_IP
------

LoCoBot Computer
^^^^^^^^^^^^^^^^

.. code-block:: bash

    export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)
    if [ -z "$ROS_IP" ]; then
        export ROS_IP=127.0.0.1
    fi

Remote Computer
^^^^^^^^^^^^^^^

.. code-block:: bash

    export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)
    if [ -z "$ROS_IP" ]; then
        export ROS_IP=127.0.0.1
    fi

ROS_MASTER_URI
--------------

LoCoBot Computer
^^^^^^^^^^^^^^^^

Remote Computer
^^^^^^^^^^^^^^^
Udev Rules
==========

