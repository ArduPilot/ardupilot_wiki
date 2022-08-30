.. _ros-install:

Installing ROS
--------------

General instructions for `installing ROS can be found here <http://wiki.ros.org/ROS/Installation>`__.

 - if installing on a desktop PC, install ROS Desktop-Full, on a companion computer ROS-Base is enough
 - add ROS tool to your shell as stated in ROS wiki
 - do not use root as the default user

Installing MAVROS
-----------------

MAVROS is compatible with all recent versions of ROS including Kinetic, Melodic and Noetic.

Instructions for `installing MAVROS can be found here <https://github.com/mavlink/mavros/tree/master/mavros#installation>`__ but in short involve running the following command.

.. code-block:: bash

    sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    chmod a+x install_geographiclib_datasets.sh
    ./install_geographiclib_datasets.sh

For ease of use on a desktop computer, please also install RQT

.. code-block:: bash

    sudo apt-get install ros-noetic-rqt ros-noetic-rqt-common-plugins ros-noetic-rqt-robot-plugins

