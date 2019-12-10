.. _ros-install:

Installing ROS
--------------

General instructions for `installing ROS Kinetic can be found here <http://wiki.ros.org/kinetic/Installation>`__.  Ubuntu users should follow `these instructions <http://wiki.ros.org/kinetic/Installation/Ubuntu>`__. (only for Ubuntu 16.04)

Ubuntu 18.04 users will need to install ROS Melodic. General instructions are available `here <http://wiki.ros.org/melodic/Installation>`__. Replace ``kinetic`` with ``melodic`` in the instructions below.

 - if installing on a desktop PC, install ROS Desktop-Full, on a companion computer ROS-Base is enough
 - add ROS tool to your shell as stated in ROS wiki
 - do not use root as the default user

Installing MAVROS
-----------------

Instructions for `installing MAVROS can be found here <https://github.com/mavlink/mavros/tree/master/mavros#installation>`__ but in short involve running the following command

.. code-block:: bash

    sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    chmod a+x install_geographiclib_datasets.sh
    ./install_geographiclib_datasets.sh

For ease of use on a desktop computer, please also install RQT

.. code-block:: bash

    sudo apt-get install ros-kinetic-rqt ros-kinetic-rqt-common-plugins ros-kinetic-rqt-robot-plugins

We recommend using `caktin_tools <https://catkin-tools.readthedocs.io/en/latest/index.html>`__ instead of the default catkin_make as it is more powerful

.. code-block:: bash

    sudo apt-get install python-catkin-tools
