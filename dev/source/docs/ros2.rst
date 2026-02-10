.. _ros2:

=====
ROS 2
=====

.. image:: ../images/logos/ros2_logo.png
    :target: ../_images/logos/ros2_logo.png

ArduPilot capabilities can be extended with `ROS <http://www.ros.org/>`__ (aka Robot Operating System).

`ROS <http://www.ros.org/>`__ provides libraries, tools, hardware abstraction, device drivers, visualizers, message-passing, package management, and more to help software developers create robot applications. ROS has been superseded by `ROS2 <http://design.ros2.org/articles/why_ros2.html>`__, and ArduPilot now natively supports it through its library `AP_DDS <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_DDS>`__.


Prerequisites
=============

- Learn to use ArduPilot first by following the relevant wiki for `Rover <https://ardupilot.org/rover/index.html>`__, `Copter <https://ardupilot.org/copter/index.html>`__ or `Plane <https://ardupilot.org/plane/index.html>`__.
- In particular, make sure the vehicle works well in Manual and Autonomous modes like Guided and Auto before trying to use ROS.
- Learn how to use ROS 2 by reading the `beginner tutorials <https://docs.ros.org/en/humble/Tutorials.html>`__.  In the case of a problem with ROS, it is best to ask on ROS community forums first (or google your error).

    We are keen to improve ArduPilot's support of ROS 2 so if you find issues (such as commands that do not seem to be supported), please report them in the `ArduPilot issues list <https://github.com/ArduPilot/ardupilot/issues>`__. A maintainer can add the `ROS` tag.

First, make sure that you have successfully installed `ROS 2 Humble <https://docs.ros.org/en/humble/Installation.html>`__ .
Currently, ROS 2 Humble is the only version supported.

You are about to create a new `ROS 2 workspace <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#id4>`__.
This page assumes that your workspace is named `ardu_ws` in your home directory, but feel free to adjust to your preferred location.

Before anything else, make sure that you have `sourced your ROS 2 environment <https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files>`__
and check if it is `configured correctly <https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#check-environment-variables>`__.


.. _ros2_installation_ubuntu:

Installation (Ubuntu)
=====================

To make installation easy, we will clone the required repositories using `vcs` and a `ros2.repos` files:

.. code-block:: bash

    mkdir -p ~/ardu_ws/src
    cd ~/ardu_ws
    vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src

This will take a few minutes to clone all the repositories your first time. It downloads the ArduPilot and `micro-ROS-Agent` repositories. Note that the master branch of ArduPilot is cloned by default. If you wish to use a different version, you will need to use Git commands to check out your desired ArduPilot version.

Now update all dependencies for `micro-ROS-Agent`:

.. code-block:: bash

    cd ~/ardu_ws
    sudo apt update
    rosdep update
    source /opt/ros/humble/setup.bash
    rosdep install --from-paths src --ignore-src -r -y

Installing the `Micro-XRCE-DDS-Gen` build dependency:

.. code-block:: bash
    
    sudo apt install default-jre
    cd ~/ardu_ws
    git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
    cd Micro-XRCE-DDS-Gen
    ./gradlew assemble
    echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc

Test `Micro-XRCE-DDS-Gen` installation:

.. code-block:: bash

    source ~/.bashrc
    microxrceddsgen -help
    # microxrceddsgen usage:
    #     microxrceddsgen [options] <file> [<file> ...]
    #     where the options are:
    #             -help: shows this help
    #             -version: shows the current version of eProsima Micro XRCE-DDS Gen.
    #             -example: Generates an example.
    #             -replace: replaces existing generated files.
    #             -ppDisable: disables the preprocessor.
    #             -ppPath: specifies the preprocessor path.
    #             -I <path>: add directory to preprocessor include paths.
    #             -d <path>: sets an output directory for generated files.
    #             -t <temp dir>: sets a specific directory as a temporary directory.
    #             -cs: IDL grammar apply case sensitive matching.
    #     and the supported input files are:
    #     * IDL files.

::
    ⚠️ If you have installed FastDDS or FastDDSGen globally on your system: eProsima's libraries and the packaging system in 
    Ardupilot are not deterministic in this scenario. You may experience the wrong version of a library brought in, or runtime 
    segfaults. For now, avoid having simultaneous local and global installs. If you followed the `global install <https://fast-dds.docs.eprosima.com/en/latest/installation/sources/sources_linux.html#global-installation/>`_ section, 
    you should remove it and switch to local install.

And finally, build your workspace:

.. code-block:: bash

    cd ~/ardu_ws
    colcon build --packages-up-to ardupilot_dds_tests

If the build fails, when you request help, please re-run the build in verbose mode like so:

.. code-block:: bash

    colcon build --packages-up-to ardupilot_dds_tests --event-handlers=console_cohesion+

If you'd like to test your ArduPilot ROS 2 installation, run:

.. code-block:: bash

    cd ~/ardu_ws
    source ./install/setup.bash
    colcon test --executor sequential --parallel-workers 0 --base-paths src/ardupilot --event-handlers=console_cohesion+
    colcon test-result --all --verbose

While `colcon` provides a convenient way for building multiple repositories in the correct order,
it hides all of the `./waf` options that ArduPilot developers are familiar with.
Most `ROS` packages written in C++ use a `CMake` build system, but ArduPilot uses `waf` and
has been wrapped by `CMake`.
If you would like all the `waf` options exposed, consider upvoting this
`issue. <https://github.com/ArduPilot/ardupilot/issues/27714>`__

The ``base-paths`` is used to limit testing only to ArduPilot.
We use a sequential executor and no parallel works because otherwise the tests try
to open multiple SITL processes on the same port, and the DDS traffic has cross-talk between
parallel tests.

Installation (MacOS)
====================

To make installation easy, we will install the required packages using `vcs` and a `ros2_macos.repos` files:

.. code-block:: bash

    cd ~/ardu_ws
    vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2_macos.repos src

Now update all dependencies:

.. code-block:: bash

    cd ~/ros_ws
    source /{path_to_your_ros_distro_workspace}/install/setup.zsh

Build microxrcedds_gen:

.. code-block:: bash

    cd ~/ardu_ws/src/microxrcedds_gen
    ./gradlew assemble
    export PATH=$PATH:$(pwd)/scripts

And finally, build your workspace:

.. code-block:: bash

    cd ~/ardu_ws
    colcon build --symlink-install --cmake-args \
    -DBUILD_TESTING=ON \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_MACOSX_RPATH=FALSE \
    -DUAGENT_SOCKETCAN_PROFILE=OFF \
    -DUAGENT_LOGGER_PROFILE=OFF \
    -DUAGENT_USE_SYSTEM_LOGGER=OFF \
    -DUAGENT_USE_SYSTEM_FASTDDS=ON \
    -DUAGENT_USE_SYSTEM_FASTCDR=ON \
    --event-handlers=desktop_notification-

If you'd like to test your installation, run:

.. code-block:: bash

    cd ~/ardu_ws
    colcon test \
    --pytest-args -s -v \
    --event-handlers console_cohesion+ desktop_notification- \
    --packages-select ardupilot_dds_tests

Installation (Docker)
=====================

Clone the ArduPilot docker project:

.. code-block:: bash

    git clone https://github.com/ArduPilot/ardupilot_dev_docker.git

Build the container:

.. code-block:: bash

    cd ~/ardupilot_dev_docker/docker
    docker build -t ardupilot/ardupilot-dev-ros -f Dockerfile_dev-ros .

Start the container in interactive mode:

.. code-block:: bash

    docker run -it --name ardupilot-dds ardupilot/ardupilot-dev-ros

Connect another bash process to the running container:

.. code-block:: bash

    docker container exec -it ardupilot-dds /bin/bash

The remaining steps are the same as for Ubuntu. You may need to install MAVProxy if it is not available on the container.

.. code-block:: bash

    pip install -U MAVProxy
