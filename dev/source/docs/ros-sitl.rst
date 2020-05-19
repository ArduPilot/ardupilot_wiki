.. _ros-sitl:

=============
ROS with SITL
=============

Connect ArduPilot to ROS
------------------------

First, a good ROS habit is to always work in a workspace directory, so create one:
Open a terminal

.. code-block:: bash

    mkdir -p ardupilot_ws/src
    cd ardupilot_ws
    catkin init
    cd src

OK ROS is ready to use, let's launch an SITL instance.
Refer to :ref:`setting-up-sitl-on-linux` to know how to install SITL.
Navigate to your SITL install and launch a copter session:

.. code-block:: bash

   sim_vehicle.py -v ArduCopter --console --map

(--console to show mavproxy console, --map to have the copter on map)

Now you got an SITL instance launched with TCP and UDP access, you should have some line like:

.. code-block:: none

    "mavproxy.py" "--master" "tcp:127.0.0.1:5760" "--sitl" "127.0.0.1:5501" "--out" "127.0.0.1:14550" "--out" "127.0.0.1:14551" "--map" "--console"

Both "--out" refer to UDP connexion create by MAVProxy. We will use UDP access with mavros.

Get back to your ROS terminal. Let's create a new directory for our launch file.

.. code-block:: bash

    mkdir launch
    cd launch

.. tip::

    It is simpler to write launch file than remembering ROS command.

Let's copy MAVROS default launchfile for ArduPilot :

.. code-block:: bash

    roscp mavros apm.launch apm.launch

You should have a new file in your directory called "apm.launch". "roscp" command is just a helper to call system command on ROS package.
Open it with your favorite editor, mine is gedit.

.. code-block:: bash

    gedit apm.launch

.. code-block:: none

    <launch>
        <!-- vim: set ft=xml noet : -->
        <!-- example launch script for ArduPilotMega based FCU's -->

        <arg name="fcu_url" default="/dev/ttyACM0:57600" /> <!-- Port et baudrate of the connexion with Pixhawk -->
        <arg name="gcs_url" default="" /> <!-- Retransmission to a GCS like Mavproxy does -->
        <arg name="tgt_system" default="1" /> <!-- MAVLink id of your drone, default is 1 -->
        <arg name="tgt_component" default="1" /> <!-- MAVLink component id of your drone, default is 1 -->
        <arg name="log_output" default="screen" /> <!-- Where ROS will ouput its message, screen is your current terminal -->

        <include file="$(find mavros)/launch/node.launch"> <!-- This launch file will launch another launch file -->
            <arg name="pluginlists_yaml" value="$(find mavros)/launch/apm_pluginlists.yaml" /> <!-- Mavros plugin configuration, we will modify that later -->
            <arg name="config_yaml" value="$(find mavros)/launch/apm_config.yaml" /> <!-- Mavros plugin list to use -->

            <arg name="fcu_url" value="$(arg fcu_url)" /> <!-- Pass the parameter to the other launch file -->
            <arg name="gcs_url" value="$(arg gcs_url)" />
            <arg name="tgt_system" value="$(arg tgt_system)" />
            <arg name="tgt_component" value="$(arg tgt_component)" />
            <arg name="log_output" value="$(arg log_output)" />
        </include>
    </launch>


To connect to SITL we just need to modify the first line to ``<arg name="fcu_url" default="udp://127.0.0.1:14551@14555" />``. save you file and launch it with

.. code-block:: bash

    roslaunch apm.launch

You should see some verbose from MAVROS that read its configuration and some line that indicate a connexion:

.. code-block:: none

    [ INFO] [1496336768.500953284]: CON: Got HEARTBEAT, connected. FCU: ArduPilotMega / ArduCopter
    [ INFO] [1496336768.536761724]: RC_CHANNELS message detected!
    [ INFO] [1496336769.533950451]: VER: 1.1: Capabilities         0x0000000000001bcf
    [ INFO] [1496336769.534021653]: VER: 1.1: Flight software:     03060000 (8a4a2722)
    [ INFO] [1496336769.534146986]: VER: 1.1: Middleware software: 00000000 (        )
    [ INFO] [1496336769.534195446]: VER: 1.1: OS software:         00000000 (        )
    [ INFO] [1496336769.534280663]: VER: 1.1: Board hardware:      00000000
    [ INFO] [1496336769.534309086]: VER: 1.1: VID/PID:             0000:0000
    [ INFO] [1496336769.534331512]: VER: 1.1: UID:                 0000000000000000
    [ WARN] [1496336769.534370049]: CMD: Unexpected command 520, result 0
    [ INFO] [1496336778.533962739]: FCU: APM:Copter V3.6-dev (8a4a2722)
    [ INFO] [1496336778.534247677]: FCU: Frame: QUAD
    [ INFO] [1496336779.021134163]: PR: parameters list received
    [ INFO] [1496336783.535151119]: WP: mission received


The connection was done !l!

Let use RQT to how ArduPilot information are shown in ROS. Normally, MAVROS will do most of the translation MAVLink <--> ROS
open another terminal and launch RQT with

.. code-block:: bash

    rqt

go to plugins/ topics /topics monitor
TADAM! !!!! You see all the topics that mavros has to create from ArduPilot information, click on the box to see the current value.
You could see in plugins/robot tools/ runtime monitor that everything is ok!

Let's try to change mode with mavros:
go to plugins / services/ services caller
set service to /mavros/set_mode
set custom_mode to 'GUIDED' and click the call button
The response should be true, you can look on /mavros/state topic that the mode is now GUIDED. it should be the same in you MAVProxy console.

Now, you know the base of ROS usage with ArduPilot! ROS got plenty others features that you can use like plotting, 3d visualisation, etc.
