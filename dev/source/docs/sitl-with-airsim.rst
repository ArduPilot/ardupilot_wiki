.. _sitl-with-airsim:

======================
Using SITL with AirSim
======================

.. youtube:: -WfTr1-OBGQ
   :width: 100%

`AirSim <https://github.com/microsoft/AirSim>`__ is a simulator for drones, cars and more, built on Unreal Engine (they also have experimental support for Unity, but right now it hasn't been implemented with ArduPilot)

It is open-source, cross-platform and provides excellent physically and visually realistic simulations. It has been developed to become a platform for AI research to experiment with deep learning, computer vision and reinforcement learning algorithms for autonomous vehicles.

Currently, support for Copter & Rover vehicles has been developed in AirSim & ArduPilot.

The integration been tested with both running on a single Linux machine and separate machines also with Ubuntu 16.04, 18.04 as well as with AirSim running in Windows 10 & ArduPilot inside WSL (Ubuntu 18.04). It is possible to run AirSim on MacOS, however integration with ArduPilot hasn't been tested on that.

AirSim is an excellent platform for testing and developing systems based on Computer Vision, etc. on simulated vehicles. It's a very feature-rich simulator with detailed environments and APIs (Python, C++, ROS) for collecting data. See the `main Airsim Readme <https://github.com/microsoft/AirSim#welcome-to-airsim>`__ for details and the features available.

A demo of AirSim running with ArduPilot SITL

.. youtube:: ElFAqtpEfKo
    :width: 100%

A list of topics for easier navigation in the page-

#. :ref:`Install AirSim <sitl-with-airsim-install>`

    * :ref:`Build on Windows <sitl-with-airsim-build-windows>`
    * :ref:`Build on Linux, MacOS <sitl-with-airsim-build-linux>`

#. :ref:`Setup Unreal Environment <sitl-with-airsim-setup-unreal>`

#. :ref:`Using AirSim with ArduPilot <sitl-with-airsim-usage>`

#. :ref:`Launch Copter SITL <sitl-with-airsim-copter-sitl>`

#. :ref:`Launch Rover SITL <sitl-with-airsim-rover-sitl>`

#. :ref:`Using Lidar <sitl-with-airsim-lidar>`

#. :ref:`Using Rangefinder <sitl-with-airsim-rangefinder>`

#. :ref:`Manual Flying using RC <sitl-with-airsim-manual-rc>`

#. :ref:`Multi-Vehicle Simulation <sitl-with-airsim-multi-vehicle>`

#. :ref:`ROS with Multiple Vehicles <sitl-with-airsim-ros-multi-vehicle>`

#. :ref:`Custom Environment <sitl-with-airsim-custom-env>`

#. :ref:`Using AirSim APIs <sitl-with-airsim-apis>`

#. :ref:`Run on Different Machines <sitl-with-airsim-separate-machines>`

#. :ref:`Debugging and Development Workflow <sitl-with-airsim-dev-workflow>`

.. _sitl-with-airsim-install:

Installing AirSim
=================

`Binaries <https://github.com/microsoft/AirSim/releases>`__ are available for AirSim for Windows and Linux platforms, ``v1.3.0`` and later have support for Copter and Rover included. Binaries are a quick and easy way to test out the features without installing Unreal Engine, etc. Just download the precompiled environments and run to get started immediately. Many different types of environments are available, some notable ones include LandscapeMountains, City and Neighbourhood.

If you're using the binaries, then you can directly skip to `Using AirSim with ArduPilot <https://ardupilot.org/dev/docs/sitl-with-airsim.html#using-airsim-with-ardupilot>`__ section. Also see `this Readme <https://github.com/microsoft/AirSim/blob/master/docs/use_precompiled.md#dont-have-good-gpu>`__ for some useful commandline options to improve the performance.

For development and testing out new features, you'll have to build AirSim from source. The setup instructions for building from source are described below.

.. _sitl-with-airsim-build-windows:

Build on Windows
----------------

The main page for Windows setup is `here <https://github.com/microsoft/AirSim/blob/master/docs/build_windows.md>`__, but the instructions have been described below as well. Check the `FAQs on the page <https://github.com/microsoft/AirSim/blob/master/docs/build_windows.md#faq>`__ if there are any problems.

#. `Install Unreal Engine <https://github.com/microsoft/AirSim/blob/master/docs/build_windows.md#install-unreal-engine>`__

#. Build AirSim - Follow the `steps on AirSim Setup <https://github.com/microsoft/AirSim/blob/master/docs/build_windows.md#build-airsim>`__ for Visual Studio packages and clone AirSim.

Run ``build.cmd`` from the command line. This will create ready to use plugin bits in the ``Unreal\Plugins`` folder that can be dropped into any Unreal project.

.. _sitl-with-airsim-build-linux:

Build on Linux, MacOS
---------------------

AirSim's page on Linux & MacOS Setup is `here <https://github.com/microsoft/AirSim/blob/master/docs/build_linux.md>`__, please see the FAQs if there are any problems. Note that only MacOS **Catalina (10.15)** is supported.

#. `Build Unreal Engine <https://github.com/microsoft/AirSim/blob/master/docs/build_linux.md#build-unreal-engine-and-airsim>`__ - After registering with Epic Games, you'll need to link your Github account with this, please see this `page <https://www.unrealengine.com/en-US/blog/updated-authentication-process-for-connecting-epic-github-accounts>`__ on how to link it.

#. **Build AirSim**

  #. Clone the repository

        ::

            git clone https://github.com/Microsoft/AirSim.git

  #. Build it

        ::

            cd AirSim
            ./setup.sh
            ./build.sh

.. _sitl-with-airsim-setup-unreal:

Setup Unreal Environment
------------------------

Finally, you will need an Unreal project that hosts the environment for your vehicles. AirSim comes with a built-in "Blocks Environment" which you can use, or you can create your own.

Blocks is not a very highly detailed environment to keep the repo size reasonable but it's useful for various testing all the times and it is the easiest way to get your feet wet in this strange land.

See AirSim's `Blocks Setup page <https://github.com/microsoft/AirSim/blob/master/docs/unreal_blocks.md>`__ for running the environment.

For using another environment, check the information at the end of the page.

.. _sitl-with-airsim-usage:

Using AirSim with ArduPilot
---------------------------

Make sure that you have :ref:`setup ArduPilot SITL <dev:building-the-code>`, completed the Unreal Environment setup or have the binaries downloaded and verified that both are working individually before proceeding. If you aren't familiar with SITL, see :ref:`Examples of using SITL <dev:sitl-examples>`.

.. note::

    Running in UE Editor: Go to ``Edit->Editor Preferences``, in the ``Search`` box type ``CPU`` and ensure that the ``Use Less CPU when in Background`` is unchecked.

.. note::

    If you're using Windows Subsystem for Linux 2 to run ArduPilot and AirSim under Windows, please see https://discuss.ardupilot.org/t/gsoc-2019-airsim-simulator-support-for-ardupilot-sitl-part-ii/46395/5 on how to connect them.


`AirSim's settings.json file <https://github.com/microsoft/AirSim/blob/master/docs/settings.md>`__ specifies the vehicle and its various properties. See the page for the options available.

It's stored in at the following places- Windows: ``Documents\AirSim``, Linux: ``~/Documents/AirSim``

The file is in usual JSON format. On the first startup, AirSim would create ``settings.json`` file with no settings.

.. _sitl-with-airsim-copter-sitl:

Launch Copter SITL
++++++++++++++++++

For using ArduCopter, the settings are as follows-

::

    {
      "SettingsVersion": 1.2,
      "LogMessagesVisible": true,
      "SimMode": "Multirotor",
      "OriginGeopoint": {
        "Latitude": -35.363261,
        "Longitude": 149.165230,
        "Altitude": 583
      },
      "Vehicles": {
        "Copter": {
          "VehicleType": "ArduCopter",
          "UseSerial": false,
          "LocalHostIp": "127.0.0.1",
          "UdpIp": "127.0.0.1",
          "UdpPort": 9003,
          "ControlPort": 9002
        }
      }
    }

.. note::

    Earlier, ``SitlPort`` was used in place of ``ControlPort`` in the settings. This change is applicable in the latest AirSim master & binaries `v1.3.0` and later. The update is backwards-compatible so even if you're using ``SitlPort``, it'll work.

First launch AirSim, after that launch the ArduPilot SITL using

::

    sim_vehicle.py -v ArduCopter -f airsim-copter --console --map

.. note::

    Initially, the editor will hang after pressing the Play button if the ArduPilot SITL hasn't been started (this is due to Lock-Step Scheduling). Run `sim_vehicle.py` and it should go back to normal.

For closing, first stop the AirSim simulation by pressing the Stop button, then close ArduPilot.
If ArduPilot is closed first, then UE hangs and you'll need to force close it.

You can restart by just pressing the Play button and then start the ArduPilot side, no need to close the Editor completely and then start it again.

.. _sitl-with-airsim-rover-sitl:

Launch Rover SITL
+++++++++++++++++

``settings.json`` for using ArduRover-

::

    {
      "SettingsVersion": 1.2,
      "SimMode": "Car",
      "OriginGeopoint": {
        "Latitude": -35.363261,
        "Longitude": 149.165230,
        "Altitude": 583
      },
      "Vehicles": {
        "Rover": {
          "VehicleType": "ArduRover",
          "UseSerial": false,
          "LocalHostIp": "127.0.0.1",
          "UdpIp": "127.0.0.1",
          "UdpPort": 9003,
          "ControlPort": 9002,
          "AutoCreate": true,
          "Sensors": {
            "Imu": {
              "SensorType": 2,
              "Enabled": true
            },
            "Gps": {
              "SensorType": 3,
              "Enabled": true
            }
          }
        }
      }
    }

First launch AirSim, after that launch the ArduPilot SITL using

::

    sim_vehicle.py -v Rover -f airsim-rover --console --map

The other features, etc. described in this page have settings, commands and files specific for Copter, but can be used for Rover as well. Certain files such as scripts and ``settings.json`` will need to be modified for Rover, separate settings for Rover have not been added so as to keep the page managable and navigatable.

You might need to tune the vehicle for proper usage, the param files for AirSim vehicles in `Tools/autotest/default_params <https://github.com/ArduPilot/ardupilot/tree/master/Tools/autotest/default_params>`__ can be modified directly, or you can create a new param file and pass its location to SITL using ``--add-param-file`` option in ``sim_vehicle.py``.

.. _sitl-with-airsim-lidar:

Using Lidar
^^^^^^^^^^^

See `Lidar Settings <https://github.com/Microsoft/AirSim/blob/master/docs/lidar.md>`__ for info on Lidar and its properties in AirSim.

Current `settings.json` file for launching ArduCopter with Lidar

::

    {
      "SettingsVersion": 1.2,
      "SimMode": "Multirotor",
      "OriginGeopoint": {
        "Latitude": -35.363261,
        "Longitude": 149.165230,
        "Altitude": 583
      },
      "Vehicles": {
        "Copter": {
          "VehicleType": "ArduCopter",
          "UseSerial": false,
          "LocalHostIp": "127.0.0.1",
          "UdpIp": "127.0.0.1",
          "UdpPort": 9003,
          "ControlPort": 9002,
          "AutoCreate": true,
          "Sensors": {
            "Imu": {
              "SensorType": 2,
              "Enabled": true
            },
            "Gps": {
              "SensorType": 3,
              "Enabled": true
            },
            "Lidar1": {
              "SensorType": 6,
              "Enabled": true,
              "NumberOfChannels": 1,
              "PointsPerSecond": 5000,
              "DrawDebugPoints": false,
              "RotationsPerSecond": 10,
              "VerticalFOVUpper": 0,
              "VerticalFOVLower": 0,
              "HorizontalFOVStart": 0,
              "HorizontalFOVEnd": 359,
              "DataFrame": "SensorLocalFrame",
              "ExternalController": true
            }
          }
        }
      }
    }


Launch Copter with Lidar using

::

    sim_vehicle.py -v ArduCopter -f airsim-copter --add-param-file=libraries/SITL/examples/Airsim/lidar.parm --console --map

By default, :ref:`BendyRuler Object Avoidance <copter:common-oa-bendyruler>` is used with the Lidar, the related parameters can be seen on the Wiki page and should be modified as required in the ``lidar.parm`` file.

You can enable the visualisation of Lidar points in the AirSim viewport by setting ``DrawDebugPoints`` to ``true`` in the Lidar sensor settings. Note that this can reduce FPS by a lot and maybe even cause memory problems and crash in releases ``v1.3.1`` and earlier.


.. _sitl-with-airsim-rangefinder:

Using Rangefinder
^^^^^^^^^^^^^^^^^

Rangefinders in ArduPilot are called Distance Sensors in AirSim. See `AirSim's Sensors page <https://github.com/microsoft/AirSim/blob/master/docs/sensors.md#distance-sensor>`__ for details, and :ref:`Rangefinder Setup <copter:common-rangefinder-landingpage>` for ArduPilot side information.

Some example settings and parameters are shown below, to create a forward and downward-facing rangefinder. Note that only sensor settings are present, which can be easily inserted inplace of the ``Sensors`` element in the Lidar example above.

::

    "Sensors": {
        "Imu": {
          "SensorType": 2,
          "Enabled": true
        },
        "Gps": {
          "SensorType": 3,
          "Enabled": true
        },
        "Distance1": {
            "SensorType": 5,
            "Enabled": true,
            "DrawDebugPoints": true,
            "Yaw": 0, "Pitch": -90, "Roll": 0,
            "ExternalController": true
        },
        "Distance2": {
            "SensorType": 5,
            "Enabled": true,
            "DrawDebugPoints": true,
            "ExternalController": true
        }
      }

Details on the settings fields are present in the `AirSim Sensor page <https://github.com/microsoft/AirSim/blob/master/docs/sensors.md#distance-sensor>`__. ``DrawDebugPoints`` is set to ``true`` here since it can help in figuring out problems in orientation. Set ``ExternalController`` to ``false`` to disable sending the particular sensor data to AP while still having the sensor present. This can be useful in cases where you want to use your own avoidance system separate from the available ones, this setting is available for Lidars and Rangefinders.

To enable Rangefinder, ``RNGFNDx_TYPE`` should be set to 100 (SITL). Parameters to enable both Rangefinders -

::

    # First Rangefinder (Distance Sensor in AirSim), facing down
    RNGFND1_TYPE 100
    RNGFND1_MIN_CM 0
    RNGFND1_MAX_CM 4000
    RNGFND1_ORIENT 25

    # Second one, facing forward
    RNGFND2_TYPE 100
    RNGFND2_MIN_CM 0
    RNGFND2_MAX_CM 4000
    RNGFND2_ORIENT 0

Rangefinders can be used for Obstacle Avoidance, precise altitude measurement. Currently ArduPilot supports upto 10 rangefinders.

.. _sitl-with-airsim-manual-rc:

Manual Flying using RC
^^^^^^^^^^^^^^^^^^^^^^

For flying manually, you need a Remote Control or RC.

Just plug the device in the computer and it should work. See `AirSim's Remote Control page <https://github.com/microsoft/AirSim/blob/master/docs/remote_control.md>`__ for details on supported devices and FAQs.

.. note::

    This feature hasn't been tested properly as of now so you might need to modify the Joystick file as mentioned in the page or set some RC parameters, especially if using a different controller.

.. _sitl-with-airsim-multi-vehicle:

Multi-Vehicle Simulation
^^^^^^^^^^^^^^^^^^^^^^^^

For simulating 2 copters, an example script has been added which will create 2 copter instances and enable Follow mode in one of them.

``settings.json`` for 2 copters

::

    {
      "SettingsVersion": 1.2,
      "SimMode": "Multirotor",
      "OriginGeopoint": {
        "Latitude": -35.363261,
        "Longitude": 149.165230,
        "Altitude": 583
      },
      "Vehicles": {
        "Copter1": {
          "VehicleType": "ArduCopter",
          "UseSerial": false,
          "LocalHostIp": "127.0.0.1",
          "UdpIp": "127.0.0.1",
          "UdpPort": 9003,
          "ControlPort": 9002
        },
        "Copter2": {
          "VehicleType": "ArduCopter",
          "UseSerial": false,
          "LocalHostIp": "127.0.0.1",
          "UdpIp": "127.0.0.1",
          "UdpPort": 9013,
          "ControlPort": 9012,
          "X": 0, "Y": 3, "Z": 0
        }
      }
    }

Press Play, cd to ardupilot directory then run the script to launch 2 copter instances.
You can optionally specify the IP address of the computer with the GCS as the first argument, by default it'll be 127.0.0.1, meaning everything is on the same computer.

::

    libraries/SITL/examples/Airsim/follow-copter.sh <IP>

To attach MAVProxy -

::

    mavproxy.py --master=127.0.0.1:14550 --source-system 1 --console --map

This will bring up the map but with only a single vehicle, use the ``vehicle`` command to switch between controlling the vehicles such as with ``vehicle 1`` & ``vehicle 2``, after which both the vehicles should be appearing on the map

Now, you can have the first vehicle (i,e with SYSID 1) flying in Guided or Auto Mission, and then takeoff the second vehicle and put it in Follow mode, after which the second copter will follow the first one.

For increasing the number of simulated vehicles, just modify the ``NCOPTERS`` variable in the script and add the settings for each individual vehicle in the ``settings.json``.

.. note::

    There can be certain problems while working on multi-vehicle simulation due to networking differences between platforms such as Linux, WSL, Cygwin, etc. `This Discuss thread <https://discuss.ardupilot.org/t/simulating-2-drones-with-sitl-airsim-in-windows-cygwin-wont-work/49292>`__ could be helpful in such cases.

.. note::

    The difference of 10 between the ports is important since the script is launching the vehicles using the ``instance`` option which increases the ports from ArduPilot's side by 10. For using different ports, modify the script as required following the instructions at the end of the page for specifying the ports.

.. _sitl-with-airsim-ros-multi-vehicle:

ROS with Multi-Vehicle Simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Using ROS for multi-vehicle tasks is a common usecase and Mavros is used for working with Mavlink-based vehicles. There are some example scripts demonstrating how to use Mavros with multiple vehciles in ArduPilot.

First is the `multi_vehicle.sh script <https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/Airsim/multi_vehicle.sh>`__ which launches multiple ArduCopter binaries with different SYSIDs and ports for each vehicle. Usage is similar to the above script -

::

    libraries/SITL/examples/Airsim/multi_vehicle.sh <IP>


The `multi_uav_ros_sitl.launch file <https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/Airsim/multi_uav_ros_sitl.launch>`__ demonstrates how to write a launch file controlling multiple vehicles with Mavros. It creates a different namespace for each drone and each drone has a separate SYSID and ports according to how the script sets the variables.
Launching the file -
::

    roslaunch libraries/SITL/examples/Airsim/multi_uav_ros_sitl.launch

Separate MAVProxy instance can be launched for each drone by connecting to the TCP ports opened by the script for each drone. The UDP ports can't be used for this if Mavros is already running since Mavros will use the UDP ports.

The ``multi_vehicle.sh`` script doesn't enable the Follow Mode, but if this is also needed and if all the vehicles are to be displayed on the same GCS, then multicast and the Follow parameters as done in the ``follow-copter.sh`` script can be added.

.. _sitl-with-airsim-custom-env:

Custom Environment
++++++++++++++++++

For using another environment on Windows, see `AirSim's custom env setup page <https://microsoft.github.io/AirSim/docs/unreal_custenv/>`__.

Linux
^^^^^

As mentioned in the above-linked page, there is no Epic Games Launcher for Linux which means that if you need to use a custom environment, you will need Windows machine to do that.

The steps are the same once you have the Windows machine, after you have downloaded the Unreal project, just copy the project over to your Linux machine.

Follow the steps till after Step 6 where you have edited the ``.uproject`` file. After editing the project file, skip Step 7,8 and directly start the Editor by going to UnrealEngine folder and start Unreal by running ``UnrealEngine/Engine/Binaries/Linux/UE4Editor``.

When Unreal Engine prompts for opening or creating project, select Browse and select your custom environment. Afterwards, continue following the Steps from 9 onwards.

.. note::

    When using a custom environment, it might be the case that there are multiple ``Player Start`` objects. In such a case, it randomly chooses one and the vehicle can start in the air and fall.

    You'll have to delete the extra ``Player Start`` objects and leave one which has to be moved to near the ground. See this excellent video by one of the AirSim developers - `Unreal AirSim Setup <https://youtu.be/1oY8Qu5maQQ>`__, specifically at 5:00 where it's demonstrated how to delete the objects and to move the position.

.. _sitl-with-airsim-apis:

Using AirSim APIs
+++++++++++++++++

`AirSim's APIs document <https://github.com/microsoft/AirSim/blob/master/docs/apis.md>`__ explains the different APIs available and their usage.

Currently, ArduPilot vehicles don't support controlling the movement through the AirSim APIs, however any method of controlling the movement which connects directly to ArduPilot rather than using AirSim’s API work, examples include DroneKit & ROS with Mavros.

The `Image APIs <https://github.com/microsoft/AirSim/blob/master/docs/image_apis.md>`__ have been tested to work with Copter, for some ready-to-run sample codes, see the files in ``PythonClient/multirotor`` such as ``opencv_show.py``. Also, any APIs which are are used to fetch information about the environment, and interact with it, or even collect sensor information will work.

A ROS wrapper has also been added. See `airsim_ros_pkgs <https://github.com/microsoft/AirSim/blob/master/docs/airsim_ros_pkgs.md>`__ for the ROS API, and `airsim_tutorial_pkgs <https://github.com/microsoft/AirSim/blob/master/docs/airsim_tutorial_pkgs.md>`__ for tutorials. Note that limitations of controlling vehicle movement exist here as well.

.. note::

    Not all the APIs have been tested with Copter, if you find things that don't work or would like to have them supported, please let us know

.. _sitl-with-airsim-separate-machines:

Run on different machines
+++++++++++++++++++++++++

#. Change the following in the ``settings.json`` file-

    #. ``UdpIp`` to the IP address of the machine running ArduPilot (Can be found using ``ipconfig`` on Windows, ``ifconfig`` on Linux.)
    #. ``LocalHostIp`` to the IP address of the current machine which is running AirSim, specific to the network adapter being used such as Ethernet or WiFi. Can be set to ``0.0.0.0`` to receive messages on all networks


#. Use ``-A`` argument in ``sim_vehicle.py`` (passes the arguments following it to the SITL instance), followed by ``--sim-address`` to specify Airsim's IP address

An example-

::

    sim_vehicle.py -v ArduCopter -f airsim-copter --console --map -A --sim-address=127.0.0.1

.. note::

    If using Windows, you might need to disable Windows Firewall to receive messages

.. _sitl-with-airsim-different-ports:

Using different ports
^^^^^^^^^^^^^^^^^^^^^

``UdpPort`` denotes the port no. which ArduPilot receives the sensor data on (i.e. the port that Airsim sends the data to)

``ControlPort`` assigns the motor control port on which Airsim receives the rotor control message

- ``--sim-port-in`` should be equal to sensor port i.e. port specified in ``UdpPort``
- ``--sim-port-out`` should be equal to motor control port i.e. port specified in ``ControlPort``

Similar to changing the IP address as mentioned above, use ``-A`` to pass the arguments to the SITL instance. Example-

::

    sim_vehicle.py -v ArduCopter -f airsim-copter --console --map -A "--sim-port-in=9003 --sim-port-out=9002"

.. _sitl-with-airsim-dev-workflow:

Development Workflow
++++++++++++++++++++

AirSim's `Development Workflow page <https://github.com/microsoft/AirSim/blob/master/docs/dev_workflow.md>`__ explains the recommended setup for developing Airsim on Windows.

For Linux, make code changes in AirLib or Unreal/Plugins folder and then run ``./build.sh`` to rebuild. This step also copies the build output to Blocks sample project.
You can then follow the steps to start Unreal Editor and launch the project. When prompted about missing .so files, press Yes to build it again.

`Linux Troubleshooting <https://github.com/microsoft/AirSim/blob/master/docs/build_linux.md#faqs>`__

`Windows FAQs <https://microsoft.github.io/AirSim/docs/build_windows/#faq>`__

`General FAQs <https://microsoft.github.io/AirSim/docs/faq/>`__


Before reporting any problems, please update the ArduPilot and AirSim installations to the latest master. After updating the local AirSim repository, make sure to run the commands mentioned in the `Unreal Environment Setup page <https://github.com/microsoft/AirSim/blob/master/docs/unreal_blocks.md>`__, otherwise the updates won't be reflected in the simulation.
