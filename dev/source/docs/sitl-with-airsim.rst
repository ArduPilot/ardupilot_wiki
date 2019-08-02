.. _sitl-with-airsim:

======================
Using SITL with AirSim
======================

.. youtube:: -WfTr1-OBGQ
   :width: 100%

`AirSim <https://github.com/microsoft/AirSim>`__ is a simulator for drones, cars and more, built on Unreal Engine (they also have an experimental Unity release, but right now it's hasn't been implemented with ArduPilot)

It is open-source, cross-platform and provides excellent physically and visually realistic simulations. It has been developed to become a platform for AI research to experiment with deep learning, computer vision and reinforcement learning algorithms for autonomous vehicles.

Currently, only support for ArduCopter vehicle has been developed in AirSim & ArduPilot.

.. note::

    This has been tested with everything in single Linux machine with Ubuntu 16.04, as well as with AirSim running in Windows 10 & ArduPilot inside WSL (Ubuntu 18.04)

AirSim is an excellent platform for testing and developing systems based on Computer Vision, etc. on simulated vehicles. It's a very feature-rich simulator with detailed environments and APIs (Python, C++, ROS) for collecting data. See the `main Airsim Readme <https://github.com/microsoft/AirSim#welcome-to-airsim>`__ for details and the features available.

A demo of AirSim running with ArduPilot SITL

.. youtube:: 0kE6gc7pn8M
    :width: 100%


Installing AirSim
=================

`Binaries <https://microsoft.github.io/AirSim/docs/use_precompiled/>`__ are available for AirSim, but since this is a very recent development, you'll have to build it yourself (on Windows as well as Linux).

Build on Windows
----------------

The main page for Windows setup is `here <https://github.com/microsoft/AirSim/blob/master/docs/build_windows.md>`__, but the instructions have been described below as well. Check the `FAQs on the page <https://github.com/microsoft/AirSim/blob/master/docs/build_windows.md#faq>`__ if there are any problems.

#. `Install Unreal Engine <https://github.com/microsoft/AirSim/blob/master/docs/build_windows.md#install-unreal-engine>`__

#. Build AirSim - Follow the `steps on AirSim Setup <https://github.com/microsoft/AirSim/blob/master/docs/build_windows.md#build-airsim>`__ for Visual Studio

  After cloning the repo, follow these steps to get the ArduCopter branch since the `AirSim PR <https://github.com/microsoft/AirSim/pull/2075>`__  for ArduCopter Support hasn't been merged

    ::

        cd AirSim
        git remote add <remote-name> https://github.com/rajat2004/AirSim.git
        git fetch <remote-name>
        git checkout -b <your-branch-name> <remote-name>/pr-arducopter

Run ``build.cmd`` from the command line. This will create ready to use plugin bits in the ``Unreal\Plugins`` folder that can be dropped into any Unreal project.


Build on Linux
--------------

AirSim's page on Linux Setup is `here <https://github.com/microsoft/AirSim/blob/master/docs/build_linux.md>`__, please see the FAQs if there are any problems

#. `Build Unreal Engine <https://github.com/microsoft/AirSim/blob/master/docs/build_linux.md#build-unreal-engine-and-airsim>`__ - After registering with Epic Games, you'll need to link your Github account with this, please see this `page <https://www.unrealengine.com/en-US/blog/updated-authentication-process-for-connecting-epic-github-accounts>`__ on how to link it.

#. **Build AirSim**

  #. Clone the repository

        ::

            git clone https://github.com/Microsoft/AirSim.git

  #. Currently, the `AirSim PR <https://github.com/microsoft/AirSim/pull/2075>`__  for ArduCopter Support hasn't been merged, so you'll need to fetch that particular branch for using Ardupilot.

        ::

            cd AirSim
            git remote add <remote-name> https://github.com/rajat2004/AirSim.git
            git fetch <remote-name>
            git checkout -b <your-branch-name> <remote-name>/pr-arducopter

  #. Build it

        ::

            ./setup.sh
            ./build.sh


Setup Unreal Environemt
-----------------------

Finally, you will need an Unreal project that hosts the environment for your vehicles. AirSim comes with a built-in "Blocks Environment" which you can use, or you can create your own.

Blocks is not a very highly detailed environment to keep the repo size reasonable but it's useful for various testing all the times and it is the easiest way to get your feet wet in this strange land.

See AirSim's `Blocks Setup page <https://github.com/microsoft/AirSim/blob/master/docs/unreal_blocks.md>`__ for running the environment.

For using another environment, check the information at the end of the page.


Using AirSim with ArduPilot
---------------------------

Make sure that you have setup ArduPilot SITL before proceeding.

.. note::

    Go to ``Edit->Editor Preferences``, in the ``Search`` box type ``CPU`` and ensure that the ``Use Less CPU when in Background`` is unchecked.


Launch SITL
+++++++++++

`AirSim's settings.json file <https://github.com/microsoft/AirSim/blob/master/docs/settings.md>`__ specifies the vehicle and it's various properties. See the page for the options available.

It's stored in at the following places- Windows: ``Documents\AirSim``, Linux: ``~/Documents/AirSim``

The file is in usual JSON format. On the first startup, AirSim would create ``settings.json`` file with no settings.

For using ArduCopter, the settings are as follows-

::

    {
      "SettingsVersion": 1.2,
      "LocalHostIp": "127.0.0.1",
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
              "AllowAPIAlways": false,
              "UdpIp": "127.0.0.1",
              "UdpPort": 9003,
              "SitlPort": 9002
            }
        }
    }

First launch AirSim, after that launch the ArduPilot SITL using

::

    sim_vehicle.py -v ArduCopter -f airsim-copter --add-param-file=libraries/SITL/examples/Airsim/quadX.parm --console --map

.. note::

    Initially, the editor will hang after pressing the Play button if the Ardupilot SITL hasn't been started (this is due to Lock-Step Scheduling). Run `sim_vehicle.py` and it should go back to normal.


For closing, first stop the AirSim simulation by pressing the Stop button, then close Ardupilot.
If Ardupilot is closed first, then UE hangs and you'll need to force close it.

You can restart by just pressing the Play button and then start the Ardupilot side, no need to close the Editor completely and then start it again.

Using Lidar
^^^^^^^^^^^

See `Lidar Settings <https://github.com/Microsoft/AirSim/blob/master/docs/lidar.md>`__ for info on Lidar

`PR <https://github.com/ArduPilot/ardupilot/pull/11835>`__ is open for adding Lidar support for AirSim in Ardupilot. Until it is merged, you'll have to fetch the specific branch for using this feature

::

    git remote add <remote-name> https://github.com/rajat2004/ardupilot.git
    git fetch <remote-name>
    git checkout -b <your-branch-name> <remote-name>/pr-airsim-lidar

Current `settings.json` file for launching Arducopter with Lidar

::

    {
      "SettingsVersion": 1.2,
      "LocalHostIp": "127.0.0.1",
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
          "DefaultVehicleState": "Disarmed",
          "UdpIp": "127.0.0.1",
          "UdpPort": 9003,
          "SitlPort": 9002,
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
              "DrawDebugPoints": true,
              "RotationsPerSecond": 10,
              "VerticalFOVUpper": 0,
              "VerticalFOVLower": 0,
              "HorizontalFOVStart": 0,
              "HorizontalFOVEnd": 359,
              "DataFrame": "SensorLocalFrame"
            }
          }
        }
      }
    }


Launch Copter with Lidar using

::

    sim_vehicle.py -v ArduCopter -f airsim-copter --add-param-file=libraries/SITL/examples/Airsim/quadX_lidar.parm --console --map

Manual Flying using RC
^^^^^^^^^^^^^^^^^^^^^^
Using an RC such as FrSky Taranis X9D Plus to fly manually is possible, you'll have to use the same branch as the Lidar for this.

Just plug the device in the computer and it should work. See `AirSim's Remote Control page <https://github.com/microsoft/AirSim/blob/master/docs/remote_control.md>`__ for details on supported devices and FAQs.

.. note::

    This feature hasn't been tested properly as of now so you might need to modify the Joystick file as mentioned in the page or set some RC parameters, especially if using a different controller.

Multi-Vehicle Simulation
^^^^^^^^^^^^^^^^^^^^^^^^

For simulating 2 copters, a example script has been added which will create 2 copter instances and enable Follow mode in one of them.

``settings.json`` for 2 copters

::

    {
      "SettingsVersion": 1.2,
      "LocalHostIp": "127.0.0.1",
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
          "DefaultVehicleState": "Disarmed",
          "UdpIp": "127.0.0.1",
          "UdpPort": 9003,
          "SitlPort": 9002
        },
        "Copter2": {
          "VehicleType": "ArduCopter",
          "UseSerial": false,
          "DefaultVehicleState": "Disarmed",
          "UdpIp": "127.0.0.1",
          "UdpPort": 9013,
          "SitlPort": 9012,
          "X": 0, "Y": 3, "Z": 0
        }
      }
    }

Press Play, cd to ardupilot directory then run the script to launch 2 copter instances.
You can specify the IP address of the computer with the GCS, if everything is on the same computer, use 127.0.0.1

::

    libraries/SITL/examples/Airsim/follow-copter.sh 127.0.0.1

To attach MAVProxy -

::

    mavproxy.py --master=127.0.0.1:14550 --source-system 1 --console --map

This will bring up the map but with only a single vehicle, use the ``vehicle`` command to switch between controlling the vehicles such as with ``vehicle 1`` & ``vehicle 2``, after which both the vehicles should be appearing on the map

Now, you can have the first vehicle (i,e with SYSID 1) flying in Guided or Auto Mission, and then takeoff the second vehicle and put it in Follow mode, after which the second copter will follow the first one.

For increasing the number of simulated vehicles, just modify the ``seq`` number in the script.

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

Using AirSim APIs
+++++++++++++++++

`AirSim's APIs document <https://github.com/microsoft/AirSim/blob/master/docs/apis.md>`__ explains the different APIs available and their usage.

Currently, ArduCopter vehicle doesn't support controlling the drone through APIs, for that you'll have to use something like Dronekit.

The `Image APIs <https://github.com/microsoft/AirSim/blob/master/docs/image_apis.md>`__ have been tested to work with Copter, for some ready-to-run sample codes, see the files in ``PythonClient/multirotor`` such as ``opencv_show.py``.

A ROS wrapper has also been added. See `airsim_ros_pkgs <https://github.com/microsoft/AirSim/tree/master/ros/src/airsim_ros_pkgs>`__ for the ROS API, and `airsim_tutorial_pkgs <https://github.com/microsoft/AirSim/tree/master/ros/src/airsim_tutorial_pkgs>`__ for tutorials.

.. note::

    Not all the APIs have been tested with Copter, if you find things that don't work or would like to have them supported, please let us know


Run on different machines
+++++++++++++++++++++++++

Change ``UdpIp`` to the IP address of the machine running Ardupilot, use ``-A`` option to pass the next arguments to the SITL instance, followed by ``--sim-address`` to specify Airsim's IP address

An example-

::

    sim_vehicle.py -v ArduCopter -f airsim-copter --add-param-file=libraries/SITL/examples/Airsim/quadX.parm --console --map -A --sim-address=127.0.0.1


Using different ports
^^^^^^^^^^^^^^^^^^^^^

``UdpPort`` denotes the port no. which Ardupilot receives the sensor data on (i.e. the port that Airsim sends the data to)

``SitlPort`` assigns the motor control port on which Airsim receives the rotor control message

- ``--sim-port-in`` should be equal to sensor port i.e. port specified in ``UdpPort``
- ``--sim-port-out`` should be equal to motor control port i.e. port specified in ``SitlPort``

Similar to changing the IP address as mentioned above, use ``-A`` to pass the arguments to the SITL instance.

Development Workflow
++++++++++++++++++++

AirSim's `Development Workflow page <https://github.com/microsoft/AirSim/blob/master/docs/dev_workflow.md>`__ explains the recommended setup for developing Airsim on Windows.

For Linux, make code changes in AirLib or Unreal/Plugins folder and then run ``./build.sh`` to rebuild. This step also copies the build output to Blocks sample project.
You can then follow the steps to start Unreal Editor and launch the project. When prompted about missing .so files, press Yes to build it again.

`Linux Troubleshooting <https://github.com/microsoft/AirSim/blob/master/docs/build_linux.md#faqs>`__