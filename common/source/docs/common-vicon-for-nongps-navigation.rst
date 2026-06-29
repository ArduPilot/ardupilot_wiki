.. _Vicon Positioning System as fake GPS for Indoor Flight:

[copywiki destination="copter,plane,rover"]

===========================================================
Vicon Positioning System as fake GPS for Indoor Flight
===========================================================

Overview
========
Robotics labs commonly have an indoor flying facility using a Vicon
indoor positioning system. These systems use infra-red cameras to give
a high rate (200Hz) position and attitude information via a network
connection.

.. youtube:: XMb4MKi2HSQ
    :width: 100%


The Vicon system comes with a SDK that provides APIs for accessing the
Vicon positioning data. To use the Vicon data with ArduPilot you need
a tool which can use this API and map it onto MAVLink packets to send
to ArduPilot. This can be done by manipulating the MAVLink packet "GPS_INPUT" to create a fake GPS source for ArduPilot.

Hardware Setup
==============

The following hardware is used in this guide:

 - Vicon camera system with a Host PC
 - Vicon Calibration Wand 
 - Linux laptop (GCS) with both WiFi and Ethernet capability
 - Pixhawk6C flight controller
 - ESP8266 WiFi module
 - Conventional Radio controller and receiver
 - Minimum of 4 reflective IR markers per drone
 - Ethernet cables
 - Drone

System Overview
===============
A Ground Control Station (GCS) in the form of a Linux laptop recieves this positional data by listening in the broadcast from Vicon's Host PC, reparsing the positional data into MAVLink's GPS_INPUT format, and is sent wirelessly over a wifi module to a flight controller.


GCS Setup
=========
Download the following on the GCS laptop:

 - **Vicon DataStream SDK 1.10.0** - Select *Tracker*. (`Vicon <https://www.vicon.com/software/datastream-sdk/>`_)
 - **pyvicon** — a Python3 wrapper by MathGaron (`GitHub <https://github.com/MathGaron/pyvicon>`_)

Set Up a Conda Virtual Environment (Optional)
---------------------------------------------
.. code:: bash

   conda create -n VICON python=3.10
   conda activate VICON


Pyvicon installation
--------------------
Within the downloaded Vicon SDK, move shared libraries .so files into somewhere accessible for your os (For my case, lib inside my "Vicon" venv). Move .cpp and .h files into pyvicon-master/pyvicon folder as well. Below is an example of how to do this:

.. code:: bash

   # Move .so shared libraries into the venv lib folder
   find ~/Downloads/ViconDataStreamSDK_1.10.0.123216h -name "*.so" \
       -exec mv {} ~/miniconda3/envs/VICON/lib/ \;

   # Move .cpp source files into pyvicon
   find ~/Downloads/ViconDataStreamSDK_1.10.0.123216h -name "*.cpp" \
       -exec mv {} ~/Downloads/pyvicon-master/pyvicon/ \;

   # Move .h header files into pyvicon
   find ~/Downloads/ViconDataStreamSDK_1.10.0.123216h -name "*.h" \
       -exec mv {} ~/Downloads/pyvicon-master/pyvicon/ \;

Install the following packages/dependencies:

.. code:: bash

   sudo apt-get install libgtk-3-dev
   sudo apt remove modemmanager -y
   python3 -m pip install numpy==2.2.6
   python3 -m pip install pathlib2==2.3.7.post1
   python3 -m pip install matplotlib==3.10.9
   python3 -m pip install opencv-python==4.13.0.92
   python3 -m pip install Cython==3.2.4
   python3 -m pip install future==1.0.0
   python3 -m pip install wxpython==4.2.5

.. note::

   ``wxpython`` can take up to 20 minutes to install. If the terminal
   appears to stall at that package, it is still working — do not
   interrupt the process.


From inside the ``pyvicon-master`` directory, build and install pyvicon:

.. code:: bash

   cd ~/Downloads/pyvicon-master
   python3 setup.py install


Network Setup
-------------

Find a means to connect the Vicon Host PC to your GCS laptop (I am using CAT6 ethernet for lowest latency) and rename it's IP address as "vicon" in the hosts file. For example:

.. code:: bash

   sudo nano /etc/hosts

.. code:: bash

    # Example IP address 
    192.168.0.218 vicon 

Press ``Ctrl+O`` and ``Enter`` to save, then ``Ctrl+X`` to exit.

Verify connectivity:

.. code:: bash

   ping vicon

A flow of ``64 bytes from vicon (192.168.0.218)`` messages confirms a
successful connection between your GCS and the Vicon Tracker.

Drone Setup
===========

Connect the Pixhawk6C to your computer and navigate to **Config → Full Parameter List**. Using the search box, set the following parameters: 

.. list-table::
   :header-rows: 1
   :widths: 40 20

   * - Parameter
     - Value
   * - AHRS_EKF_TYPE
     - 3
   * - EK3_SRC1_POSXY
     - 3
   * - EK3_SRC1_POSZ
     - 3
   * - EK3_SRC1_VELXY
     - 3
   * - EK3_SRC1_VELZ
     - 3
   * - EK3_SRC1_YAW
     - 2
   * - EK3_SRC2_POSXY (Same for SRC2's VELXY, POSZ, VELZ, YAW)
     - 0
   * - EK3_SRC3_POSXY (Same for SRC3's VELXY, POSZ, VELZ, YAW)
     - 0
   * - EK3_MAG_CAL
     - 5
   * - EK3_POSNE_M_NSE
     - 0.002
   * - GPS1_TYPE
     - 14
   * - GPS1_DELAY_MS
     - 50
   * - COMPASS_USE
     - 0
   * - COMPASS_USE2
     - 0
   * - COMPASS_USE3
     - 0

These parameters configure the Extended Kalman Filter (EKF3) and GPS
subsystem to accept Vicon data as the main position source and disable
compass reliance. 

.. note::
    An empirical GPS1 delay of 50ms was found to work best with the Vicon system. If using a different network setup, you may need to adjust this value depending on your network latency.

    For remote connection between GCS and the flight controller, I used an ESP8266 WiFi module connected to a Pixhawk6C via the telemetry port. Update the SERIAL1_BAUD parameter to 921600 and SERIAL1_OPTIONS to 2 for this setup. If using a different telemetry module, you may need to adjust this value.


IR Marker Placement
-------------------
Mount at least 4 IR markers on the drone using hot glue, positioning
them where they are clearly visible from the camera's perspective. Use an
**asymmetrical arrangement**. 

.. note::
    If flying multiple drones simultaneously,
    each drone must have a geometrically distinct marker pattern to prevent
    the Vicon system from misidentifying objects.


Vicon Setup
===========
Switch on the Vicon Positioning System and also the Vicon Host PC, allow the system to warm up (~20 minutes).


Creating a Camera Mask
----------------------

Navigate to the **CALIBRATE** tab. Under *CREATE CAMERA MASK*, click
**START**. This takes a snapshot of the environment and filters out
any ambient IR reflections. 

Calibrating the Cameras
------------------------

Under **CALIBRATE CAMERAS**, click **START**. The computer will
verbally confirm ``"Calibration started"``.

Prepare the Vicon Calibration Wand and switch on its LEDs. Enter the flight arena and sweep the wand in a
cyclical arc facing the cameras, covering all camera positions in the
arena. Camera calibration progress is indicated by LED colour change:

 - Small cameras: red/pink → green when calibrated
 - Large cameras: red → green, with an on-screen progress bar

Continue sweeping until all cameras turn from static green to static
red, indicating complete calibration.

Return to the PC and check the **CAMERA CALIBRATION FEEDBACK** panel.
Once the progress bar completes, inspect the **World Error** and
**Image Error** values for all cameras. 

.. note::
    Any orange or red tiles in the table indicate a poor calibration — repeat the physical sweep in that case.

    Emphasize your sweep in areas your drone is expected to fly. (e.g. if you are flying in the corner, do more sweeps in that corner).

Setting the Origin
------------------

From within your testing environment, place the wand flat on the ground at your chosen
origin location. 

The direction convention for the Calibration Wand is usually known as: Bottom of handle = West, left handle = North. Align the wand as orthogonally as possible with your testing environment.

.. warning::

   A poorly oriented origin will result in a skewed inertial coordinate
   frame, which will cause problems for any path-planning algorithms.

Leave the wand on the ground, return to the PC, and navigate to **SET
VOLUME ORIGIN**. Click **START**. The computer will verbally confirm
``"Origin set"``.

Defining New Drone Objects
--------------------------

The Vicon system identifies objects by recognising the geometric
arrangement of IR markers on a drone. To define a new object:

1. Place the drone inside the testing environment. 

2. Align Autopilot's forward direction with the origin's North axis.

3. On the PC, click the **OBJECT** tab.

4. In the viewport (switch to **3D ORTHOGONAL to -Z** if markers are
   not visible), use Alt+click to drag-select your IR markers. The
   selected markers will show which cameras are tracking them.

5. At the bottom left of the screen, type a name for your object and
   click **CREATE**, then **SHARED**.

6. Locate your newly created object in the object list and save it.

7. Select your object in the list and ensure no other objects are selected.

.. warning::

   Ensure your Autopilot's North is aligned with Vicon's North when declaring a new object, otherwise you will have a mismatch between body and inertial frame. 

GCS integration 
-------------------------

If you haven't installed MAVProxy and pymavlink already:

.. code:: bash

   python3 -m pip install pymavlink==2.4.49
   python3 -m pip install mavproxy==1.8.74

You must clarify to Mavproxy which Vicon object to track. Navigate to the MAVProxy's module folder and find `mavproxy_vicon.py`, for example:

.. code:: bash

   ~/miniconda3/envs/VICON/lib/python3.10/site-packages/MAVProxy/

Open ``mavproxy_vicon.py`` and edit lines 29–36:

 - Set ``origin_lat``, ``origin_lon``, and ``origin_alt`` to ``0``.
 - Set ``object_name`` to the name of your drone object as defined in
   the Vicon Tracker (e.g. ``[TroglodytesDrone]``).

Save the file.

Ensure:
 - The GCS is connected to the Vicon Host PC.
 - The GCS is connected to the flight controller via WiFi module/your means.
 - The drone is placed inside the flight arena and tracked by the Vicon system.

Launch MAVProxy:

.. code:: bash

   mavproxy.py --master :<port_number> --console --map

Within the MAVProxy prompt, load and start the Vicon module:

.. code:: bash

   module load vicon
   vicon set
   vicon start

A successful connection will print:

.. code:: bash

   Connected to subject "<OBJECT_NAME>" segment "<OBJECT_NAME>"

The MAVProxy console should then display live NED (North-East-Down)
position and Euler angles (Roll, Pitch, Yaw) for the drone.

.. note::

   If the console reports a height of approximately -12cm while the
   drone is on the ground, this is expected — it reflects the offset
   between the drone's base and the geometric centre of the IR marker
   cluster.

Checking Orientation
--------------------

Monitor the Vicon status output in the MAVProxy console, carefully verify these values are consistent and sensible as you physically rotate and translate the drone before proceeding to flight.

Test Flight
-----------
Set the following flight mode assignments (using a 3-way switch on channel
5 or your preferred channel):

 - Flight Mode 1: Stabilize
 - Flight Mode 4: Alt Hold
 - Flight Mode 6: Loiter

Once the EKF3 reports yaw alignment and GPS lock, the drone is ready
to arm. For a first flight, :ref:`Stabilize mode <stabilize-mode>` is your manual control. :ref:`Alt Hold mode <altholdmode>` should be attempted first with great caution, then cascadingly loiter, guided, auto mode. 

As configurations differs between the lab configuration, this serves as more of a working princple of Vicon integration with Ardupilot. For a more extensive Dummy Guide based on experiences with specifc hardware in the Cranfield indoor flight arena, as well greater visualizations, it is available at: `Vicon Indoor Positioning System with Pixhawk6C <https://matthewt0809.github.io/Vicon-Positioning-System-To-Pixhawk6C-Guide/>`_.

.. warning::    

    The ESP8266 is reported to have connection issues in swarm flights.
    Use a Raspberry Pi wifi hosting feature for remote communication if conducting multi-drone operations. 
    
    Not all Pixhawk variants behave identically with this configuration — older
    Radiolink Pixhawk hardware does not work. 
    
    This guide has not been tested with PX4 firmware.

    Do not use Windows Subsystem for Linux (WSL) for the GCS — port communication permissions are difficult to configure correctly.
