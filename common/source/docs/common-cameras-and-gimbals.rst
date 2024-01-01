.. _common-cameras-and-gimbals:

===================
Cameras and Gimbals
===================

Copter, Plane and Rover support up to 3-axis gimbals, including advance
features like automated aiming of the camera at a Region of Interest
(ROI), and automatic triggering of a camera shutter.  Follow the links below
to explanations of how to configure gimbals and shutter triggering.

.. image:: ../../../images/Cameras_Gimbals.jpg
    :target: ../_images/Cameras_Gimbals.jpg

Gimbals
=======

ArduPilot supports both brushless direct drive gimbals (Tarot, SimpleBGC, SToRM32)
that have their own self-stabilization controllers and the simpler servo-driven
gimbals in which ArduPilot controls the stabilisation. Some gimbals also integrate still or video cameras.

-  :ref:`Servo Gimbals <common-camera-gimbal>` — older-style servo-driven gimbal where ArduPilot provides stabilisation
-  :ref:`Brushless PWM <common-brushless-pwm-gimbal>` - brushless gimbals that accept PWM or SBUS input for angle control
-  :ref:`DJI RS2 and RS3-Pro gimbals <common-djirs2-gimbal>`
-  :ref:`Gremsy Mio, Pixy, S1, T3, T7 and ZIO <common-gremsy-pixyu-gimbal>` - high quality 3-axis gimbals
-  :ref:`SimpleBGC (aka AlexMos) Gimbal Controller <common-simplebgc-gimbal>` - a popular 2-axis or 3-axis brushess gimbal controller which uses a custom serial interface
-  :ref:`Siyi ZR10, ZR30 and A8 <common-siyi-zr10-gimbal>` - 3-axis gimbal and camera
-  :ref:`SToRM32 Gimbal Controller <common-storm32-gimbal>` — an inexpensive 2-axis or 3-axis brushless gimbal controller which responds to MAVLink commands (a richer format than PWM) over a serial interface
-  :ref:`ViewPro gimbals <common-viewpro-gimbal>`
-  :ref:`Xacti gimbals <common-xacti-gimbal>`

Gimbals may be attached to retractable mounts to prevent ground contact or to reduce air resistant in flight. Mount control is covered on the :ref:`common-mount-targeting` page.

.. _common-cameras-and-gimbals_camera_shutter_triggering:

Cameras
=======

Camera manufacturers use many differing mechanisms for remote control of the
camera (including its shutter or video recording). The following topics explain how to configure camera shutter and controls, and list a number of different approaches for converting
the output signal into the form expected by your particular camera. If the camera is integrated with the gimbal, control of the camera is detailed in that gimbals setup.

General Camera Control and Parameters
-------------------------------------

-  :ref:`Camera General Parameters and Control<common-camera-controls>`

Shutter Controllers
-------------------

-  :ref:`Servo or Relay controlled camera shutter <common-camera-shutter-with-servo>` (servo, relay).
-  :ref:`Camera Triggering using CHDK <common-apm-to-chdk-camera-link-tutorial>`
-  :ref:`Camera Triggering using CHDK Tutorial <common-chdk-camera-control-tutorial>` (non-standard integration)
-  :ref:`Seagull IR Camera Trigger <common-camera-trigger-seagull-ir>`
-  :ref:`Seagull MAP2 Camera Trigger <common-camera-trigger-seagull-map2>`
-  :ref:`Seagull MAP-X2 Camera Trigger and Logger <common-camera-trigger-seagull-mapx2>`
-  :ref:`Seagull REC Camera Trigger <common-camera-trigger-seagull-rec>`
-  :ref:`StratosnapperV2 Camera Trigger <common-camera-trigger-stratosnapperv2>`

Control of Specific Camera Models
---------------------------------

-  :ref:`Airpixel Entire Geotagger <common-geotagging-airpixel-entire>`
-  :ref:`FLIR Vue Pro Thermal Camera <common-flir-vue-pro>`
-  :ref:`RunCam Camera Control <common-camera-runcam>`


Common fixes for poor video quality
===================================

Some of the more common causes and solutions for poor video are listed
below:

-  "Jello" effect (or rolling shutter) is a by-product of using a camera
   with a CMOS sensor (GoPro, et al) caused by vibration from unbalanced
   props/motors and can be mitigated by mounting the camera on soft
   rubber, silicone, foam ear plugs or sometimes just on velcro.
-  digital and optical stabilization systems found in many cameras often
   do not perform well because of the vibrations found on many
   multicopters.

   -  Exceptions: the Sony video camera balanced steady shot system is
      very effective even at maximum 30 power zoom.

-  For better and smoother Yaw, use Expo control on your RC and lower
   the :ref:`ACRO_Y_RATE<ACRO_Y_RATE>` gain in the autopilot.

It is important to remember that even with a perfect setup, photography
is an art as well as a science. Using the camera pointing straight to
the ground is a good place to start, but more dramatic viewpoints can be
achieved with angles other than vertical. Mount about 40 degrees
deviation from vertical to obtain mainly ground photos but with an oblique
view. About 70 degrees off vertical will give you a lot more sky thus giving
more scenic photos. ArduPilot will stabilize the gimbal to whatever position you set.

.. toctree::
    :hidden:

    Brushless PWM <common-brushless-pwm-gimbal>
    DJI RS2 and RS3-Pro gimbals <common-djirs2-gimbal>
    Gremsy Pixy U Gimbal <common-gremsy-pixyu-gimbal>
    Servo Gimbal <common-camera-gimbal>
    SimpleBGC Gimbal Controller <common-simplebgc-gimbal>
    Siyi ZR10, ZR30 and A8 <common-siyi-zr10-gimbal>
    SToRM32 Gimbal Controller <common-storm32-gimbal>
    ViewPro gimbals <common-viewpro-gimbal>
    Xacti gimbals <common-xacti-gimbal>
    FLIR Vue Pro Thermal Camera <common-flir-vue-pro>
    Airpixel Entire Geotagger <common-geotagging-airpixel-entire>
    Seagull IR Camera Trigger <common-camera-trigger-seagull-ir>
    Seagull MAP2 Camera Trigger <common-camera-trigger-seagull-map2>
    Seagull MAP-X2 Camera Trigger and Logger <common-camera-trigger-seagull-mapx2>
    Seagull REC Camera Trigger <common-camera-trigger-seagull-rec>
    StratosnapperV2 Camera Trigger <common-camera-trigger-stratosnapperv2>
    Camera Trigger Directly from AUX Ports <common-pixhawk-camera-trigger-setup>
    Camera Triggering Configuration <common-camera-shutter-with-servo>
    Camera Triggering using CHDK <common-apm-to-chdk-camera-link-tutorial>
    Camera Triggering using CHDK Tutorial <common-chdk-camera-control-tutorial>
    RunCam Camera Control <common-camera-runcam>
    Gimbal / Mount Controls <common-mount-targeting>
    Camera Controls <common-camera-controls>
