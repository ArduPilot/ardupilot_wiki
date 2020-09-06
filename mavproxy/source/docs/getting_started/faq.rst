=====================================
Frequently Asked Questions and Issues
=====================================

#. I have random "noise" displaying on the console, showing as corrupted text. How do I remove it?

    MAVProxy will, by default, show corrupted packets. To disable showing them, use ``set shownoise false`` in the MAVProxy console.



#. How do I get MAVProxy to execute specific commands on startup?

    Put the commands in mavinit.scr as per :ref:`here <mavproxy-mavinit>`.


#. Which Flight controllers are compatible with MAVProxy?

    The ArduPilot vehicle types (`ArduPlane <https://ardupilot.org/plane/>`_, `ArduCopter <https://ardupilot.org/copter/>`_, `ArduSub <http://www.ardusub.com/>`_ and `ArduRover <https://ardupilot.org/rover/>`_) are fully compatible. Other Mavlink-based flight controllers (such as the `PX4 <https://px4.io/>`_ stack) should be compatible, but may not show all flight data.


#. How do I output a TCP connection to another GCS program?

    To output a TCP link from MAVProxy to a GCS program (ie. Mission Planner), use the ``--out=tcpin:0.0.0.0:<port>`` commandline option. This will tell MAVProxy to wait for a TCP connection on the specified port. In the other GCS program, configure it to point to the IP address of the device MAVProxy is running on and the port ``<port>``.
    
#. How do I disable other GCS sub-clients (from ``--out``) from sending commands to the flight controller?

    The ability to foward commands sent by other sub-clients is enabled by default. It can be disabled by typing ``set mavfwd false``
    
