==================
GoPro Manager
==================

.. code:: bash

    module load gopro

For 3DR Solo vehicles fitted with a GoPro camera, this module allows commanding
of the camera via the ``GOPRO_COMMAND_SHUTTER`` MAVLink message.

To power the camera on and off:

.. code:: bash

    gopro power <on|off>

To set the camera mode between photo and video:

.. code:: bash

    gopro mode <video|camera>

To start and stop the shutter:

.. code:: bash

    gopro shutter <start|stop>

To get the camera status:

.. code:: bash

    gopro status
