.. _balance_bot-code_upload:

==============================
Cloning and uploading the code
==============================
The balance bot module is not part of any currently released version of the rover firmware. So the only way to get it, is to clone the ardupilot repository on github. Then we need to manually build and upload the rover code to your flight controller. 

.. tip:: If you wish to run only the SITL simulator, only steps 1 and 2 are required

The steps to do this are mentioned below:

#. Clone the ardupilot repository following the instructions :ref:`here <git-clone>`. 
#. Then initialise all the submodules as explained :ref:`here <git-submodules>`.
#. Follow the intructions :ref:`here <building-setup-linux>` to setup your build environment. 
#. Upload the code to your flight controller board as explained `here <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.

Once the upload is complete, your board will reboot.
