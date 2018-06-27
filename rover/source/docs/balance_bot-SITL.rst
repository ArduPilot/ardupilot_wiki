.. _balance_bot-SITL:

==========================
Using SITL for Balance Bot
==========================

If you are new to using SITL, please follow these :ref:`instructions<setting-up-sitl-on-linux>` to setup SITL. For executing SITL balance bot, navigate to the ardupilot/APMrover2 directory on terminal and execute the command:

:: 

    sim_vehicle.py -f balancebot --console --map

The vehicle should start in Manual Mode. You can now move it around using the mavproxy's rc override commands. rc3 is throttle and rc1 is yaw.

If you are not familiar with using mavproxy commands, please refer to the `mavproxy documentation <http://ardupilot.github.io/MAVProxy/html/index.html>`__ or this quick :ref:`guide<using-sitl-for-ardupilot-testing>`.