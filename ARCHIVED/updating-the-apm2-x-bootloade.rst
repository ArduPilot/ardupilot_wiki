.. _updating-the-apm2-x-bootloade:

========================================
Archived: Updating the APM2.x BootLoader
========================================

The Atmel 2560 boot loader should not have to replaced, however
sometimes developers or users or a brownout etc., will corrupt the boot
loader or the fuse bits. This renders the Mission Planner and other
firmware uploads unsuccessful.

You will need an AVI ISP Programmer to flash the boot loader and set the
fuses correctly.

The 2560 Boot loader HEX file is located in the APM source downloads for
Arudcopter, Plane, Rover in the folder. ::

    ..\\hardware\\arduino\\bootloaders\\stk500v2\\ 
    
Flashing details are here:  [TBD]

.. todo:: 

    Editors: I can't find details for flashing boot loader - need to add a
    link or details. Also is there a stand alone download location for the
    boot loader? 
