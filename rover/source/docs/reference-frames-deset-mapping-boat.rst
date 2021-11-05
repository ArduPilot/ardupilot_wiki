.. _reference-frames-deset-mapping-boat:

==================
DeSET mapping boat
==================

..  youtube:: V8N3lA-20fs
    :width: 100%

The DeSET mapping boat is a relatively small but agile electric mapping boat designed by JapanDrones <http://japandrones.com/> and AttracLab <http://attraclab.com/> with input from Shimane University <https://www.shimane-u.ac.jp/en/> and Lighthouse <https://isana.lighthouse-frontier.tech/>.

Parts List
----------

- :ref:`Torqeedo 1003c electric motor <common-torqeedo>`
- 6x `Panasonic panasonic NKY580B02 16Ah Bike batteries <https://www.amazon.co.jp/-/en/Panasonic-NKY580B02-25-2V-Lithium-Battery/dp/B07B533K72>`__
- `Ideal diodes <https://discuss.ardupilot.org/t/ideal-diodes-when-using-multiple-batteries/75487>`__ to allow safely connecting multiple batteries
- `Hondex HE-8S sonar <https://www.honda-el.co.jp/product/marine/lineup/gps_plotter_fishfinder/he-8s>`__
- Waterproof box to hold autopilot, Herelink receiver, dAISy2+ and steering mechanism
- "Glands" for passing wires between waterproof box and main frame
- :ref:`CubePilot CubeOrange <common-thecubeorange-overview>` or similar :ref:`ArduPilot compatible autopilot <common-autopilots>`
- `CubePilot HereLink <https://www.cubepilot.com/#/herelink/features>`__ or any 8 channel (or more) transmitter

The steering mechanism is a custom built servo and pulley system but we hope to eventually provide a 3D printable steering mechanism

Parameters: `deset-mapping-boat.param <https://github.com/ArduPilot/ardupilot/blob/master/Tools/Frame_params/deset-mapping-boat.param>`__

Firmware used: Rover-4.2.0
