.. _reefmaster-for-bathymetry:

==========
ReefMaster
==========

.. image:: ../images/reefmaster-logo.png
    :target: https://reefmaster.com.au

`ReefMaster <https://reefmaster.com.au/>`__ is a popular tool for analysing sonar data.  This page provides steps for uploading sonar data collected by an ArduPilot drone boat to ReefMaster to produce 2D and 3D maps.

Note that these instructions apply to boats with a sonar directly connected to the autopilot and the lidar is providing only simple distances.  ArduPilot does not yet support directly consuming sidescan or backscatter data from a sonar.

Steps before Mapping
--------------------

- The vehicle should be :ref:`setup as a boat <boat-configuration>` meaning :ref:`FRAME_CLASS <FRAME_CLASS>` should be set to "2" (Boat)
- The :ref:`sonar <common-underwater-sonars-landingpage>` should be directly connected to the autopilot with :ref:`RNGFNDx_ORIENT <RNGFND1_ORIENT>`  = "25" (Down)
- :ref:`Onboard logs <common-downloading-and-analyzing-data-logs-in-mission-planner>` can be large so ensure the autopilot has a good quality SD card inserted with sufficient space (8GB or larger is a good choice)

Extract DPTH messages from the Onboard Log
------------------------------------------

- :ref:`Download the Onboard Logs over MAVLink <common-downloading-and-analyzing-data-logs-in-mission-planner>` or eject the autopilot's SD card, insert it into your PC and directly copy the .BIN files from the APM/LOGS directory to your PC
- Open the .BIN file using Mission Planner's Data >> DataFlash Logs >> Review a Log feature
- Check the "Data Table" checkbox, then click anywhere on the data table's column header and select "DPTH" from the drop-down and push the "Filter" button

  .. image:: ../images/reefmaster-log-filter.png
      :target: ../_images/reefmaster-log-filter.png

- Right-mouse-button-click in the data area and select "Export Visible" and save to a .csv file

  .. image:: ../images/reefmaster-log-export-visible.png
      :target: ../_images/reefmaster-log-export-visible.png

Import to ReefMaster
--------------------

- Open ReefMaster and create a new workspace
- Select File, Import Logs and Waypoints, Browse and select the CSV file produced above
- On the "Import Options" window set the following fields and then push the check button

    - Separator: Comma
    - Latitude: 6
    - Longitude: 7
    - Depth: 8

  .. image:: ../images/reefmaster-import-csv.png
      :target: ../_images/reefmaster-import-csv.png

- The "Import GPS Assets" window should appear next, press the check button on the bottom right
- Press the arrow on the top left to return to the main view
- Under "Tracks", right-mouse-button-click on the new track and "Add Track to Map Project" >> "New Project".  The sonar data should appear on the map.

  .. image:: ../images/reefmaster-add-track.png
      :target: ../_images/reefmaster-add-track.png

Creating 2D/3D maps
-------------------

- Under "Project", push "Define Map Area" and select an area with sonar data

  .. image:: ../images/reefmaster-define-map-area.png
      :target: ../_images/reefmaster-define-map-area.png

- Push "Generate Map" and "Map View" buttons to produce a 2D map

  .. image:: ../images/reefmaster-generate-map.png
      :target: ../_images/reefmaster-generate-map.png

- Push "3D View" to produce a 3D map

  .. image:: ../images/reefmaster-generate-map-3d.png
      :target: ../_images/reefmaster-generate-map-3d.png

- Push "Export" >> "Export Map Project" to create images or Google Earth KMZ files
