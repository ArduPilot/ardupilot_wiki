.. _mission-planner-flight-data:

===========================
Mission Planner Flight Data
===========================

DRAFT

This section covers the information you will need to use the features in
the Mission Planner Flight Data - selected in the Top menu of Mission
Planner.

-  **:ref:`An Introduction <mission-planner-ground-control-station>`** 
   An overview with pointers to the various areas of information.
-  **Heads Up Area**  (HUD) This is the area in the upper left side of
   the Mission Planner menu.

   -  See "An Introduction" above to see what each area and entry
      displays.
   -  You can detach the HUD to a separate window by double clicking
      anywhere in the window.  Close the window to put back in the
      Mission Planner.
   -  Options: Several Options are available by right clicking the HUD.

      -  User Items: You can add any of the telemetry parameters to the
         display by right clicking on the HUD, clicking User Items, and
         checking the items you want displayed.  Note that you can view
         all of the telemetry in the Control and Status area - Status
         button.
      -  Other options - what they do TBD

-  **Control and Status (Lower Left) Area** The Control and Status area
   of the Flight Data screen is in the lower left hand portion of
   Mission Planner.  In this area you can select any of several
   different menu items. Some items provide informations (status) and
   other items allow control of the vehicle using the telemety uplink.
   (Telemetry radio is required)

   -  Quick:  Use this area for a quick look at just a few telemetry
      values in large text.  Double click to add items.
   -  Actions: Use this area to control your Auto Pilot either for
      testing (using USB and no motor battery) or for controlling you
      vehicle  More detail TBD
   -  Gauges: Use this area to gauges for the four popular telemetry
      data. Double click the speed gauge to change the top speed.
   -  Status: Clicking the Status menu button will display all of the
      telemetry parameters
   -  Servos:  TBD
   -  **:ref:`Telemetry Logs <common-mission-planner-telemetry-logs>`**   Use
      this section to view, analyze, convert, and play back telemetry
      logs that are recorded by Mission Planner.

      -  :ref:`Playing Back Missions with Tlogs <common-recording-and-playing-back-missions>` 
         Specifics on playing back your mission and viewing the mission
         in the map area.

   -  Data Flash Logs  This is just a short cut to analyze a data flash
      log from this area without going to the Terminal mode. Click
      Browse to open a log file and establish a new window to
      view/analyze the log.
      Details :ref:`here <common-downloading-and-analyzing-data-logs-in-mission-planner>`.
   -  :ref:`Scripts: <using-python-scripts-in-mission-planner>` - A different way to create missions

-  **Map Area:** The map area on the right side of the Flight Data
   screen displays the vehicle track as it moves, provides other
   information and allows the user to enter some control actions - which
   send commands to the vehicle (telemetry required)

-  Right Click Options:  Right click on the map area to select one of
   these options.

-  Fly to here: (Command) This is guided mode  (see bottom of this
   page :ref:`Guided Mode <mission-planner-ground-control-station>`)
-  Fly to here Alt:  (Command) Lets you enter an altitude.
-  Point Camera Here: (Command)
-  Trigger Camera Now  (Command):
-  Flight Planner:  You can open the flight planner window in the map
   area and leave the rest of the Flight Data Screen as is.

-  Information/options at the bottom of the screen

   -  hdop, sats: shows information about the GPS reception quality and
      the number of satellites in view.
   -  Legend:  Each color corresponds to the color of the corresponding
      line showing directions and headings. Black is the GPS track as
      you vehicle travels.
   -  Tuning:  Opens/closes the tuning window. Details are found in
      various areas of the website (Need links for Plane, Copter).
   -  Auto Pan: Checking this box will make the map follow the vehicle
      and thus keep the vehicle in the center of the screen.
   -  Zoom: Shows or selects the current zoom level of the map.

-  Scroll bar: Use the scroll bar to change the zoom level of the map.

-  Other Hints and Tricks for Map area

   -  Change the map used:  Right Click, select Planner, change the map,
      then close the Planner view.  The new map will now be used in the
      Flight Data view.

      Note: To open the action pane in Planner view, click the little
      green chevron button [<<] in the upper right corner.

