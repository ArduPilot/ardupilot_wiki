.. _user-alerts-developer:

==========================
User Alerts for Developers
==========================

A User Alert is a formal report of an ArduPilot issue that may affect the safe operation of a vehicle. An "issue" is typically a bug report or GitHub Issue. "Safe Operation" is the ability of a vehicle to reliably respond to user commands in a timely manner AND the ability of the vehicle to reliably follow commanded actions in automated flight modes.

Examples of Safe Operation issues include:

- Corrupted packets in I2C bus locking up the flight controller
- Non-commanded disarming whilst in-flight
- Malformed stored waypoint causes vehicle to fly away whilst in AUTO mode
- Triggering of the watchdog reset whilst in-flight

Examples of issues which are NOT Safe Operation issues include:

- Sub-optimal parameter values that lead to poor navigation performance
- User error or misunderstanding of parameter configuration
- Issues that, if experienced, an average user would be likely to safely recover from

User Alerts will only be for ArduPilot software issues. Manufacturer hardware will not be included at this stage, although this may be reviewed at a later date.

As per the open nature of ArduPilot, all User Alerts will be published publicly on `Github <https://github.com/ardupilot/useralerts>`__.

The User Alerts system consists of 3 parts:

- A process to raise, authorise and manage User Alerts
- A method of storing and handling the User Alert data
- The ability of applications (GCS software, websites) to ingest and use the User Alert data.


.. note::

   All User Alerts reported from 16/09/2020 (or later) are in the User Alert database. There is no guarantee
   that the database holds all User Alerts reported before this date.
   
Processes
=========

The process for entering a new User Alert is:

1. Bug reported or suspected
2. Issue raised on ArduPilot GitHub
3. Developer decides if safety issue
4. Developer raises PR on User Alert GitHub repo
5. 2nd Developer reviews and confirms that it's a real ArduPilot software bug
6. 3rd person confirms PR is filled out correctly and process has been followed
7. PR in User Alert repo merged by authorised team member

The process for resolving an active User Alert is:

1. Patch created for ArduPilot
2. Patch tested
3. Patch merged into ArduPilot master
4. Patch in stable ArduPilot release X.Y.Z
5. Developer Raises PR on User Alert Github repo to update User Alert fields
6. 2nd Developer reviews and confirms
7. 3rd person confirms PR is filled out correctly and process has been followed
8. PR in User Alert repo merged by authorised team member

Data Structure
==============


The User Alert data will be hosted on a `Github repository <https://github.com/ardupilot/useralerts>`__ under the ArduPilot organisation.

The repository will contain the following file/folder structure:

- README.md
- examples

  - EX00001.json
  - ...

- alerts

  - UA00001.json
  - ...

Each User Alert will be a json file. There will be 1 User Alert per json file. An applicable User Alert will be stored in the ``alerts`` folder.

The repository will have a small CI routine to confirm the validity of the json files with each PR.

The json files themselves will each have the below fields. There are examples in the `Github repository <https://github.com/ArduPilot/useralerts/tree/master/examples>`__ (the ``EX0000x.json`` files).

.. list-table::
   :widths: 20 40 40
   :header-rows: 1
   :class: useralerts-table

   * - Field Name
     - Type
     - Description

   * - dateRaised
     - ``YYYYMMDD``
     - Date that this User Alert was raised.

   * - affectedFirmware
     - String array containing ``["all"]`` OR individual firmwares: ``["copter", "sub", "antenna", "plane", "rover", "AP_Periph"]``
     - Which ArduPilot firmware is affected. Use comma separated value to specify multiple vehicles if "all" does not work.

   * - hardwareLimited
     - String array of board names, using the names as per the build system. For example: ``["CubeBlack", "Pixhawk1-1M", "KakuteF4"]``
     - If the User Alert is only applicable to certain boards, list the board names. An empty array indicates that all boards are affected.

   * - description
     - ``string``
     - Textual description of the User Alert. Should be understandable by an average user.

   * - criticality
     - ``int`` of value 1,2,3 or 4.
     - An assessment of the likelihood of the issue occurring. 1 = CRITICAL - Likely to be encountered by all vehicle configurations. 2 = CRITICAL - Likely to be encountered by specific vehicle configurations, 3 = MAJOR - possible to be encountered. 4 = MINOR - unlikely to be encountered.

     
   * - mitigation
     - ``string``
     - Textual description of any mitigations that a user can take to prevent the issue from occurring BEFORE a patched ArduPilot is released. Should be understandable by an average user.

   * - fixCommit
     - String array of commit ID’s. For example: ``["23874b32c9281dja", "w9085bqwfskjdtnu243"]``
     - Commit ID of the fix (on master branch) for this Issue. Can be multiple commits if the fix commits are spread among different libraries. Empty array if no fix yet. If this field is not ``[]``, it is considered that the issue has been fixed in master.

   * - dateResolved
     - ``YYYYMMDD``
     - Date that this User Alert was resolved. "Resolved" in this case means a patched ArduPilot version has been released for ALL affected vehicle and board types, and no further edits to this User Alert are expected.

   * - linkedIssue
     - ``string``
     - URL to Issue in ArduPilot GitHub repository. Optional.

   * - linkedInfo
     - String array of URLs
     - URLs to any supporting information about the issue, such as forum posts. Optional.

   * - linkedPR
     - ``string``
     - URL to the fix PR in ArduPilot GitHub repo. Blank if there is not PR yet.

   * - versionFrom
     - Dict of firmware versions. For example: ``{"copter": "4.0.1", "plane": "4.0.5"}``
     - ArduPilot release which introduced the issue, if known.  Empty assumes all previous versions. The dict must cover all firmwares listed in "Affected firmware".

   * - versionFixed
     - Dict of firmware=version. For example: ``{"copter": "4.0.1", "plane": "4.0.5"}``
     - ArduPilot release which contains fix. List must cover all firmwares listed in "Affected firmware". It is assumed that all versions between VersionFrom and this are affected by the User Alert. This field is an empty if there is no fixed version yet.

   * - lastmodified
     - ``string``
     - Date and time that this User Alert was modified, in ISO8601 format. This field is automatically added by the CI and does not need to be manually added.

Application Ingestion
=====================

To make application ingestion easier, there will be a generated manifest file listing all user alerts. These can then be filtered by the ``versionFrom``, ``versionFixed``, ``affectedFirmware`` and ``hardwareLimited`` fields to match with the user's autopilot and display any relevant user alerts.

There are URL's for both an example manifest (for testing purposes) and the actual user alerts manifest:

- URL for example User Alerts: https://firmware.ardupilot.org/useralerts/examplemanifest.json
- URL for User Alerts: https://firmware.ardupilot.org/useralerts/manifest.json

There is also a timestamp of the last date and time that the manifests were uploaded in https://firmware.ardupilot.org/useralerts/lastmodified.txt, in ISO8601 format.
