.. _custom-build-server:

=================================
ArduPilot Custom Firmware Builder
=================================

Design
------
The ArduPilot Custom Firmware Builder is a monolithic Flask application running on ArduPilot's autotest server, with Apache WSGI serving as the web server. The application includes a builder thread that continuously consumes build requests from a queue and processes them accordingly.

The available versions are listed in a file called `remotes.json`. You can find the schema for the `remotes.json` file `here <https://github.com/ArduPilot/CustomBuild/blob/main/remotes.schema.json>`__. We also have an automated setup that updates this file with the latest releases from ArduPilot, as well as some tags from certain whitelisted repositories belonging to members of the ArduPilot development team. Below is a diagram illustrating how the `remotes.json` file is updated automatically.

::

                                                   +------+                                                             
                                                   | Cron |                                                             
                                                   +--+---+                                                             
                                                      |                                                                 
                                                      |run every hour                                                   
                                                      v                                                                 
                                     +-----------------------------------+                                              
                                     |                                   |                                              
                                     | update_remotes_json_cronjob_*.sh  |                                              
                                     |                                   |                                              
                                     |+---------------------------------+|                                              
                                     ||         fetch_releases.py       ||           update     +------------+          
                                     ||  (add AP's official releases)   ++------+-------------->|remotes.json|          
                                     |+---------------------------------+|      |               +-----+------+          
                                     |+---------------------------------+|      |                     |read             
                                     ||     fetch_whitelisted_tags.py   ++------+                     v                 
                                     ||(add tags from whitelisted repos)||            +--------------------------------+
                                     |+---------------------------------+|            |          Flask App             |
                                     |+---------------------------------+|            |                                |
    +--------------------+           ||      POST /refresh_remotes      ||            | (validates remotes.json using  |
    |secrets/reload_token+----------->|     {"token": reload_token}     ++----------->| remotes.schema.json and lists  |
    +--------------------+    read   |+---------------------------------+|  trigger   | versions on 'add a build' page)|
                                     +-----------------------------------+            +--------------------------------+
                                                                                                      ^                 
                                                                                                      |read             
                                                                                             +--------+----------+      
                                                                                             |remotes.schema.json|      
                                                                                             +-------------------+      


To obtain the list of boards and available features for a particular version, the application checks out the specific commit of the listed version and then uses the `build_options.py <https://github.com/ArduPilot/ardupilot/blob/master/Tools/scripts/build_options.py>`__ and `board_list.py <https://github.com/ArduPilot/ardupilot/blob/master/Tools/scripts/board_list.py>`__ files to retrieve the necessary information.

Publishing a tag
----------------
In addition to listing official ArduPilot releases, we also allow development team members to publish tags from their ArduPilot forks to be listed on the custom build server. To publish a tag, the user must be included in the list of whitelisted users. To get whitelisted, add your name `here <https://github.com/ArduPilot/CustomBuild/blob/ac24c25cf346e3bfb70a8fb023f23d7c4bf367a1/scripts/fetch_whitelisted_tags.py#L35>`__ and create a pull request. Ensure that the name of the fork is `ardupilot`.

The tag you wish to list should follow a specific naming format:

- Tags named in the format `custom-build/xyz` are allowed to be built for all vehicles.
- Tags named in the format `custom-build/vehicle/xyz` are allowed to be built only for the specified vehicle.

Examples:
- A tag named `custom-build/some-test-work` will be listed under all vehicles.
- A tag named `custom-build/Copter/some-test-work` will be listed under Copter only.

After this, you can push the tag to your remote repository. The tag will be listed on the server once the cron job that adds these tags runs.
