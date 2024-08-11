.. _common-custom-firmware:

=================================
ArduPilot Custom Firmware Builder
=================================

As the ArduPilot software evolves through community contributions and development efforts, it’s expanding rapidly with many new features being added. However, not all users need every feature. Depending on their specific requirements and applications, users might prefer to exclude certain features from the software.

To address this need, we have developed the **Custom Firmware Builder**, a web application hosted at `custom.ardupilot.org <https://custom.ardupilot.org/>`__. This process helps in reducing the firmware size, making it easier to fit into flight control units (FCUs) with limited flash memory. It also benefits our testers, allowing them to test specific features without needing to upgrade their flight controllers due to space constraints.

The source code of the custom firmware builder can be found `here. <https://github.com/ArduPilot/CustomBuild/>`__

Submitting a build request
--------------------------

To submit a build request, follow these steps:

#. Click the **'Add a build'** button located at the top right corner of the homepage.
#. Choose the vehicle, version, and board,in that order, for which you want to build the firmware.
#. Select the features you wish to include in the firmware. Keep in mind that some features may depend on others. When you select a feature, all its dependencies will be automatically selected.The default features for that board/vehicle will automatically be populated. The user can then select/deselect the desired features. Note that it is possible to select a set of features that will not fit on a board. In that case, the build will fail.
#. Click the **'Generate'** button. You will be redirected to the homepage, and a build log for the request you just submitted will appear.
#. Once the build is complete, click the **'folder'** icon next to your build request entry in the build list table. This will take you to the directory on the server where your build artifacts are hosted. You can download them from there.

custom-beta
-----------
Before deploying new features to the main instance of the application, we test them on a dedicated testing instance. This testing environment is accessible at `custom-beta.ardupilot.org <https://custom-beta.ardupilot.org>`__. Feel free to use the beta features and provide your valuable feedback to help us improve the application.


[copywiki destination="copter,plane,rover,planner,blimp"]
