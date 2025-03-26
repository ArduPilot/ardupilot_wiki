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

    .. image:: ../../../images/custom-firmware-build-server-addbuild.png
        :target: ../_images/custom-firmware-build-server-addbuild.png
        :width: 450px

#. Choose the vehicle, version, and board,in that order, for which you want to build the firmware.

    .. image:: ../../../images/custom-firmware-build-server.png
        :target: ../_images//custom-firmware-build-server.png
        :width: 450px

#. Select the features you wish to include in the firmware

    - The default features for the board/vehicle will automatically be selected
    - Some features depend on other features so when a feature is selected, all its dependencies will be automatically selected but if the feature is later deselected, the dependencies are not automatically deselected
    - It is possible to select a set of features that will not fit on a board which will cause the build will fail.

#. Click the **'Generate'** button. You will be redirected to the homepage, and a build log for the request you just submitted will appear.
#. Once the build is complete, click the **'folder'** icon next to your build request entry in the build list table. This will take you to the directory on the server where your build artifacts are hosted. Download the .apj file
#. Use your GCS to install the .apj file onto your autopilot.  If using Mission Planner, open the Install Firmware and click then "Load custom firmware" link

    .. image:: ../../../images/mission-planner-load-custom-firmware.png
        :target: ../_images/mission-planner-load-custom-firmware.png
        :width: 450px

custom-beta
-----------
Before deploying new features to the main instance of the application, we test them on a dedicated testing instance. This testing environment is accessible at `custom-beta.ardupilot.org <https://custom-beta.ardupilot.org>`__. Feel free to use the beta features and provide your valuable feedback to help us improve the application.


[copywiki destination="copter,plane,rover,planner,blimp"]
