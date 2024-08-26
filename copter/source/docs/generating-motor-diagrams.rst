.. _generating-motor-diagrams:

================================
Generating Copter Motor Diagrams
================================

Motor diagrams such as the one below can be automatically generated and (optionally) automatically included in the :ref:`Connect ESCs and Motors<connect-escs-and-motors>` page.

.. image:: ../images/m_01_01_quad_x.svg
    :target: ../_images/m_01_01_quad_x.svg
    :scale: 44%
    :alt: QUAD X

Prerequisites
=============

* **Minimum:** A working ``ArduPilot/ardupilot_wiki`` build environment.

  *  See :ref:`Wiki Editing - Setting Up the Environment<common-wiki-editing-setup>`.

* **Recommended:** Additionally, a working ``ArduPilot/ardupilot`` build environment.

  * See :ref:`Building the code<dev:building-the-code>`.

Updating Generated JSON Data
============================

``scripts/motor-diagram-data/AP_Motors_test.json`` contains motor matrix information for all supported frame classes and types and is used to generate the diagram imagery. It should be updated whenever a new frame class or type is added or existing ones are significantly changed. After updating, re-generate motor diagrams using ``build_motor_diagrams.py`` as described below.

.. note::
    ``AP_Motors_test.json`` is procedurally generated and is not intended to be edited directly. Instead, edit ``AP_Motors_display.json`` as described below if diagram output must be adjusted/customized.

Updating AP_Motors_test.json (recommended method)
-------------------------------------------------

Enter the ardupilot build environment and execute the following:

.. code-block:: bash

    cd ardupilot # or wherever your ardupilot repo is cloned
    # configure and build the AP_Motors_test executable
    ./waf configure --board linux
    ./waf --targets examples/AP_Motors_test
    # generate AP_Motors_test.json
    ./build/linux/examples/AP_Motors_test p > AP_Motors_test.json

Move (or copy) the resulting file to ``ardupilot_wiki/scripts/motor-diagram-data/``. Assuming the ``ardupilot_wiki`` environment is set up in the parent directory of ``ardupilot`` :

.. code-block:: bash

    mv AP_Motors_test.json ../ardupilot_wiki/scripts/motor-diagram-data/

Once the output has been verified, changes can be committed to a new branch, and a pull request can be created.

Updating AP_Motors_test.json (alternate method)
-----------------------------------------------

``ardupilot_wiki/scripts/generate_motor_json.py`` can be used to update ``AP_Motors_test.json`` independent of the ArduPilot build environment. It fetches C++ header and source files from the ArduPilot repo and uses a similar method to the ``AP_Motors_test`` executable. Output should be functionally identical to ``AP_Motors_test`` for the purpose of generating Copter motor diagrams, but it is a more fragile method and may not produce JSON data for new/niche frame classes and types.

Enter the ardupilot_wiki build environment and execute the following:

.. code-block:: bash

    cd ardupilot_wiki # or wherever your ardupilot_wiki repo is cloned
    # make a backup
    mv scripts/motor_diagram_data/AP_Motors_test.json scripts/motor_diagram_data/AP_Motors_test.json.bak
    # generate AP_Motors_test.json
    ./scripts/generate_motor_json.py -o

There are a few custom options for the script. Run ``./scripts/generate_motor_json.py -h`` for a full list.

Once the output has been verified, the backup file can be deleted before committing changes to a new branch and creating a pull request.

Updating All Motor Diagrams
===========================

If ``AP_Motors_test.json`` or ``AP_Motors_display.json`` are changed, the following command can be used to update all copter motor diagrams:

.. code-block:: bash

    cd ardupilot_wiki # or wherever your ardupilot_wiki repo is cloned
    # optionally preview wiki image tags before building
    ./scripts/build_motor_diagrams.py --preview
    # generate copter motor diagrams and modify connect-escs-and-motors.rst 
    ./scripts/build_motor_diagrams.py --build

The ``--build`` option will build all copter motor diagrams and output them to ``copter/source/images/``. Additionally, it will replace the existing copter motor diagram images with the newly generated ones in ``copter/source/docs/connect-escs-and-motors.rst``. The ``--preview`` option will create the diagrams but stops short of altering the wiki page.

There are a number of other options for the script. Run ``./scripts/build_motor_diagrams.py -h`` for a full list.

Altering the Appearance of Motor Diagrams
=========================================

.. note::
    Most frame types do not require any additional display customization, but any diagram's appearance can be modified as follows.

``scripts/motor-diagram-data/AP_Motors_display.json`` amplifies and/or overrides the data in ``AP_Motors_test.json`` for selected frame types. ``AP_Motors_display.json`` can be edited to alter the appearance of diagrams. Each field overrides the corresponding field in ``AP_Motors_test.json`` or adds amplifying information for display in the diagram.

After editing ``AP_Motors_display.json``, re-generate diagrams using ``build_motor_diagrams.py`` as described above.

* All fields except class and frame IDs are optional (if not specified, values from ``AP_Motors_test.json`` are used)

* ``ClassName`` and ``TypeName`` override the title text

* ``Notes`` are displayed below the title text as part of the diagram (smaller font)

* ``WikiNotes`` add a wiki ```.. note::``` block below the diagram

* ``Comments`` are information only and ignored when generating the diagrams

* ``Skip`` if true, this frame is skipped by the generator script (useful for frames that have duplicate diagrams)

* ``motors`` includes motor matrix data.

  * ``Number`` is the intended motor output channel

  * ``TestOrder`` is the order in which the motor is tested (letters ``A``, ``B``, ``C``, etc.)

  * ``Rotation`` is the motor rotation direction (CCW or CW). A ``?`` indicates no yaw torque.

  * ``Roll`` is the "roll factor" of the motor and functions as an X coordinate for the diagram. Positive is below the origin, and negative is above. Generally, use values between -0.5 and 0.5.
 
  * ``Pitch`` is the "pitch factor" of the motor and functions as a Y coordinate for the diagram. Positive is to the right, and negative is to the left. Generally, use values between -0.5 and 0.5.

  * ``FrameLines`` can be used to override the default frame arm depiction, where each motor is connected to the center of the frame. The coordinate system functions the same as for ``Roll`` and ``Pitch``.

The following Quad A-Tail example showcases many of the available options:

.. code-block:: json

    {
        "1": {
            "ClassName": "QUAD",
            "5": {
                "Comments": "Motor vectors adjusted to show accurate diagram",
                "TypeName": "A TAIL",
                "WikiNotes": [
                    "Quad A Tail and V Tail frames do not use the front motors for yaw control (NYT). Motor rotation direction does not matter for these motors."
                ],
                "FrameLines": [
                    [0, 0, -0.5, 0.266],
                    [0, 0, 0.5, 0.266],
                    [0, 0, 0, -0.3],
                    [-0.008, -0.3, 0.4, -0.5],
                    [0.008, -0.3, -0.4, -0.5]
                ],
                "motors": [
                    {
                        "Number": 1,
                        "TestOrder": 1,
                        "Rotation": "?",
                        "Roll": -0.5,
                        "Pitch": 0.266
                    },
                    {
                        "Number": 2,
                        "TestOrder": 3,
                        "Rotation": "CCW",
                        "Roll": 0.3,
                        "Pitch": -0.5
                    },
                    {
                        "Number": 3,
                        "TestOrder": 4,
                        "Rotation": "?",
                        "Roll": 0.5,
                        "Pitch": 0.266
                    },
                    {
                        "Number": 4,
                        "TestOrder": 2,
                        "Rotation": "CW",
                        "Roll": -0.3,
                        "Pitch": -0.5
                    }
                ]
            }
        }
    }
