.. _copter-use-case-overview:

========================
Copter Use-Case Overview
========================

This article provides an overview of some of the main use cases for
Copter.

Overview
========

The Copter autopilot provides a stable flying platform that enables
precise manual and automated control over vehicle position, speed,
orientation and actions. The supported control behaviours are made
available using autopilot :ref:`flight/control modes <flight-modes>`:

-  Manual flight modes like
   :ref:`Stabilize <stabilize-mode>`,
   :ref:`Alt Hold <altholdmode>` and
   :ref:`Loiter <loiter-mode>` provide
   different types of stabilisation and make vehicles easier to fly and
   position. Other manual modes like :ref:`Follow Me <ac2_followme>` and :ref:`RTL (Return-to-Launch) <rtl-mode>`
   automate tasks that would otherwise require complex manual
   adjustments.
-  :ref:`AUTO Mode <auto-mode>` allows
   you to run complex missions that you can define using a :ref:`ground control station <common-choosing-a-ground-station>`.
-  A companion computer on the vehicle can communicate with/control
   Copter (for example, using
   `DroneKit-Python <http://python.dronekit.io/>`__) and perform
   computationally intensive low-latency tasks like computer-vision.

This stability and precision, and the flexibility in terms of manual and
automated control, make Copter the ideal platform for many UAV
applications.

This article provides an overview of some of the main use cases, with
special emphasis on those requiring photographic/video inspection or
payload delivery in difficult-to-reach places.

.. tip::

   Additional use cases and information are covered in the section
   :ref:`Use Cases and Applications <common-use-cases-and-applications>`.

Still and Video Photography
===========================

Copter allows you to easily get to locations and take pictures/video
that would otherwise be difficult (or impossible) to reach. For this
reason, photography applications are currently the most highly developed
use-case.

Copter provides a stable platform for photography, with additional
stability and independent control of camera position (relative to the
vehicle) provided by brushless camera gimbals. Copter supports
camera-friendly flight modes like :ref:`Follow Me <ac2_followme>` and allows you to
control/maintain the camera target at a specific region of interest.

Advanced systems like `3DR Solo <https://3dr.com/solo-drone/>`__
implement even more advanced vehicle/camera control ("smart shots")
using `DroneKit-Python <http://python.dronekit.io/>`__ running on a
Companion Computer.

First Person View (FPV)
=======================

*First Person View* allows you to fly your copter from the perspective
of an actual on board pilot. This use case is discussed in the topic
:ref:`First Person View (FPV) <common-fpv-first-person-view>`.

Disaster response
=================

Copters can help save lives/provide relief in the event of major
disasters (fires, flood, tornadoes, earthquakes, volcanic eruptions
etc).

They are particularly useful for search and survey tasks, and for
delivery of low-weight critical supplies, information and assistance.
They can do this relatively cheaply without putting additional lives at
risk (freeing up other resources to do actual recovery).

Search (and rescue)
===================

Copter makes an excellent platform for locating missing individuals and
groups. Vehicles can perform a grid search and take photographs for
either on-board (using a companion computer) or later analysis. Copter
can search in hard-to-reach areas, and may be used in large numbers due
to their low cost.

.. tip::

   Fixed wing vehicles have much greater range than Copter, and may be
   more suitable for searching large areas with low ground-cover.

Agricultural applications
=========================

Agricultural inspection is a growing field for UAV applications.
Examples include:

-  Tile and drainage inspections
-  Barn roof and silo inspections
-  Irrigation pivot inspections
-  Hail and cattle damage inspection for crop insurance claims
-  Scare off pest-wildlife that eat crops
-  Patrol for hunters on your private land
-  Locate missing cattle (This is where a thermal camera comes in
   handy.)
-  Video check-ins for landlords

.. tip::

   This promises to be one of the most important and earliest adopted
   civilian uses of Multicopters. One benefit is that there are fewer
   restrictions when flying over private land.

Forest fire mitigation
======================

Copter has great potential for fire monitoring and detection (with an
infrared camera, a Plane or Copter UAV can detect small camp fires even
in heavy tree cover).

Hazard/danger mitigation
========================

More generally, Copter and Plane are useful for other hazard mitigation
as a cost-effective alternative to patrolling using airplanes,
helicopters, or ground-based services.

They are already being used for shark patrols in beach areas, and there
is no reason they could not similarly be used in any other "patrol"
activity.

3D Mapping and GIS (Geographic Information Systems)
===================================================

Copter makes an effective 3D Mapping platform with a wide variety of
potential applications. For more information see the topic :ref:`3D Mapping <common-3d-mapping>`.

Inspection, Verification and Sample Collection
==============================================

Architectural and building inspection/verification are possibly the
fastest growing UAV use case - due to the obvious benefits to being able
to check construction quality and condition without having to create
expensive scaffolding and other safety infrastructure. Copter is
similarly useful for contour analysis, drainage and verifying adherence
to plans.

Copter is also useful for sample collection in difficult to reach or
hazardous areas (this requires that the vehicle is fitted with a small
probe or other sample device). The `Modcopter Sample Collection System <https://permalink.lanl.gov/object/tr?what=info:lanl-repo/lareport/LA-UR-13-23300>`__
is an excellent government-backed student project for accessing a
variety of samples.

Payload Based Applications
==========================

Copter is suitable for delivery of low-mass emergency supplies,
including flotation devices, communications devices, shark repellent
etc.

There are active investigations into other commercial applications
including crop spraying and package delivery.

Other applications
==================

Copters are being used or considered in many other applications:

-  Initial "pilot line" stringing for power lines from hilltop to hill
   top.
-  Painting, touch up and maintenance.
-  Tree trimming and spraying.
-  Building and home cleaning.

More detail and additional use case information is covered in the
section :ref:`Use Cases and Applications <common-use-cases-and-applications>`.
