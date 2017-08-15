.. _release-procedures:

=========================
Copter Release Procedures
=========================

This page outlines the steps that are normally followed for a Copter
release. Listed mostly as a reference so that we don't forget the steps.

Alpha Testing
=============

The `AutoTester <http://autotest.ardupilot.org/>`__ that runs after each
commit and highlights issues that it's been setup to test.

Developers and some Alpha testers perform intermittent tests of master
especially after new features have been added.

Beta Testing Release Candidates
===============================

Release candidates are made available to beta testers through the
mission planner's Beta Firmware's link.  The Mission Planner Beta
Firmware link pulls the version from the `Copter/beta directory of firmware.ardupilot.org. <http://firmware.ardupilot.org/Copter/beta/>`__

Someone with ardupilot GitHub commit access (normally Randy) makes the
new firmware available through the following steps:

**For Pixhawk/PX4:**

Open a Git Bash terminal in the ardupilot repository:

a) ``git checkout master`` (or the ArduCopter-3.1.2 branch if this is a
release candidate for a patch release)

b) update ArduCopter.pde's firmware version and ReleaseNotes.txt

in ardupilot, PX4Firmware and PX4NuttX directories:

c) ``git show ArduCopter-beta`` and record the old git tags in case a
backout is required

d) ``git tag -d ArduCopter-beta``

e) ``git push origin :refs/tags/ArduCopter-beta``

f) ``git tag ArduCopter-beta HEAD``

g) ``git push origin ArduCopter-beta``

**For APM1/APM2:**

Open a Git Bash terminal in the ardupilot repository:

a) ``git checkout master`` (or the ArduCopter-3.1 branch if this is a
release candidate for a patch release)

b) update ArduCopter.pde's firmware version and ReleaseNotes.txt (if not
already done above for Pixhawk)

in the ardupilot directory:

c) ``git show ArduCopter-beta-apm1`` and record the old git tags in case
a backout is required

d) ``git show ArduCopter-beta-apm2``

e) ``git tag -d ArduCopter-beta-apm1``

f) ``git tag -d ArduCopter-beta-apm2``

g) ``git push origin :refs/tags/ArduCopter-beta-apm1``

h) ``git push origin :refs/tags/ArduCopter-beta-apm2``

i) ``git tag ArduCopter-beta-apm1 HEAD``

j) ``git tag ArduCopter-beta-apm2 HEAD``

k) ``git push origin ArduCopter-beta-apm1``

l) ``git push origin ArduCopter-beta-apm2`` Commit a small change (`like this one <https://github.com/ArduPilot/ardupilot/commit/a38e00c048d705085782912442a9a019db4304d0>`__)
to the ardupilot directory which will cause the firmware to be rebuilt.

**Check the versions are available**

Open the Mission Planner's Initial Setup > Install Firmware page and
click the "Beta firmwares" link and ensure that the version displayed
below each multicopter icon has updated.

.. image:: ../images/ReleaseProcedures_MPBetaFirmwares.jpg
    :target: ../_images/ReleaseProcedures_MPBetaFirmwares.jpg

**Communicating the new release candidate**

Let Beta testers know the new version is available by emailing the
drones-discuss@googlegroups.com and arducoptertesters@googlegroups.com.
Include the contents of the ReleaseNotes.txt so people know what has
changed.

Post a note into the latest "Copter Released!" thread (`like this one for AC3.1 <http://diydrones.com/forum/topics/arducopter-3-1-released>`__)
and if this is the first release candidate for a new version create a
new discussion on DIYDrones (`like this one for AC3.2 <http://diydrones.com/forum/topics/arducopter-3-2-beta-testing>`__)
to concentrate the discussion.

**Issue Tracking**

Issues reported by Beta Testers via the DIYDrones discussion or the
arducoptertesters email list are tracked by the lead Tester (Marco) and
developer (Randy) perhaps using a google spreadsheet (`like this one <https://docs.google.com/spreadsheets/d/1yrYKJ-Txf5DBbEI7x4sk1p0Gts-5gjXCoiIdAyfnL7M/edit#gid=0>`__).
They are then investigated by one of the developers (usually Randy,
Leonard, Jonathan or Rob) and if it's determined to be an issue it's
added to the regular `Issues List <https://github.com/ArduPilot/ardupilot/issues?labels=ArduCopter&state=open>`__.
Sometimes beta testers directly log an issue into the issues list and
this is ok but it risks the issues list becoming a support forum so it's
better that it is investigated before being added to the issues list. 
Someone with edit access on the issues list (usually Randy) should set
the "Milestone" on the issue to the current release if we plan to
resolve the issue before the official release.

Final Release
=============

**Branching and tagging before the release**

Approximately 2 to 3 weeks before the final release a new branch is
created in the ardupilot repository. This protects the final release
from changes that could invalidate the testing and also avoids
disrupting development for future releases.

In a Git Bash terminal in the ardupilot repository:

``git checkout -b ArduCopter-3.2`` (replace "ArduCopter-3.2" with the
new release name)

``git push``

Tags are added to the PX4Firmware and PX4NuttX repositories so that we
can be sure which commits were included in the release:

``git tag ArduCopter-3.2`` (replace "ArduCopter-3.2" with the new
release name)

``git push origin ArduCopter-3.2``

**Get a video ready**

Ask Marco or other members of the testing team to provide videos which
will be included in the final release discussion.

**Deciding to release**

The "ok to release" decision is made by the lead developer (Randy) after
a discussion on the weekly dev call, a review of the outstanding issues
and after getting the "ok" from the lead tester (Marco).

**Release Steps**

Someone with commit access (usually Randy) does the following:

**For Pixhawk/PX4:**

Open a Git Bash terminal in the ardupilot repository:

a) ``git checkout ArduCopter-3.2`` ("ArduCopter-3.2" should be replaced
with the branch name for the release)

b) update ArduCopter.pde's firmware version and ReleaseNotes.txt

in ardupilot, PX4Firmware and PX4NuttX directories:

c) ``git show ArduCopter-stable`` and record the old git tags in case a
back-out is required

d) ``git tag -d ArduCopter-stable``

e) ``git push origin :refs/tags/ArduCopter-stable``

f) ``git tag ArduCopter-stable HEAD``

g) ``git push origin ArduCopter-stable``

h) ``git tag ArduCopter-3.2-px4 HEAD`` (where "3.2" should be replaced
with the release number)

i) ``git push origin ArduCopter-3.2-px4``

**For APM1/APM2:**

Open a Git Bash terminal in the ardupilot repository:

a) ``git checkout ArduCopter-3.2`` ("ArduCopter-3.2" should be replaced
with the branch name for the release)

b) ``git show ArduCopter-stable-apm1`` and record the old git tags in
case a back-out is required

c) ``git show ArduCopter-stable-apm2`` and record the old git tags in
case a back-out is required

d) ``git tag -d ArduCopter-stable-apm1``

e) ``git tag -d ArduCopter-stable-apm2``

f) ``git push origin :refs/tags/ArduCopter-stable-apm1``

g) ``git push origin :refs/tags/ArduCopter-stable-apm2``

h) ``git tag ArduCopter-stable-apm1 HEAD``

i) ``git tag ArduCopter-stable-apm2 HEAD``

j) ``git push origin ArduCopter-stable-apm1``

k) ``git push origin ArduCopter-stable-apm1``

l) ``git tag ArduCopter-3.2-apm HEAD`` (where "3.2" should be replaced
with the release number)

m) ``git push origin ArduCopter-3.2-apm``

**Check the new versions are available**

Open the Mission Planner's Initial Setup > Install Firmware page and
ensure that the version displayed below each multicopter icon has
updated.

**Communicating the Release**

Let testers and developers know the release has completed by emailing
the drones-discuss@googlegroups.com and
arducoptertesters@googlegroups.com. In general there should be no
changes from the final release candidate.  Include the full list of
changes since the last official release which can be taken from the
ReleaseNotes.txt.

Create a new "ArduCopter Released!" thread (`like this one for AC3.1 <http://diydrones.com/forum/topics/arducopter-3-1-released>`__)
including videos from the beta testers and stand by for any support
issues that may arise.

Didn't find what you are looking for?
=====================================

If you think of something that should be added to this site, please
`open an issue <https://github.com/ArduPilot/ardupilot/issues>`__ or
post a comment on the
`drones-discuss <https://groups.google.com/forum/#!forum/drones-discuss>`__
mailing list.
