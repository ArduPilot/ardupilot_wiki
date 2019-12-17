.. _editing-prs:

======================================
How to Edit Someone Else's PR Directly
======================================

Occasionally, it is desirable to make some small changes to someone else's submitted PR in order to facilitate its readiness for merging. This describes one procedure for doing this.

Assumptions
===========

It is assumed that you have worked on the GitHub repository for ArduPilot before and have cloned a fork of the repository locally.

Procedure
=========

First, determine if you have been given permission to modify the PR. If so, then at the bottom of the PR on GitHub.com you will see :

"Add more commits by pushing to the ``xyz`` branch on ``username/ardupilot``"

First set up a remote to username's repository, in this case we will call it ``tempremote`` :
::

   git remote add tempremote https://github.com/username/ardupilot.git

this sets up ``tempremote`` to point to the originators repository on GitHub.

To pull the PR branch ``xyz`` down to your local computer's repository in order to modify it:

::

   git fetch tempremote  xyz

which will copy the PR's branch into a local buffer.

Then create a local branch ``username-xyz`` and which is a copy of the PR

::

   git checkout -b username-xyz tempremote/xyz


Next make your mods and commit them with a commit message explaining your changes. This message will actually be removed in the next step, but it's good to make it something meaningful if you need to reset or revert in the future.

You should also "squash" (see :ref:`git-interactive-rebase` ) your commit into the originator's before pushing to his repository yourself. At this point you can use the originator's commit message or change it if it needs. Also, you should be sure that the entire commit conforms to the ArduPilot style guidelines (see :ref:`style-guide` ).

Finally, you can force push over the originators PR to include your modifications:

::

  git push -f tempremote HEAD:xyz

The changes will be reflected in the PR on GitHub. You may now delete your ``username-xyz`` branch locally.