.. _git-rebase:

======================================
Rebasing: keeping your code up to date
======================================

As you develop, the (original) master branch of the ArduPilot repository
is likely to be updated, and you should `keep your fork and your local branches up to date <http://robots.thoughtbot.com/keeping-a-github-fork-updated>`__.
Rebasing allows you to re-apply your changes on top of the latest
version of the original repo, making it much easier for the project to
merge them.

The following commands can be used to rebase your fork of the project to
the "upstream master" (main project repo). You can enter these commands
direct into a Linux/OSX Terminal. If using GitHub for Windows, launch
the "Git Shell" utility that was installed with GitHub for Windows
application.

#. Navigate to your ardupilot git repository.

   ::

       cd <ardupilot-path>

#. Ensure you are looking at your master branch

   ::

       git checkout master

#. Ensure your repository is connected to the upstream repository you
   forked from.

   ::

       git remote add upstream https://github.com/ArduPilot/ardupilot.git

#. Fetch changes from the upstream master.

   ::

       git fetch upstream

#. Rebase your current branch from the upstream master.

   ::

       git rebase upstream/master

#. Ensure your repository is connected to the your repository on github.

   ::

       git remote add origin https://github.com/your-github-account-name/ardupilot.git

#. Now push the updated master to your github repository

   ::

       git push origin master
