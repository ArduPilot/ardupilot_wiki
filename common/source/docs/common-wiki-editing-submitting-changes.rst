.. _common-wiki-editing-submitting-changes:

=================================
Wiki Editing - Submitting Changes
=================================

Once you have completed changes to the wiki on your local machine and tested they are rendered correctly, follow the instructions below to submit a PullRequest to have them included in the wiki.  This is the same :ref:`PR process used for the flight code <dev:submitting-patches-back-to-master>`

.. note:: 

    If you are not familiar with Git and GitHub please refer to the :ref:`Using Git <dev:where-to-get-the-code>` pages to learn the basics.

There are a number of way of using Git/Github - including a number of different GUI and command line tools.  The typical process for working with Git on the command line is:

#. :ref:`Rebase <git-rebase>` your repo's master branch on the upstream ardupilot_wiki repo's master branch:

   - The very first time a "remote" pointing back to the upstream ardupilot_wiki repository must be created

     .. code-block:: bash

       git remote add upstream https://github.com/ArduPilot/ardupilot_wiki.git

   - Switch to your repo's master branch

     .. code-block:: bash

       git checkout master

    .. note:: 

        If the above command fails because of changes made to some files, temporarily "stash" the changes by typing ``git stash save my-changes`` and then later re-applied the changes by typing ``git stash apply stash@{0}``

   - Pull in the changes from upstream/master to your master branch

     .. code-block:: bash

       git fetch upstream
       git rebase upstream/master

     .. note:: Be sure to repeat these first steps if you make new changes in the future, to assure that your local repo is up to date


#. Create a branch in your local clone for your changes

   .. code-block:: bash

       git checkout -b my_well_named_branch

#. Make any changes needed and test them locally

#. Add (aka "Stage") and Commit your changes:

   .. code-block:: bash

       git add the_name_of_file_you_changed.rst
       git commit -m "A short explanation of the change"

#. Push your commits to your fork on GitHub

   .. code-block:: bash

       git push --set-upstream origin my_well_named_branch

#. Open your clone's repository on the GitHub web page and `Create a pull request on GitHub <https://help.github.com/articles/about-pull-requests/>`__. You'll be making a pull request from your fork/branch on GitHub to the    ardupilot_wiki/master repository. If using the GitHub for Windows client, one convenient way to navigate to the repository/branch is to click one one of your commits and click the "github" (view this commit on github.com) button:

   .. image:: ../../../images/PullRequest_OpenWikiCloneOnGitHubWebPage.png
       :target: ../_images/PullRequest_OpenWikiCloneOnGitHubWebPage.png

#. On top of the web page select the "Pull Request" tab page, and then select the green "New pull request" button:

   .. image:: ../../../images/PullRequest_InitiateWikiPullRequest.png
       :target: ../_images/PullRequest_InitiateWikiPullRequest.png

#. The comparison should be between ardupilot_wiki:master and the new branch you created. The website probably has defaulted to your clone's master branch so click the "compare" combo box and change it to the correct branch:

   .. image:: ../../../images/PullRequest_InitiateWikiPullRequest2.png
       :target: ../_images/PullRequest_InitiateWikiPullRequest2.png

#. Check the list of change at the bottom of the page only includes your
   intended changes, then press "Create pull request".


How to get changes approved
===========================

All changes to the wiki are `reviewed <https://github.com/ArduPilot/ardupilot_wiki/pulls>`__ by the wiki "maintainers" to help reduce the chance of misleading or incorrect information being posted.  Feel free to post comments in the PullRequest and/or attend the :ref:`weekly dev meeting <dev:ardupilot-discord-server>` to escalate getting your changes submitted.

.. tip::

   To update your fork of the repository, both locally and on GitHub, you may repeat the first three steps of section one and then push the master branch:

   #. git remote add upstream https://github.com/ArduPilot/ardupilot_wiki.git  (If you never did before)
   #. git checkout master
   #. git fetch upstream
   #. git rebase upstream/master
   #. git push -f origin master


[copywiki destination="copter,plane,rover,planner,planner2,antennatracker,dev,ardupilot,mavproxy"]
