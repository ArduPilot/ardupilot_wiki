.. _submitting-patches-back-to-master:

=================================
Submitting Patches Back to Master
=================================

Once you have a bug fix or new feature you would like to have included
in the APM projects, you should submit a `Pull Request <https://help.github.com/articles/using-pull-requests>`__.

The main developers will see your changes in the `Pulls list, <https://github.com/ArduPilot/ardupilot/pulls>`__ 
review them and if all goes well they will be merged into master.

Preparing commits
-----------------

Commits should be made to your fork/clone of the project in a new branch
(i.e. not "master") that is up-to-date with the ardupilot/ardupilot
master branch does and not include any other changes. See :ref:`Working with the ArduPilot Project Code <where-to-get-the-code>` for
instructions on how to correctly update your working branch.

**Commits should be small, and do just one thing.** If a change touches
multiple libraries then there should be a separate commit per library,
and a separate commit per vehicle directory. This is true even if it
means that intermediate commits break the build.

Unix line endings (LF) are used. Git should take care of this
automatically, but if you notice that you have a lot of files that show
up as changed in ``git status`` but you didn't touch those files, you
may need to `check to see if local git settings regarding line endings are correct <https://help.github.com/articles/dealing-with-line-endings>`__.

**Well-written, concise comments are good** and are encouraged.

Do not submit patches with commented-out code, or code that is never
reachable within ``#define`` s.

Before submitting code to the official repository, **clean up your local
commit history**. When submitting patches the convention is to `use an interactive rebase <https://help.github.com/articles/interactive-rebase>`__ (eg.
"git rebase -i HEAD~10") to re-arrange patches and fold things together,
so the patch set it "cleaned up" for submission. The idea is to present
a logical set of patches for review. It can take a bit of effort to get
used to interactive rebase, but it is definitely worth learning. Refer
to `online resources <http://gitready.com/advanced/2009/02/10/squashing-commits-with-rebase.html>`__
to understand how to use this tool.

Try to follow the style conventions of the existing code.  In
particular, check that your :ref:`editor uses 4 spaces instead of tab <editing-the-code>`.

Log messages
------------

Commit messages should be of the form: ::

    Subsystem: brief description

    Longer description...

Example: ::

    APM_Control: reduce the number of parameter saves in autotune

    don't save a parameter unless it has changed by 0.1%

Before you submit a pull request
--------------------------------

Before submitting a pull request please do the following

-  Use git rebase on current master. Your changes should not include any
   merges, and should contain just the patches you want as the last
   patches in your tree.
-  read your changes, doing your own review. The best way to do this is
   to use the "gitk" tool. Look over your own changes critically. Make
   sure they don't include anything you don't want to go into the pull
   request. It is best to read your changes at least several hours after
   you wrote the code, and preferably the next day. Look over them
   carefully and look for any bugs.
-  if you have access to a Linux build environment then build your
   modified tree using Tools/scripts/build_all.sh. That will test that
   all the builds for different boards and vehicle types work. If you
   don't have a Linux build environment then please test the build for
   APM2 and PX4 and rover, copter and plane if your changes may affect
   those environments.
-  test your changes in SITL if possible. If you can't run SITL then
   test your changes in a real vehicle.

Submitting pull requests
------------------------

To submit a patch for review and possible inclusion in the official
repository, follow these directions:

#. Clean up your local commit history with ``git rebase -i`` as
   described above.
#. Push your local branch to GitHub. If using the GitHub for Windows
   client, this screenshot shows what your commit/sync screen could look
   like:

   .. image:: ../images/PullRequest_CommitChanges.png
       :target: ../_images/PullRequest_CommitChanges.png

#. Open your clone's repository on the GitHub web page and `Create a pull request on GitHub <https://help.github.com/articles/using-pull-requests>`__.
   You'll be making a pull request from your fork/branch to the
   ardupilot/master repository. If using the GitHub for Windows client,
   one convenient way to navigate to the repository/branch is to click
   one one of your commits and click the "github" (view this commit on
   github.com) button:

   .. image:: ../images/PullRequest_OpenCloneOnGitHubWebPage.png
       :target: ../_images/PullRequest_OpenCloneOnGitHubWebPage.png
   
#. On the right side of the web page select "Pull Request", and then
   select the green "New pull request" button:

   .. image:: ../images/PullRequest_InitiatePullRequest1.png
       :target: ../_images/PullRequest_InitiatePullRequest1.png

#. The comparison should between ardupilot:master and the the new branch
   you created for the feature but it has probably defaulted to your
   clone's master branch so click the "Edit" button and change it to the
   correct branch:

   .. image:: ../images/PullRequest_InitiatePullRequest2.png
       :target: ../_images/PullRequest_InitiatePullRequest2.png
   
#. Check the list of change at the bottom of the page only includes your
   intended changes, then press "Click to create pull request for this
   comparison".
#. Let the dev team know! Post on the
   `drones-discuss <https://groups.google.com/forum/#!forum/drones-discuss>`__
   mailing list so the discussion and code review is easily publicly
   available.

It is very common, especially for large changes, for the main developers
to ask you to modify you pull request to fit in better with the existing
code base or resolve some knock-on impact that you may not have known
about.  Please don't take this the wrong way, we're definitely not
trying to make your life difficult!
