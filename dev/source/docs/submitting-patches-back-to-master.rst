.. _submitting-patches-back-to-master:

=================================
Submitting Patches Back to Master
=================================

   .. image:: ../images/PullRequest_TopImage.png
       :width: 70%

Once you have a bug fix or new feature you would like to have included in the ArduPilot project
you should submit a `Pull Request <https://help.github.com/articles/about-pull-requests/>`__.
The main developers will see your changes in the `Pulls list, <https://github.com/ArduPilot/ardupilot/pulls>`__ 
review them and if all goes well they will be merged into master.

This page provides advice to make the process go smoothly.

Preparing commits
-----------------

- Commits should be **in a new branch** of your :ref:`fork <git-fork>`/:ref:`clone <git-clone>` (i.e. not "master"). These should be done on a local branch and then pushed to your web-based fork of ArduPilot.

- The new branch should be :ref:`up-to-date <git-rebase>` with `ArduPilot/master <https://github.com/ArduPilot/ardupilot>`__  and should not include any other changes.

- **Commits should be small and do just one thing.** If a change touches
  multiple libraries then there should be a separate commit per library,
  and a separate commit per vehicle directory. This is true even if it
  means that intermediate commits break the build.

- **Well-written, concise comments** are encouraged.

    Commit messages should be of the form:

    ::

        Subsystem: brief description

        Longer description...

    Example:

    ::

        APM_Control: reduce the number of parameter saves in autotune

        do not save a parameter unless it has changed by 0.1%

- **clean up your local commit history** :ref:`using interactive rebase <git-interactive-rebase>`
  (i.e. ``git rebase -i "HEAD~10"``) to re-arrange patches and fold things together. The idea is to present
  a logical set of patches for review. It can take a bit of effort to get
  used to interactive rebase, but it is definitely worth learning. Refer
  to `online resources <http://gitready.com/advanced/2009/02/10/squashing-commits-with-rebase.html>`__
  to understand how to use this tool. Also see this video: https://youtu.be/iMUWQUD_Rsg

- Each commit should involve only a single library. The commits of the changes in each library should be squashed (see :ref:`git-interactive-rebase`) into the minimal number of commits per library that allows readability and clarity when reviewing (usually only one commit). The "Tools/gittools/git-subsystems-split" script can be run on any commit spanning multiple libraries to create a single commit for each library module affected before submitting. if needed.

- Do not submit patches with commented-out code or code that is never reachable within ``#define`` s.

- Do not include git merge commits in your commit history; rebase your branch on the master branch, do not ``git pull`` changes in.

- Do not include any git ``fixup!`` commits in your PR; these are only ever supposed to be a temporary part of a git workflow

- Try to follow the :ref:`style guide <style-guide>` so your code fits in with the existing code.
  In particular, ensure your editor uses 4 spaces intead of tabs.

- Unix line endings (LF) are used. Git should take care of this
  automatically, but if you notice that you have a lot of files that show
  up as changed in ``git status`` but you didn't touch those files, you
  may need to `check to see if local git settings regarding line endings are
  correct <https://help.github.com/articles/dealing-with-line-endings>`__.

- read your changes, doing your own review. The best way to do this is
  to use the "gitk" tool. Look over your own changes critically. Make
  sure they do not include anything you don't want to go into the pull
  request. It is best to read your changes at least several hours after
  you wrote the code, and preferably the next day. Look over them
  carefully and look for any bugs.

- if you have access to a Linux build environment then build your
  modified tree using `Tools/scripts/build_all.sh <https://github.com/ArduPilot/ardupilot/blob/master/Tools/scripts/build_all.sh>`__.
  That will test that
  all the builds for different boards and vehicle types work. If you
  don't have a Linux build environment then please test the build for
  Pixhawk boards and rover, copter and plane if your changes may affect
  those vehicles.

- test your changes in :ref:`SITL <sitl-simulator-software-in-the-loop>` if possible.
  If you can't run SITL then test your changes in a real vehicle.

Submitting a pull request
-------------------------

To submit a `pull request on GitHub <https://help.github.com/articles/using-pull-requests>`__ for review and possible inclusion in the official
repository, follow these directions:

#. Open your :ref:`fork <git-fork>` on the GitHub web page select the branch from the drop-down and then push the "New Pull Request" button

   .. image:: ../images/PullRequest_InitiatePullRequest1.png
       :width: 70%
       :target: ../_images/PullRequest_InitiatePullRequest1.png

#. Check the "base fork" is `ArduPilot/ardupilot <https://github.com/ArduPilot/ardupilot>`__ and "base" is "master" and then fill in the PR's subject line and detailed description.  The detailed description should include any evidence of testing performed.
   
   Check the list of changes at the bottom of the page only includes your
   intended changes, then push the "Create pull request" button.

   .. image:: ../images/PullRequest_InitiatePullRequest2.png
       :width: 70%
       :target: ../_images/PullRequest_InitiatePullRequest2.png

Next Steps
----------

You can track the state of your PR from the `Pull Requests list <https://github.com/ArduPilot/ardupilot/pulls>`__.

PRs are more likely to be merged quickly if:

- The PR clearly states what changes in behaviour are expected
- Good testing evidence is provided.  This could be graphs of logs recorded before and after the change
- Code follows the :ref:`style guide <style-guide>`
- Each commit in the PR affects only one subsystem and the commit title is prefixed with the subsystem name (e.g. "AP_GPS: correct uBlox logging parameter descriptions").  The `Tools/gittools/git-subsystems-split <https://github.com/ArduPilot/ardupilot/blob/master/Tools/gittools/git-subsystems-split>`__ script may be useful for this
- PR passes all automated CI tests.  CI tests sometimes fail for reason unrelated to your PR (e.g. dependency failures, "flapping tests").  Please force push the PR to re-run the tests
- The PR is discussed on one of the :ref:`weekly dev calls <ardupilot-discord-server>`.  To get the PR discussed add the "DevCallTopic" or "DevCallEU" label.  If you are unable to add the label ping one of the core developers on `ArduPilot Discord <https://ardupilot.org/discord>`__ (see the "code-review" channel).  "Core developers" can be identified by the "dev-team" badge in Discord and some also appear as `top contributors in Github <https://github.com/ArduPilot/ardupilot/graphs/contributors>`__

It is very common, especially for large changes, for the core developers to ask you to modify the pull request to fit in better with the existing code base or resolve some knock-on impact that you may not have known about. Please don't take this the wrong way, we're definitely not trying to make your life difficult!

If your PR gets stuck please understand that this is likely just because ArduPilot receives a lot of code contributions and it is difficult for the core dev team to review and merge them all.  Here's some advice to get a PR "unstuck":

- Review the advice above titled "PRs are more likely to be merged quickly if"
- Attend one of the two :ref:`weekly dev calls <ardupilot-discord-server>`
- Send a note to a "core developer" in the code-review channel.  These can be identified by the "dev-team" badge in Discord and some also appear as `top contributors in Github <https://github.com/ArduPilot/ardupilot/graphs/contributors>`__
