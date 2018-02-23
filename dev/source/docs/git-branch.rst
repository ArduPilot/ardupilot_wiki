.. _git-branch:

================================
Branching and changing some code
================================

Branches are a way to isolate different paths of development, which can
then be combined in a single (often named "master") branch. Refer to
`this short guide <http://gitready.com/beginner/2009/01/25/branching-and-merging.html>`__
for more information, in particular the resources under the :ref:`Learning Git <where-to-get-the-code_learning_git>` section. In this section of
the tutorial, you'll make a branch and change some code.

Branch names are up to you, but it can be helpful to choose short
descriptive names. The branch name used for this tutorial is
"apm_git_tutorial"

**OSX/Linux Terminal commands**

These commands assume your current working directory is the root of the
repository you cloned.

.. tip::

   These same commands can be used in Windows too if you use a command
   line git client (e.g. The "Git Shell" utility that was installed with
   GitHub for Windows.

#. Create a branch.

   ::

       git checkout -b apm_git_tutorial

#. Change some code. For this tutorial, open the
   **Tools/GIT_Test/GIT_Success.txt** in your preferred text editor,
   and put your name at the end of the file. Save the file.
#. See that you've changed some files by checking the status:

   ::

       git status

#. Commit your work to the branch to add your changes to the git
   history:

   ::

       git add Tools/GIT_Test/Git_Success.txt
       git commit -m 'Added name to GIT_Success.txt'

   Please see below for further information regarding conventions for
   committing work that you expect to be integrated into the official
   releases. When you commit, you're required to add a log message
   explaining what you did in the commit. See :ref:`Submitting Patches Back to Master <submitting-patches-back-to-master>` for more
   information on how to do this, and for the purpose of this tutorial,
   you can just use a single line stating: "Added name to
   GIT_Success.txt":

#. Push your branch to GitHub. This will copy your work on your local
   branch to a new branch on GitHub. Pushing branches is a precondition
   for collaborating with others on GitHub or for submitting patches
   back to the official releases.  It is assumed origin is the remote
   name of your fork of the github repository.

   ::

       git push origin HEAD:apm_git_tutorial

**Windows (GitHub GUI)**

In the Github GUI you used to clone the repository, you can create a
branch and commit it.

#. Create a branch. In the GitHub for Windows application, click on the
   'master' button in the upper right corner of the window.

   .. image:: ../images/APM-Git-Github-Windows-Branch.jpg
       :target: ../_images/APM-Git-Github-Windows-Branch.jpg

   Enter 'apm_git_tutorial' and click the "+ create branch:
   apm_git_tutorial" dropdown.

   .. figure:: ../images/GitHubForWindowsClient_CreateBranch.png
      :target: ../_images/GitHubForWindowsClient_CreateBranch.png

      Create a new branch in Github for Windows

#. Change some code. For this tutorial, open the
   **Tools/GIT_Test/GIT_Success.txt** in your preferred text editor,
   and put your name at the end of the file. Save the file.
#. The Git for Windows client shows the changed file and has a place
   where you can enter a summary and description of the change. For the
   purpose of this tutorial, you can just use a single line stating:
   "Added name to GIT_Success.txt"

   .. figure:: ../images/GitHubForWindowsClient_CommitingChange.png
      :target: ../_images/GitHubForWindowsClient_CommitingChange.png

      Github for windows: Commit Change

   .. note::

      Please see :ref:`Submitting Patches Back to Master <submitting-patches-back-to-master>` for further
         information regarding conventions for committing work that you expect
         to be integrated into the official releases (this will include a much
         more detailed commit message). 

#. Commit your work to the branch by pressing the **Commit to
   apm_git_tutorial** link.
#. Push your local branch to GitHub (pushing branches is a precondition
   for collaborating with others on GitHub or for submitting patches
   back to the official releases). In the client you can do this by
   pressing the **Publish** link:

   .. figure:: ../images/GitHubForWindowsClient_Publish_Push.png
      :target: ../_images/GitHubForWindowsClient_Publish_Push.png

      GitHub for Windows Client: Pushing changes

Congratulations! This is bulk of the normal process you'd follow when
working on code to submit back to the official project. The next step is
to :ref:`submit a pull request <submitting-patches-back-to-master>`
so your changes can be considered for addition to the main project.
