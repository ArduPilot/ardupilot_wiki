.. _how-to-use-the-auth-command-to-sign-a-pixhawk-board-with-your-certificate-of-authenticity:

===========================================================================================
How to use the "auth" command to sign a Pixhawk Board with your Certificate of Authenticity
===========================================================================================

The essence of this process is an RSA private/public key pair and a
signing process that uses these keys to put some unique information onto
every Pixhawk board.

Public/Private Key/s, SD Card, and Logging
==========================================

The "auth" command will

-  read a properly prepared public/private key data from the SD card
-  use the key off the SD card to create the Certificate-Of-Authenticity
   (COA) in the One-Time_programmable (OTP) ROM.
-  log the results to a log file on the SD card, for your records. 
   (optional)

Bootloader
==========

The bootloader must identify itself as revision 4 or  later for this to
work.

eg: px_uploader.py should say something like: "Found board xxxx
bootloader rev 4 on /xxxxxxx "

Firmware
========

The firmware must contain a directory Firmware/src/systemcmds/auth and
have a recent "auth.c".

-  typically made and uploaded with "make px4fmu-v1_auth upload", or
   similar.
-  verify you have a suitable version of the firmware loaded on your PX4
   by connecting to the nsh in the usual way, and typing 'auth'[enter]
-  the "auth" command can do a bunch of stuff related to
   reading/writing/verifying/signing/logging of OTP data.    It's the
   main tool you'll use ( see below).  It can read/write public/private
   key data from SD card, or it can use a "hardcoded" TEST version.

Preparing SD card (one time only)
=================================

-  Using a tool on linux or OSX called *ssh-keygen* make a new "pair" of
   1024bit RSA keyfiles like this:

   ::

       ssh-keygen -t rsa -b 1024  -f rsa1024

-  Generating public/private rsa key pair.
-  Enter passphrase (empty for no passphrase):  [just press enter]
-  Enter same passphrase again: [just press enter, again]

.. warning::

   This is
      your *PRIVATE* key  - do not share this file.  back it up and keep
      it safe.

.. tip::

   This is your
      *PUBLIC* key  - share it with all GCS makers, etc.

-  Format the private key file to suit the PX4:
-  Copy the rsa1024 to a new file, called "privatekey.txt"
-  Edit the file with a text editor to remove the first and last lines
   of the file (they say ``-----BEGIN RSA PRIVATE KEY-----`` and
   ``-----END RSA PRIVATE KEY-----``) and save it.
-  Copy the **privatekey.txt** to a SD card which you will insert into a
   PX4 that is setup as per above.  Do not use this method for the
   public key.  See below.
-  Make the public key file on the SD card:

   -  With the SD card inserted, and the Pixhawk booted, use the *nsh
      shell*, and type:

      ::

          auth -m

      this will use the **privatekey.txt** on the SD card, and create a
      **publickey.txt** on the SD card (this file is needed for many of
      the auth commands to work).

Validate it works
=================

-  There are lots of options to the "auth" command that you can use to
   test your configuration.

   .. warning::

      The only really \*hazardous\* options are ``-w`` and ``-k``.
      Avoid these till you are sure everything else seems to be going well.

-  It is a good idea to reboot each time you use the 'auth' command, as
   it's very aggressive on RAM usage.
-  When you are sure you have everything OK, run ``auth -k -l``  (it
   will write a new COA to OTP, and lock it) and optionally ``auth -v``
   to verify it worked.

Automate running the 'auth' script  from the SD card
====================================================

-  Make an "etc" folder on the SD card if one is not already there.
-  Make a rc, or **rc.txt** file ( either works ) on the SD card, in the
   */etc* folder if one is not already there.
-  Edit the **rc.txt** file, and put the ``auth -k -l`` command in the
   file as you wish it to be run.
-  (When booted with this card inserted it will make the PX4 flash the
   OTP area with the COA and log the results to **OTPCertificates.log**)
