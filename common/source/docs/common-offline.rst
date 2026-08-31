:orphan:

.. Reached from the top menu rather than a toctree, so mark it orphan and keep
.. the build warning-free.

.. _common-offline:

==============
Offline Copies
==============

Keep the documentation readable with no connection: in a hangar, in a field, or
anywhere the signal runs out.

This page offers three different things:

- **Save a wiki** downloads its documentation into this browser so that you
  can read it offline.
- **Save as .html** creates a portable file that you can open from a USB stick
  or copy to another computer.
- **Install app** adds a launcher icon and its own window. It does not download
  documentation.

No account is needed. The browser downloads the selected documentation from
this site and stores it on this device; it does not upload your documentation
or create a file in your Downloads folder. To create a file, use **Save as
.html** below.

For how any of this works, what it needs to run and how to test it, see `How
Offline Copies Work <../../dev/docs/wiki-offline-copies.html>`__ in the
developer documentation.

.. raw:: html

   <!-- The tool. Markup only:

          styling    _static/common_offline.css, copied into every wiki by
                     copy_common_source_files() from common/source/_static,
                     the same route common_theme_override.css takes. That is
                     what puts it inside each wiki's archive; it previously
                     lived in frontend/, outside the Sphinx build, so it
                     reached no archive and the panel was unstyled offline.
          behaviour  _static/common_offline_page.js      (the panel)
                     _static/common_offline_export.js    (reading the cache)
                     _static/common_offline_document_builder.js  (what the .html says)

                     Static assets for the same reason the stylesheet is: they
                     travel inside each wiki's archive, so the panel works
                     offline without the service worker having to know they
                     exist.

        Self-contained - every tag opened here is closed here. A <div> opened
        in one raw block and closed in another lands inside different section
        wrappers, which corrupts the nesting and pushes the page footer out of
        the content column.

        Follows ArduPilot's other tool front-end, custom.ardupilot.org: dark
        header strip, status badges, progress bars. -->


   <noscript><p class="apo-note apo-note-warn">This tool needs JavaScript.
     Reading the wiki does not: every page works as it always has.</p></noscript>

   <div id="storage-warning"></div>
   <p id="quota-warning" class="apo-note apo-note-warn" hidden></p>

   <div class="apo">

     <div class="apo-mode">
       <label class="apo-switch" title="Turn offline mode on or off">
         <input type="checkbox" id="offline-mode" aria-label="Offline mode" />
         <span class="apo-switch-track"></span>
       </label>
       <div>
         <strong>Offline mode: <span id="offline-mode-state">off</span></strong>
         <div class="apo-hint">Runs a service worker in this browser so the pages
           you have read, and the wikis you save, open with no connection.
           Saving a wiki turns it on. Turning it off removes everything saved
           in this browser.</div>
       </div>
     </div>
     <p id="offline-off-warning" class="apo-note apo-note-warn apo-off-warning" hidden>
       Turning offline mode off removes everything this site has saved in this
       browser (<span id="offline-off-size"></span>). Files you saved to disk
       with Save as .html are not affected.
       <span class="apo-off-actions">
         <button id="offline-off-confirm" class="apo-btn apo-btn-danger">Turn off and remove</button>
         <button id="offline-off-keep" class="apo-btn apo-btn-ghost">Keep</button>
       </span>
     </p>

     <div class="apo-head">
       <p class="apo-title">Save for Offline Use</p>
       <span class="apo-actions">
         <button id="download-cache-btn" class="apo-btn apo-btn-primary">Save selected wikis</button>
         <button id="check-btn" class="apo-btn apo-btn-ghost">Check for updates</button>
         <button id="clear-btn" class="apo-btn apo-btn-danger">Remove all</button>
       </span>
     </div>

     <div class="apo-scroll">
     <table class="apo-table">
       <thead>
         <tr>
           <th class="apo-name"><span class="apo-pick"><input type="checkbox"
                 id="select-all" title="Select every wiki"
                 aria-label="Select every wiki" /><span>Wiki</span></span>
               <label class="apo-pick apo-all-params"><input type="checkbox"
                 id="all-params"
                 aria-label="Save every parameter version of every wiki"
                 title="Save every parameter version of every wiki" /><span>all
                 params</span></label></th>
           <th class="apo-num">Size</th>
           <th class="apo-num apo-pages-h">Pages</th>
           <th class="apo-num apo-col-progress">Progress</th>
           <th class="apo-num">Status</th>
         </tr>
       </thead>
       <tbody id="wiki-rows"></tbody>
     </table>
     </div>

     <div class="apo-foot">
       <label><input type="checkbox" id="autoupdate" checked="checked" />
         <span>Update saved pages automatically</span></label>
       <div class="apo-status apo-foot-status">
         <span id="selection-total"></span>
         <span id="cache-progress" hidden="hidden"></span>
         <span id="check-result" hidden="hidden"></span>
       </div>
       <div class="apo-status apo-foot-status">
         <span id="storage-status"></span>
         <span id="build-date"></span>
       </div>
     </div>

     <div class="apo-files">
       <h3>Save a copy as a file</h3>
       <p class="apo-hint" style="margin-top:0">Built on this device from the
         wikis ticked above. Anything not saved yet is downloaded first, so
         one press is enough. Each file contains exactly the wikis you chose,
         shared images included.</p>

       <div class="apo-file">
         <button id="dl-single" class="apo-btn apo-btn-outline">Save as .html</button>
         <div class="apo-hint">A single self-contained page. Double-click it, nothing
           to install. Search works across the full text of every page. It runs
           from a USB stick, though it is large and takes a moment to open.</div>
       </div>
     </div>

   </div>

What to Expect
==============

**Saving a wiki lets you read every page in it without a connection.** Pages
come from your own device rather than the network, so they open quickly in a
hangar, on a plane, or anywhere with no signal.

It costs storage. Roughly:

===============================  ==========
Shared images (required)         about 440 MB
A vehicle wiki on top of that    4 to 75 MB
All ten wikis                    about 700 MB
===============================  ==========

The shared images are needed whichever wiki you choose, because nearly every
page uses them, so the first save is the large one. Saving a second wiki
afterwards costs only its own pages.

.. note::

   Saved pages live in your browser's storage for this site. Clearing site data
   removes them, and you would need to download again.

How It Works
============

The wiki is stored by your browser and served from your own device. No account
is involved, and the pages you read offline are the same files the site serves
normally. While automatic updates are enabled, the wiki may check this site for
changed pages while it is open.

Offline mode is off until you turn it on with the switch at the top of this
page; saving a wiki turns it on too. Off, the site behaves exactly as it
always has and nothing runs in the background. Turning it off removes the
saved wikis and every cached page from this browser and returns the site to
how it was; you are asked first, and told how much will go. Files you saved
with **Save as .html** are not affected.

Install as an App
=================

.. note::

   Installing downloads nothing, and is not needed to read offline. Saving pages
   works in an ordinary browser tab. It gives the wiki its own window and a
   launcher icon. Depending on your browser, installing may also help it retain
   saved pages when the device runs low on space.

.. raw:: html

   <div class="apo-install-row">
     <button id="ap-install-app" class="apo-btn apo-btn-outline" hidden="hidden">Install app</button>
     <span id="install-state" class="apo-hint"></span>
   </div>

   <script src="../_static/common_offline_document_builder.js" defer="defer"></script>
   <script src="../_static/common_offline_export.js" defer="defer"></script>
   <script src="../_static/common_offline_unpack.js" defer="defer"></script>
   <script src="../_static/common_offline_update.js" defer="defer"></script>
   <script src="../_static/common_offline_page.js" defer="defer"></script>

[copywiki destination="copter,plane,rover,sub,blimp,antennatracker,dev,planner,planner2,ardupilot,mavproxy"]
