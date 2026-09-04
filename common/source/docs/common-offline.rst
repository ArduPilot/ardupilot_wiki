:orphan:

.. Reached from the top menu rather than a toctree, so mark it orphan and keep
.. the build warning-free.

.. _common-offline:

==============
Offline Copies
==============

We have implemented a way to easily save offline copies of the ArduPilot wiki
so that key documentation can still be accessed when there is no internet
connection available. You can select which parts of the wiki you would like to
save and never be left scratching your head trying to remember how to
configure a vehicle or plan a mission.

Having the ability to access the trove of information from the wiki at the
field improves the ability to troubleshoot issues and gives you the confidence
to fly safely.

There are three ways you can go about making offline copies:

1. You can download all, or parts, of the wiki **into the browser**, which
   will be cached and available to use with no internet connection. (But if
   you run out of storage the browser can delete it without warning you.)
2. You can **install the wiki as an app**, like a normal app such as Google
   Chrome. This also makes it less likely that the browser will remove the
   saved wikis without warning.
3. You can save all, or parts, of the wiki **as a .html file**, which you can
   save to a USB stick and copy to another device.

No account is needed. The browser downloads the selected documentation from
the site and stores it on this device; it does not upload your documentation
or create a file in your Downloads folder unless you download the HTML.

To learn more about how any of this works, what it needs to run, and how to
test it, see `How Offline Copies Work
<../../dev/docs/wiki-offline-copies.html>`__ in the developer documentation.

Implementing Offline Copies
===========================

Select the parts of the wiki that you wish to access offline in the table
below and click **Save selected wikis**. You will find that if you turn off
your internet connection, you will still be able to access the parts of the
wiki that you have selected.

Click **Check for updates** periodically to ensure that you have the most up
to date version of the parts of the wiki that you have saved. You can select
**Remove all** to clear the cache if you no longer need access to the saved
wiki.

You can turn on **Offline mode**, which runs a service worker in the browser
so the pages you have read, and the wikis you save, open with no connection.
Turning it off removes everything saved in the browser.

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

     <div class="apo-subhead">Install as an app</div>
     <div class="apo-install">
       <div class="apo-install-row">
         <button id="ap-install-app" class="apo-btn apo-btn-primary">Install app</button>
         <span id="install-state" class="apo-hint"></span>
       </div>
       <div class="apo-hint">If your device is running low on storage, your
         browser may unexpectedly clear the cached data, such as your offline
         wiki. Installing as an app makes that less likely, and gives the wiki
         its own window and a launcher icon. It downloads nothing by
         itself.</div>
       <!-- Screenshots of the install flow go here. -->
     </div>

     <div class="apo-subhead">Save as a static HTML page</div>
     <div class="apo-files">
       <p class="apo-hint" style="margin-top:0">Built on this device from the
         wikis ticked above. Anything not saved yet is downloaded first, so
         one press is enough. Each file contains exactly the wikis you chose,
         shared images included.</p>

       <div class="apo-file">
         <button id="dl-single" class="apo-btn apo-btn-primary">Save as .html</button>
         <div class="apo-hint">A single self-contained page. Double-click it, nothing
           to install. Search works across the full text of every page. It runs
           from a USB stick, though it is large and takes a moment to open.</div>
       </div>
     </div>

   </div>

   <script src="../_static/common_offline_document_builder.js" defer="defer"></script>
   <script src="../_static/common_offline_export.js" defer="defer"></script>
   <script src="../_static/common_offline_unpack.js" defer="defer"></script>
   <script src="../_static/common_offline_update.js" defer="defer"></script>
   <script src="../_static/common_offline_page.js" defer="defer"></script>

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

**It is faster, too.** Roughly:

=============================  ========================================
A page you have read           opens in a blink instead of a second
The huge parameter lists       about three times faster to appear
=============================  ========================================

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

Things to Consider
==================

- **The browser can evict saved wikis** if the device runs very low on
  storage, without warning you first. Installing as an app helps, and the
  storage line under the table says whether this browser has granted
  persistent storage.
- **Clearing this site's data removes everything saved here**, as does
  clearing browsing data in most browsers. Offline mode turns off with it.
- **Private and incognito windows keep nothing** once the last one closes.
- **A saved wiki is a snapshot.** Leave automatic updates on, or press Check
  for updates now and then, so your copy follows the site.

[copywiki destination="copter,plane,rover,sub,blimp,antennatracker,dev,planner,planner2,ardupilot,mavproxy"]
