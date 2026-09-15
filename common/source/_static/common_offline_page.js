/*
 * [copywiki destination="copter,plane,rover,sub,blimp,antennatracker,dev,planner,planner2,ardupilot,mavproxy"]
 *
 * The Offline page's panel. Lists the wikis from offline-manifest.json, saves
 * a wiki by fetching its archive and unpacking it into Cache Storage (one cache
 * per wiki, completion marker written last), checks for updates by build id,
 * removes saved copies, and starts the single-file export. State is read from
 * Cache Storage on every render, never remembered.
 */
(function (global) {
  'use strict';

  // Answers "is the page running the code just deployed" at runtime.
  var VERSION = 'verified-bytes-1';
  global.ArduPilotOfflineVersion = VERSION;


  // Fallback rows until the manifest arrives.
  var COMMON = { id: 'common', name: 'Shared images (required)', mb: 442, images: 2398, pages: 28, required: true };
  var WIKIS = [
    { id: 'copter', name: 'Copter', mb: 74, pages: 860 },
    { id: 'dev', name: 'Developer', mb: 52, pages: 313 },
    { id: 'plane', name: 'Plane', mb: 42, pages: 829 },
    { id: 'rover', name: 'Rover', mb: 32, pages: 761 },
    { id: 'sub', name: 'Sub', mb: 15, pages: 653 },
    { id: 'blimp', name: 'Blimp', mb: 12, pages: 292 },
    { id: 'planner', name: 'Mission Planner', mb: 12, pages: 76 },
    { id: 'mavproxy', name: 'MAVProxy', mb: 7, pages: 115 },
    { id: 'planner2', name: 'APM Planner 2', mb: 5, pages: 43 },
    { id: 'antennatracker', name: 'Antenna Tracker', mb: 4, pages: 55 }
  ];

  // Overridden by the manifest's artifact_base.
  var ARTIFACT_BASE = '/offline';

  var PAGE_CACHE_PREFIX = 'ardupilot-pages-';
  var OFFLINE_CACHE_PREFIX = 'ardupilot-offline-';
  var COMPLETE_MARKER = '/__ap_complete__';
  // Kept in step with FOLD_INTO_COMMON (build) and FOLDED_INTO_COMMON
  // (sw.js, common_offline_unpack.js).
  var FOLDED_INTO_COMMON = ['ardupilot'];
  var AUTOUPDATE_KEY = 'ap-autoupdate';
  // Quota estimates are fuzzed, and unpacking needs working room.
  var HEADROOM = 1.5;

  // Build id of the manifest currently published, filled in on load.
  var CURRENT_BUILD = null;

  function el(id) { return document.getElementById(id); }

  // The worker memoises which caches exist; tell it after every change.
  function notifyWorkerCachesChanged() {
    try {
      if (navigator.serviceWorker && navigator.serviceWorker.controller) {
        navigator.serviceWorker.controller.postMessage({ type: 'CACHES_CHANGED' });
      }
    } catch (err) { /* no controller yet; nothing is memoised to invalidate */ }
  }

  function fmt(bytes) {
    var m = bytes / 1048576;
    return m >= 1024 ? (m / 1024).toFixed(1) + ' GB' : Math.round(m) + ' MB';
  }

  // Progress card for update checks. Styled in common_offline.css.
  var toastEl = null, toastHideTimer = null;

  function toast(opts) {
    if (typeof document === 'undefined' || !document.body) { return; }
    if (!toastEl) {
      toastEl = document.createElement('div');
      toastEl.className = 'ap-toast';
      toastEl.setAttribute('role', 'status');
      toastEl.setAttribute('aria-live', 'polite');
      toastEl.innerHTML =
        '<div class="ap-toast-title"></div>' +
        '<div class="ap-toast-msg"></div>' +
        '<button type="button" class="ap-toast-action apo-btn apo-btn-primary" hidden></button>' +
        '<div class="ap-toast-track"><div class="ap-toast-bar"></div></div>';
      document.body.appendChild(toastEl);
    }
    if (opts.mode === 'hide') {
      toastEl.classList.remove('ap-toast-show');
      return;
    }
    if (toastHideTimer) { clearTimeout(toastHideTimer); toastHideTimer = null; }

    toastEl.querySelector('.ap-toast-title').textContent = opts.title || '';
    toastEl.querySelector('.ap-toast-msg').textContent = opts.msg || '';

    var action = toastEl.querySelector('.ap-toast-action');
    if (action) {
      if (opts.action && opts.onAction) {
        action.textContent = opts.action;
        action.hidden = false;
        action.onclick = function () {
          action.disabled = true;
          opts.onAction();
        };
      } else {
        action.hidden = true;
        action.onclick = null;
      }
    }
    var track = toastEl.querySelector('.ap-toast-track');
    var bar = toastEl.querySelector('.ap-toast-bar');
    toastEl.classList.remove('ap-toast-done');
    track.classList.remove('ap-toast-sweep');

    if (opts.mode === 'sweep') {
      track.classList.add('ap-toast-sweep');
    } else if (opts.mode === 'progress') {
      var pct = Math.max(3, Math.min(100, opts.pct || 0));
      bar.style.width = pct + '%';
    } else if (opts.mode === 'done') {
      toastEl.classList.add('ap-toast-done');
      toastHideTimer = setTimeout(function () { toast({ mode: 'hide' }); }, 3500);
    }
    // Force a reflow so the slide-in transition runs even on the first show.
    void toastEl.offsetWidth;
    toastEl.classList.add('ap-toast-show');
  }

  /* ---------- measured state ---------- */

  function storedWikis() {
    return caches.keys().then(function (names) {
      var stored = {};
      var partial = {};
      var offline = names.filter(function (n) { return n.indexOf(OFFLINE_CACHE_PREFIX) === 0; });
      return Promise.all(offline.map(function (name) {
        return caches.open(name).then(function (cache) {
          var id = name.slice(OFFLINE_CACHE_PREFIX.length);
          // Without the completion marker it is an aborted download.
          return cache.match(COMPLETE_MARKER).then(function (marker) {
            if (marker) { stored[id] = true; } else { partial[id] = true; }
          });
        });
      })).then(function () { partialIds = partial; return stored; });
    });
  }

  function countCachedPages() {
    return caches.keys().then(function (names) {
      return Promise.all(names
        .filter(function (n) { return n.indexOf(PAGE_CACHE_PREFIX) === 0; })
        .map(function (n) {
          return caches.open(n).then(function (c) {
            return c.keys().then(function (k) { return k.length; });
          });
        })).then(function (counts) {
          return counts.reduce(function (a, b) { return a + b; }, 0);
        });
    });
  }

  function storage() {
    var est = navigator.storage && navigator.storage.estimate
      ? navigator.storage.estimate() : Promise.resolve({});
    var per = navigator.storage && navigator.storage.persisted
      ? navigator.storage.persisted() : Promise.resolve(false);
    return Promise.all([est, per]).then(function (r) {
      return { estimate: r[0] || {}, persisted: r[1] };
    });
  }

  /* ---------- rendering ---------- */

  function renderStorage() {
    return Promise.all([storage(), countCachedPages()]).then(function (r) {
      var est = r[0].estimate, persisted = r[0].persisted, pages = r[1];
      var used = est.usage || 0;

      // Saved wikis and pages cached while reading are different things.
      var savedWikis = Object.keys(storedIds).filter(function (id) {
        return id !== 'common';
      }).length;
      var parts = [];
      parts.push(savedWikis
        ? savedWikis + ' wiki' + (savedWikis === 1 ? '' : 's') + ' saved'
        : 'no wikis saved');
      var partial = Object.keys(partialIds).length;
      if (partial) {
        parts.push(partial + ' incomplete download' + (partial === 1 ? '' : 's'));
      }
      if (pages) {
        parts.push(pages + ' page' + (pages === 1 ? '' : 's') + ' cached while reading');
      }
      // Chrome's quota figure lags a deletion by seconds; say so and look again.
      var nothingHeld = !savedWikis && !pages && !storedIds.common;
      if (nothingHeld && used > 5 * 1048576) {
        parts.push('freeing space');
        if (!reclaimTimer) {
          reclaimTimer = setTimeout(function () {
            reclaimTimer = null;
            renderStorage();
          }, 3000);
        }
      } else {
        parts.push(fmt(used) + ' used');
      }
      // Free space is not shown: the quota is a fuzzed figure, not disk space.
      parts.push('storage ' + (persisted ? 'permanent' : 'temporary'));
      el('storage-status').textContent = parts.join(' · ');

      var evicted = evictedIds();
      var evictNote = evicted.length
        ? '<div class="apo-note apo-note-warn">&#9888; ' +
          (evicted.length === 1 ? 'A wiki you saved is' : evicted.length + ' wikis you saved are') +
          ' no longer here. Your browser reclaimed the space. Save ' +
          (evicted.length === 1 ? 'it' : 'them') + ' again, and install the ' +
          'wiki as an app to make that less likely.</div>'
        : '';
      el('storage-warning').innerHTML = evictNote + (persisted
        ? ''
        : '<div class="apo-note apo-note-warn">&#9888; Storage is ' +
          '<strong>temporary</strong>. Your browser can delete these saved pages ' +
          'without warning if this device runs low on space. Installing the wiki ' +
          'as an app makes that less likely. ' +
          '<a href="#install-as-an-app" data-ap-install>Install it now</a>, ' +
          'or read what that means below.</div>');

      // Nothing stored, nothing to remove.
      var clear = el('clear-btn');
      if (clear) {
        var anything = pages > 0 || Object.keys(storedIds).length > 0;
        clear.disabled = !anything || checkBusy || !!activeDownload ||
          !!activeExport || exportBusy;
        clear.title = !anything ? 'Nothing is stored on this device'
          : clear.disabled ? 'Wait for the update to finish'
          : 'Removes saved wikis and pages cached while reading';
        if (!anything && clearArmed) { disarmClear(clear); }
      }

      updateExportState();
      updateSaveState();
      updateTotal();
    });
  }

  function wikiById(id) {
    for (var i = 0; i < WIKIS.length; i++) {
      if (WIKIS[i].id === id) { return WIKIS[i]; }
    }
    return null;
  }

  function selectionBytes() {
    var selectedTotal = 0, toDownload = 0;

    selected().forEach(function (c) {
      var b = parseInt(c.dataset.mb, 10) * 1048576;
      selectedTotal += b;
      if (!storedIds[c.value]) { toDownload += b; }
    });

    var commonBytes = (COMMON.mb || 0) * 1048576;
    selectedTotal += commonBytes;
    if (!storedIds.common) { toDownload += commonBytes; }

    return { total: selectedTotal, toDownload: toDownload };
  }

  var storedIds = {};
  // Markerless caches: downloads that did not finish.
  var partialIds = {};

  // Mirrored to localStorage so an evicted wiki can be reported.
  var SAVED_IDS_KEY = 'ap-saved-ids';

  function savedRecord() {
    try {
      var raw = window.localStorage.getItem(SAVED_IDS_KEY);
      var arr = raw ? JSON.parse(raw) : [];
      return Array.isArray(arr) ? arr : [];
    } catch (err) { return []; }
  }

  function rememberSaved(id) {
    try {
      var rec = savedRecord();
      if (rec.indexOf(id) === -1) {
        rec.push(id);
        window.localStorage.setItem(SAVED_IDS_KEY, JSON.stringify(rec));
      }
    } catch (err) { /* private browsing: eviction notices are a nicety */ }
  }

  function forgetSaved(id) {
    try {
      var rec = savedRecord().filter(function (x) { return x !== id; });
      window.localStorage.setItem(SAVED_IDS_KEY, JSON.stringify(rec));
    } catch (err) { /* ignore */ }
  }

  // Recorded as saved but no longer in Cache Storage: the browser reclaimed it.
  function evictedIds() {
    return savedRecord().filter(function (id) {
      return id !== 'common' && !storedIds[id];
    });
  }
  // Set while waiting for the browser to finish reclaiming deleted space.
  var reclaimTimer = null;


  // Historical parameter versions travel inside the archive, as deltas
  // against the newest stable; the manifest lists them so the row can say so.
  function paramsOf(w) {
    return (w && w.param_versions) || [];
  }

  // The deltas need the decoder to read. Checked once a saved wiki carries
  // any, and if this browser cannot run it the reader is offered the plain
  // pages instead: the site serves every version as ordinary HTML.
  var PLAIN_VERSION_MB = 0.3;   // gzipped over the wire, about what each costs
  var decoderCheck = null;     // the self-test, run once per page load
  var decoderVerdict = null;   // null: not asked; true or false once known
  var plainFetch = null;       // the fallback download in flight

  function wikisWithVersions() {
    return WIKIS.filter(function (w) { return storedIds[w.id] && paramsOf(w).length; });
  }

  // The carried versions still stored as deltas, per saved wiki; a plain
  // page fetched over one no longer counts.
  function deltasHeld() {
    return Promise.all(wikisWithVersions().map(function (w) {
      return caches.open(OFFLINE_CACHE_PREFIX + w.id).then(function (cache) {
        return Promise.all(paramsOf(w).map(function (v) {
          var key;
          try { key = ApUnpack.cachePathFor(w.id, w.id + '/' + v.file); } catch (err) { return null; }
          return cache.match(key).then(function (hit) {
            return hit && hit.headers && hit.headers.get('x-ap-encoding') === 'zstd-delta'
              ? { w: w, v: v, key: key } : null;
          });
        }));
      });
    })).then(function (lists) {
      return [].concat.apply([], lists).filter(Boolean);
    });
  }

  function renderDeltaWarning(held) {
    var line = el('delta-warning');
    if (!line) { return; }
    if (decoderVerdict !== false || !held.length) {
      line.hidden = true;
      line.innerHTML = '';
      return;
    }
    line.hidden = false;
    line.innerHTML = '&#9888; This browser cannot rebuild the compressed parameter ' +
      'versions your saved wikis carry, so those pages will not open offline. ' +
      'The current parameter list is unaffected. ' +
      '<button type="button" id="plain-params-btn" class="apo-btn apo-btn-ghost">' +
      'Download them as plain pages (about ' +
      Math.round(held.length * PLAIN_VERSION_MB) + ' MB)</button>';
  }

  function checkDecoder() {
    if (!wikisWithVersions().length) {
      renderDeltaWarning([]);
      return Promise.resolve(decoderVerdict);
    }
    if (!decoderCheck) {
      decoderCheck = ApUnpack.decoderWorks().then(function (ok) {
        decoderVerdict = ok;
        return ok;
      });
    }
    return decoderCheck.then(function (ok) {
      if (ok) { renderDeltaWarning([]); return ok; }
      return deltasHeld().then(function (held) { renderDeltaWarning(held); return ok; });
    });
  }

  /** Fetch the plain page for every carried version of every saved wiki and
   * store it over the delta; a page with no delta header stores as any page. */
  function fetchPlainVersions() {
    if (plainFetch) { return plainFetch; }
    if (activeDownload || activeExport) {
      return Promise.reject(new Error('A download is already running; try again when it finishes.'));
    }
    var progress = el('cache-progress');
    var button = el('plain-params-btn');
    if (button) { button.disabled = true; }
    progress.hidden = false;
    var done = 0, failed = [], wanted = [];
    plainFetch = deltasHeld().then(function (held) {
      wanted = held;
      return held;
    }).then(function (held) { return held.reduce(function (chain, item) {
      return chain.then(function () {
        var w = item.w, v = item.v, url = item.key;
        progress.textContent = 'Fetching ' + w.name + ' parameters ' + v.label +
                               ' (' + (done + 1) + ' of ' + wanted.length + ')\u2026';
        // Tagged as an update so the worker goes to the network and never
        // answers with the delta it cannot read, or its offline page.
        var tagged = url + '?ap-update=' + encodeURIComponent(CURRENT_BUILD || '1');
        return fetch(tagged, { cache: 'no-cache' }).then(function (r) {
          if (!r.ok) { throw new Error('HTTP ' + r.status); }
          // A captive portal answers 200 with its own page; a header that
          // does not say HTML is refused rather than stored as the version.
          var ct = r.headers && r.headers.get && r.headers.get('Content-Type');
          if (ct && !/html/i.test(ct)) { throw new Error('served as ' + ct); }
          return r.arrayBuffer();
        }).then(function (buf) {
          return caches.open(OFFLINE_CACHE_PREFIX + w.id).then(function (cache) {
            return ApUnpack.storeEntry(cache, url, v.file, new Uint8Array(buf));
          });
        }).then(function () { done++; }, function (err) {
          console.warn('[offline] plain parameter page skipped', url, err && err.message);
          failed.push(v.file);
        });
      });
    }, Promise.resolve()); }).then(function () {
      plainFetch = null;
      notifyWorkerCachesChanged();
      if (failed.length) {
        progress.textContent = done + ' of ' + wanted.length + ' parameter pages saved as plain ' +
          'pages; ' + failed.length + ' could not be fetched (' + failed[0] + '). Try again later.';
      } else {
        progress.textContent = 'All ' + done + ' parameter versions are saved as plain pages.';
      }
      return renderWikis().then(renderStorage);
    });
    return plainFetch;
  }

  // Common is images plus a folded wiki; a count there says nothing useful.
  function countCell(w) {
    return w.images ? '' : (w.pages || '');
  }

  function renderWikis() {
    return storedWikis().then(function (stored) {
      storedIds = stored;
      var rows = [COMMON].concat(WIKIS).map(function (w) {
        var isStored = !!stored[w.id];
        var box = w.required
          ? '<input type="checkbox" checked disabled title="Required">'
          : '<input type="checkbox" class="wiki-check" value="' + w.id +
            '" data-mb="' + w.mb + '"' + (isStored ? ' checked' : '') + '>';
        var badge = isStored
          ? '<span class="apo-badge apo-badge-stored">Saved</span>'
          : partialIds[w.id]
            ? '<span class="apo-badge apo-badge-partial">Incomplete, save again</span>'
            : '<span class="apo-badge apo-badge-none">Not saved</span>';
        return '<tr data-wiki="' + w.id + '">' +
                 '<td class="apo-name"><label class="apo-pick">' + box +
                   '<span>' + w.name + '</span></label>' +
                   (paramsOf(w).length
                     ? ' <span class="apo-param-count">' +
                         paramsOf(w).length + ' parameter versions</span>'
                     : '') +
                 '</td>' +
                 '<td class="apo-num">' + w.mb + ' MB</td>' +
                 '<td class="apo-num apo-pages">' + countCell(w) + '</td>' +
                 // From state: this tbody is rebuilt when a download finishes.
                 '<td class="apo-num apo-col-progress"><div class="apo-progress"' +
                   (isStored ? '' : ' hidden') + '>' +
                   '<div class="apo-progress-bar" style="width:' +
                     (isStored ? '100%' : '0') + '"></div>' +
                   '<span>' + (isStored ? '100%' : '') + '</span></div></td>' +
                 '<td class="apo-num">' + badge + '</td>' +
               '</tr>';
      });
      el('wiki-rows').innerHTML = rows.join('');
      checkDecoder().catch(function () { /* the warning speaks for itself */ });

      var clear = el('clear-btn');
      if (clear) {
        var anySaved = Object.keys(stored).length > 0;
        if (anySaved && !activeDownload && !activeExport && !exportBusy && !checkBusy) {
          clear.disabled = false; clear.title = '';
        }
      }

      syncSelectAll();
      updateExportState();
      updateSaveState();
      updateTotal();
    });
  }

  /** Show progress on one wiki's own row, the way the build tool does. */
  function rowProgress(wikiId, percent, label) {
    var row = document.querySelector('tr[data-wiki="' + wikiId + '"]');
    if (!row) { return; }
    var wrap = row.querySelector('.apo-progress');
    var bar = row.querySelector('.apo-progress-bar');
    var text = row.querySelector('.apo-progress span');
    var badge = row.querySelector('.apo-badge');
    if (!wrap) { return; }

    if (percent === null) {
      wrap.hidden = true;
      return;
    }
    if (percent >= 100) {
      wrap.hidden = false;
      bar.style.width = '100%';
      text.textContent = label || '100%';
      if (badge) {
        badge.className = 'apo-badge apo-badge-stored';
        badge.textContent = 'Saved';
      }
      return;
    }
    wrap.hidden = false;
    bar.style.width = Math.max(0, Math.min(100, percent)) + '%';
    text.textContent = label || Math.round(percent) + '%';
    if (badge) {
      badge.className = 'apo-badge apo-badge-busy';
      badge.textContent = 'Saving';
    }
  }

  function selected() {
    return Array.prototype.slice.call(document.querySelectorAll('.wiki-check:checked'));
  }

  function selectable() {
    return Array.prototype.slice.call(document.querySelectorAll('.wiki-check'));
  }

  /** Ticked when every wiki is, indeterminate when some are. */
  function syncSelectAll() {
    var box = el('select-all');
    if (!box) { return; }
    var all = selectable().length;
    var on = selected().length;
    box.checked = all > 0 && on === all;
    box.indeterminate = on > 0 && on < all;
    box.disabled = all === 0;
  }

  function toggleAll(on) {
    selectable().forEach(function (c) { c.checked = on; });
    syncSelectAll();
    updateTotal();
    updateExportState();
    updateSaveState();
  }

  function updateTotal() {
    var b = selectionBytes();
    var total = el('selection-total');
    if (!total) { return; }

    // Short: the column is barely wider than the checkbox.
    if (!b.total) {
      total.textContent = 'Nothing selected';
    } else if (!b.toDownload) {
      total.innerHTML = '<strong>' + fmt(b.total) + '</strong> selected, ' +
                        'all already saved';
    } else {
      total.innerHTML = '<strong>' + fmt(b.toDownload) + '</strong> to download' +
                        (b.toDownload === b.total ? '' :
                          ' &middot; ' + fmt(b.total - b.toDownload) + ' of ' +
                          fmt(b.total) + ' already saved');
    }
    warnIfOverQuota(b);
  }

  // Warn while choosing: WebKit discards the origin on a write over quota.
  var QUOTA_MARGIN = 1.15;

  function warnIfOverQuota(b) {
    var line = el('quota-warning');
    if (!line) { return; }
    if (!b.toDownload) { line.hidden = true; return; }
    storage().then(function (r) {
      var est = (r && r.estimate) || {};
      if (est.quota === undefined) { line.hidden = true; return; }
      var free = (est.quota || 0) - (est.usage || 0);
      if (free >= b.toDownload * QUOTA_MARGIN) { line.hidden = true; return; }
      line.hidden = false;
      line.textContent = '\u26A0 This needs about ' + fmt(b.toDownload) +
        ' and your browser is offering ' + fmt(free) + '. Downloading anyway ' +
        'can lose everything already saved for this site, without an error. ' +
        'Deselect a wiki, or free up space first.';
    }).catch(function () { line.hidden = true; });
  }

  /* ---------- actions ---------- */

  // Destructive, so it asks in place and disarms itself if left alone.
  var CONFIRM_MS = 6000;
  // A double click must never delete anything.
  var CONFIRM_DEAD_MS = 700;

  var clearArmed = null;
  var clearArmedAt = 0;

  function disarmClear(btn) {
    if (clearArmed) { clearTimeout(clearArmed); }
    clearArmed = null;
    btn.textContent = 'Remove all';
    btn.classList.remove('apo-btn-armed');
    var bar = btn.querySelector('.apo-arm');
    if (bar) { bar.remove(); }
  }

  function confirmClear() {
    var btn = el('clear-btn');
    if (!btn) { return; }

    if (clearArmed) {
      if (Date.now() - clearArmedAt < CONFIRM_DEAD_MS) { return; }
      disarmClear(btn);
      return clearAll();
    }

    return Promise.resolve().then(function () {
      var used = heldBytes();
      btn.textContent = used ? 'Delete ' + fmt(used) + '? Press again'
                             : 'Press again to confirm';
      btn.classList.add('apo-btn-armed');

      var bar = document.createElement('span');
      bar.className = 'apo-arm';
      btn.appendChild(bar);
      // Force a layout so the transition starts from full width.
      void bar.offsetWidth;
      bar.style.transitionDuration = CONFIRM_MS + 'ms';
      bar.classList.add('apo-arm-run');

      clearArmedAt = Date.now();
      clearArmed = setTimeout(function () { disarmClear(btn); }, CONFIRM_MS);
    });
  }

  // Manifest sizes of what is saved; storage.estimate() lags a deletion.
  function heldBytes() {
    return [COMMON].concat(WIKIS).reduce(function (n, w) {
      return storedIds[w.id] ? n + (w.mb || 0) * 1048576 : n;
    }, 0);
  }

  function clearAll() {
    return caches.keys().then(function (names) {
      return Promise.all(names
        .filter(function (n) { return n.indexOf('ardupilot-') === 0; })
        .map(function (n) { return caches.delete(n); }));
    }).then(function () {
      // Deliberately removed, so it is not an eviction.
      try { window.localStorage.removeItem(SAVED_IDS_KEY); } catch (err) { /* ignore */ }
      notifyWorkerCachesChanged();
      return renderWikis().then(renderStorage);
    });
  }

  // Retire a folded wiki's old cache once common holds its pages.
  function dropFoldedCaches(entry) {
    if (!entry || entry.id !== 'common') { return Promise.resolve(); }
    return Promise.all(FOLDED_INTO_COMMON.map(function (id) {
      return caches.delete(OFFLINE_CACHE_PREFIX + id).then(function (gone) {
        if (gone) {
          delete storedIds[id];
          forgetSaved(id);
        }
      });
    })).then(function () { notifyWorkerCachesChanged(); })
      .catch(function () { /* tidying, not a step of the download */ });
  }

  // A download that dies partway leaves a cache with holes.
  function checkRoom(neededBytes) {
    return storage().then(function (r) {
      var est = r.estimate;
      if (est.quota === undefined) { return true; }
      var available = (est.quota || 0) - (est.usage || 0);
      if (available < neededBytes * HEADROOM) {
        throw new Error('Not enough room: this needs about ' + fmt(neededBytes) +
          ' plus working space, and only ' + fmt(available) + ' is available. ' +
          'Deselect a wiki or free up space.');
      }
      return true;
    });
  }

  /* ---------- download and unpack ---------- */

  var mimeFor = ApUnpack.mimeFor;

  function updateCfg() {
    return {
      base: ARTIFACT_BASE, build: CURRENT_BUILD, wikis: WIKIS,
      here: location.pathname.split('/')[1],
      offlinePrefix: OFFLINE_CACHE_PREFIX, completeMarker: COMPLETE_MARKER,
      mimeFor: mimeFor,
      getSignal: function () { return activeDownload ? activeDownload.signal : undefined; }
    };
  }

  var activeDownload = null;
  // Set while the exporter is packing pages; its button then cancels.
  var activeExport = null;
  // An update found while packing waits here for the export to finish.
  var updateDeferred = false;
  // Set while an update check is rewriting caches; exports wait for it.
  var updateWriting = false;

  function cancelDownload() {
    if (activeDownload) { activeDownload.abort(); }
  }

  /** Download what is selected and not held; `refreshIds` re-fetches stored wikis. */
  function saveSelectedReal(refreshIds, fromButton) {
    // The same button becomes Cancel; nothing else may cancel a download.
    if (activeDownload) {
      if (fromButton) { return cancelDownload(); }
      return Promise.reject(new Error('A download is already running; ' +
                                      'try again when it finishes.'));
    }
    // The plain-page fallback writes the same caches.
    if (plainFetch) {
      return Promise.reject(new Error('The parameter pages are still downloading; ' +
                                      'try again when they finish.'));
    }
    // Packing reads these caches; nothing may rewrite them underneath it.
    if (activeExport && !fromButton) {
      return Promise.reject(new Error('An export is being written; ' +
                                      'try again when it finishes.'));
    }
    // A running check owns the caches too; its own refresh call carries
    // refreshIds, and an export's pre-save runs under exportBusy, which the
    // check defers to. A reader's plain Save is what waits.
    if ((checkBusy || updateWriting) && !refreshIds && !exportBusy) {
      return Promise.reject(new Error('An update check is running; ' +
                                      'try again when it finishes.'));
    }

    var refresh = refreshIds || [];
    var chosen = selected().map(function (c) { return c.value; });
    // Chosen wikis first, common last, so a wiki is readable in seconds.
    var queue = WIKIS.filter(function (w) {
      return chosen.indexOf(w.id) !== -1;
    }).concat([COMMON]).filter(function (w) {
      return !storedIds[w.id] || refresh.indexOf(w.id) !== -1;
    });

    if (!queue.length) {
      el('cache-progress').hidden = false;
      el('cache-progress').textContent =
        'Everything selected is already saved. Use Check for updates to refresh it.';
      return Promise.resolve();
    }

    var totalBytes = queue.reduce(function (a, w) {
      return a + (w.mb || 0) * 1048576;
    }, 0);

    var progress = el('cache-progress');
    var button = el('download-cache-btn');
    var received = 0;
    var failedWith = null;

    progress.hidden = false;
    activeDownload = new AbortController();
    // The download owns the panel: nothing else may write these caches.
    var heldButtons = ['clear-btn', 'check-btn', 'dl-single'].map(el);
    heldButtons.forEach(function (b) { if (b) { b.disabled = true; } });
    button.classList.add('busy');
    setLabel('Cancel');

    function setLabel(text) {
      var lbl = button.querySelector('.lbl');
      if (lbl) { lbl.textContent = text; } else { button.textContent = text; }
    }
    function report(text) { progress.textContent = text; }
    report('Checking space…');

    // Persistence is asked for before storing, so the data is protected
    // from the start, but never waited on: Firefox turns the request into
    // a prompt the reader may not notice, and the promise stays pending
    // until they do. The download goes ahead after a moment regardless.
    var persistFirst = Promise.resolve(false);
    try {
      if (navigator.storage && navigator.storage.persist) {
        var asked = navigator.storage.persist().catch(function () { return false; });
        persistFirst = Promise.race([
          asked,
          new Promise(function (resolve) { setTimeout(function () { resolve(false); }, 1500); })
        ]);
        // An answer that arrives after the save has finished still counts;
        // the storage line must not say temporary until the next tick.
        asked.then(function (granted) { if (granted) { renderStorage(); } });
      }
    } catch (err) { /* no storage manager; nothing to wait for */ }

    return persistFirst
      .then(function () { return checkRoom(totalBytes); })
      .then(function () {
        // Marked complete only at the end.
        return queue.reduce(function (chain, entry) {
          return chain.then(function () {
            var cacheName = OFFLINE_CACHE_PREFIX + entry.id;
            // Unpacked over the existing copy, which stays readable throughout;
            // entries the new archive no longer carries are pruned at the end.
            return caches.open(cacheName).then(function (cache) {
              // raw_bytes: the browser decompresses before we count.
              var entryBytes = entry.raw_bytes || (entry.mb || 0) * 1048576;
              var entryGot = 0;
              var unpacked;
              var wrote = false;   // any entry stored over the old copy
              return ApUnpack.fetchArchive(entry, cache, function (n) {
                received += n;
                entryGot += n;
                var pct = Math.min(99, Math.round(received / totalBytes * 100));
                rowProgress(entry.id,
                  entryBytes ? Math.min(99, (entryGot / entryBytes) * 100) : pct);
                report(entry.name + ' · ' + pct + '%');
              }, {
                base: ARTIFACT_BASE,
                build: CURRENT_BUILD,
                hash: function (body) { wrote = true; return ApUpdate.hashBytes(body); },
                signal: activeDownload ? activeDownload.signal : undefined
              }).then(function (names) {
                unpacked = names;
                // The bytes are in; the table check comes next, and on a slow
                // device it is long enough to need saying.
                report('Checking ' + entry.name + '\u2026');
                rowProgress(entry.id, 99, 'checking');
                // The file table both verifies this save and drives updates;
                // without it nothing vouches for what just arrived.
                return fetch(ApUpdate.tableUrl(entry, ARTIFACT_BASE, CURRENT_BUILD), { cache: 'no-cache' })
                  .then(function (r) {
                    if (!r.ok) { throw new Error('HTTP ' + r.status); }
                    return r.json();
                  })
                  .then(function (table) {
                    // A 200 whose body is null or not an object vouches for
                    // nothing either.
                    if (!table || typeof table !== 'object' ||
                        Array.isArray(table)) {
                      throw new Error('not a file table');
                    }
                    return table;
                  })
                  .catch(function (err) {
                    var e = new Error('could not verify ' + entry.name + ' (' +
                                    ((err && err.message) || 'no file table') +
                                    '); nothing was marked saved. Try again.');
                    e.apVerify = true;
                    throw e;
                  });
              }).then(function (table) {
                // The fetch above throws rather than yield a missing table.
                {
                  // A build published mid-save leaves the table naming pages the
                  // archive lacked; marked complete, that mismatch would never
                  // heal. Nor would corrupt bytes under a right name: updates
                  // compare this table to the published one and see no change.
                  var have = {};
                  (unpacked || []).forEach(function (e) { have[e.name] = e.hash || true; });
                  var missing = [];
                  var damaged = [];
                  Object.keys(table).forEach(function (n) {
                    if (!have[n]) { missing.push(n); }
                    else if (!table[n] ||
                             (have[n] !== true && have[n] !== table[n])) { damaged.push(n); }
                  });
                  // A mid-save build rotation is the likelier story, so it
                  // is told first; an unlisted extra has no hash behind it
                  // and is refused rather than kept.
                  if (missing.length) {
                    var em = new Error('the server published a new build while saving ' +
                                    entry.name + '; try again in a moment');
                    em.apVerify = true;
                    throw em;
                  }
                  var unlisted = (unpacked || []).filter(function (e) {
                    return !Object.prototype.hasOwnProperty.call(table, e.name);
                  });
                  if (unlisted.length) {
                    var eu = new Error('the archive for ' + entry.name +
                                    ' held a file the server does not list (' +
                                    unlisted[0].name + '); nothing was marked saved. Try again.');
                    eu.apVerify = true;
                    throw eu;
                  }
                  if (damaged.length) {
                    var ed = new Error('part of ' + entry.name + ' arrived damaged (' +
                                    damaged[0] + '); nothing was marked saved. Try again.');
                    ed.apVerify = true;
                    throw ed;
                  }
                }
                // Prune what the new archive no longer carries.
                var keep = {};
                (unpacked || []).forEach(function (e) {
                  keep[ApUnpack.cachePathFor(entry.id, e.name)] = true;
                });
                return cache.keys().then(function (requests) {
                  return Promise.all(requests.map(function (request) {
                    var key = String(request.url || request)
                      .replace(/^https?:\/\/[^/]+/, '').split('?')[0].split('#')[0];
                    if (keep[key] || key === COMPLETE_MARKER ||
                        key === ApUpdate.TABLE_KEY) { return null; }
                    return cache.delete(request);
                  }));
                }).then(function () {
                  return ApUpdate.storeTable(cache, table);
                });
              }).catch(function (err) {
                if (!err || !(err.apVerify || wrote)) { throw err; }
                // The unpack already rewrote entries; whether verification
                // failed or the unpack itself died part-way, an older marker
                // would serve that mix as complete, and so would the worker's
                // memo of it until told.
                return cache.delete(COMPLETE_MARKER).then(function () {
                  return cache.delete(ApUpdate.TABLE_KEY);
                }).then(function () { notifyWorkerCachesChanged(); throw err; },
                        function () { notifyWorkerCachesChanged(); throw err; });
              }).then(function () {
                // The marker records the build an update check compares against.
                return cache.put(COMPLETE_MARKER,
                  new Response(JSON.stringify({
                    build: CURRENT_BUILD, saved: Date.now(), id: entry.id
                  }), { headers: { 'Content-Type': 'application/json' } }));
              }).then(function () {
                rowProgress(entry.id, 100, 'done');
                // One source of truth, updated the moment it is true.
                storedIds[entry.id] = true;
                rememberSaved(entry.id);
                notifyWorkerCachesChanged();
                return dropFoldedCaches(entry).then(renderStorage);
              });
            });
          });
        }, Promise.resolve());
      })
      .then(function () { report('Saved'); })
      .catch(function (err) {
        if (err && err.name === 'AbortError') {
          report('Cancelled. Anything already saved is kept.');
        } else if (err && err.name === 'QuotaExceededError') {
          report('Ran out of space; this download is incomplete. ' +
                 'Free some space and save again.');
        } else {
          report((err && err.message) || 'Download failed');
        }
        // Remembered past the cleanup: the caller must see the failure,
        // or an update wrapper would announce success over it. A cancel is
        // the reader's own decision, not a failure to escalate.
        if (!err || err.name !== 'AbortError') {
          failedWith = err || new Error('Download failed');
        }
      })
      .then(function () {
        activeDownload = null;
        button.classList.remove('busy');
        setLabel('Save selected');
        heldButtons.forEach(function (b) {
          if (!b) { return; }
          // During an export's span only its own button comes back, and a
          // still-running check keeps its own button until its tail.
          if (b.id === 'check-btn' && checkBusy) { return; }
          if (b.id === 'dl-single' || !exportBusy) { b.disabled = false; }
        });
        resumeDeferredUpdate();
        // Only unfinished bars are cleared.
        queue.forEach(function (w) {
          if (!storedIds[w.id]) { rowProgress(w.id, null); }
        });
        return renderWikis().then(renderStorage).then(function () {
          if (failedWith) { throw failedWith; }
        });
      });
  }

  /** Compare the server's build with each stored copy and update what is behind. */
  var checkBusy = false;

  function checkForUpdates(quiet) {
    var out = el('check-result');
    if (checkBusy) { return Promise.resolve(); }
    if (activeDownload) {
      if (out) {
        out.hidden = false;
        out.textContent = 'A download is running; check again when it finishes.';
      }
      return Promise.resolve();
    }
    checkBusy = true;
    var checkBtn = el('check-btn');
    if (checkBtn) { checkBtn.disabled = true; }
    var clearBtn = el('clear-btn');
    if (clearBtn) { clearBtn.disabled = true; }

    // An automatic run says nothing until it has news.
    function announce(text) { out.hidden = false; out.textContent = text; }
    var report = quiet ? function () {} : announce;

    report('Checking…');
    if (!quiet) { toast({ title: 'Checking for updates', msg: '', mode: 'sweep' }); }

    return fetch('/offline/offline-manifest.json', { cache: 'no-cache' })
      .then(function (r) {
        if (!r.ok) { throw new Error('could not reach the server'); }
        return r.json();
      })
      .then(function (manifest) {
        CURRENT_BUILD = manifest.generated || CURRENT_BUILD;
        if (manifest.common && manifest.wikis) {
          COMMON = manifest.common;
          WIKIS = manifest.wikis;
        }
        return caches.keys().then(function (names) {
          var offline = names.filter(function (n) {
            return n.indexOf(OFFLINE_CACHE_PREFIX) === 0;
          });
          return Promise.all(offline.map(function (name) {
            // The cache name gives the id even when the marker is unreadable.
            var id = name.slice(OFFLINE_CACHE_PREFIX.length);
            // A folded wiki's own cache is a leftover: common carries its
            // pages now, so it is retired rather than endlessly "updated".
            if (FOLDED_INTO_COMMON.indexOf(id) !== -1) {
              // The worker never serves this cache; retired either way.
              dropFoldedCaches(COMMON);
              return null;
            }
            return caches.open(name).then(function (c) {
              return c.match(COMPLETE_MARKER).then(function (m) {
                if (!m) { return null; }
                return m.json().then(function (info) {
                  return (info.build && info.build !== CURRENT_BUILD)
                    ? (info.id || id) : null;
                }).catch(function () { return null; });
              });
            });
          }));
        });
      })
      .then(function (results) {
        var stale = results.filter(Boolean);
        if (!stale.length) {
          report('Up to date.');
          if (!quiet) {
            toast({ title: 'Up to date', msg: 'Your saved wikis are current.',
                    mode: 'done' });
          }
          return;
        }
        if (exportBusy || activeExport) {
          // The exporter owns these caches for its whole span, the pre-save
          // included: even the differential update rewrites them.
          report('Updates found; they will be fetched after the export finishes.');
          toast({ mode: 'hide' });
          updateDeferred = true;
          return undefined;
        }
        // Real news, announced even on an automatic run.
        announce('Updating ' + stale.length + ' item' +
                 (stale.length === 1 ? '' : 's') + '…');
        toast({ title: 'Updating saved wikis',
                msg: 'Checking what changed…', mode: 'sweep' });

        // Differential first; no stored table means the archive.
        var byId = {};
        [COMMON].concat(WIKIS).forEach(function (w) { byId[w.id] = w; });

        var moved = 0, full = [];
        // The selection is the reader's. Both update paths end in a
        // re-render that ticks every stored wiki, and the archive path
        // borrows the boxes on top; it is recorded here and handed back
        // whichever way the update went.
        var chosenBefore = selectable().filter(function (c) { return c.checked; })
          .map(function (c) { return c.value; });
        var restoreSelection = function () {
          selectable().forEach(function (c) {
            c.checked = chosenBefore.indexOf(c.value) !== -1;
          });
          syncSelectAll();
          updateTotal();
          updateExportState();
          updateSaveState();
        };
        updateWriting = true;
        return stale.reduce(function (chain, id) {
          return chain.then(function () {
            var entry = byId[id];
            if (!entry) { full.push(id); return; }
            return ApUpdate.updateStored(entry, updateCfg(), function (done, total) {
              announce('Updating ' + entry.name + ' · ' +
                       done + ' of ' + total + ' files…');
              toast({ title: 'Updating ' + entry.name,
                      msg: done + ' of ' + total + ' files',
                      mode: 'progress', pct: total ? (done / total) * 100 : 0 });
            }).then(function (result) {
              if (!result) { full.push(id); return; }
              moved += result.changed + result.removed;
            }, function () {
              full.push(id);
            });
          });
        }, Promise.resolve()).then(function () {
          updateWriting = false;
          if (!full.length) {
            if (moved) {
              announce('Updated ' + moved + ' file' + (moved === 1 ? '' : 's') + '.');
              toast({ title: 'Update complete',
                      msg: 'Updated ' + moved + ' file' + (moved === 1 ? '' : 's') + '.',
                      mode: 'done' });
              notifyWorkerCachesChanged();
            } else {
              report('Already up to date.');
              if (!quiet) {
                toast({ title: 'Up to date', msg: 'Nothing had changed.',
                        mode: 'done' });
              }
            }
            return renderWikis().then(restoreSelection);
          }
          // These cannot be updated in place, so they are downloaded again,
          // on a timer too: an out-of-date saved copy is worse than a download.
          var nameOf = function (id) { return (byId[id] && byId[id].name) || id; };
          announce('Downloading again: ' + full.map(nameOf).join(', ') + '\u2026');
          toast({ title: 'Updating saved wikis',
                  msg: 'Downloading again: ' + full.map(nameOf).join(', ') + '.',
                  mode: 'sweep' });

          // Re-select what needs a full fetch and reuse the one download path.
          selectable().forEach(function (c) {
            c.checked = full.indexOf(c.value) !== -1;
          });
          syncSelectAll();
          updateTotal();
          updateExportState();
          updateSaveState();
          // The one caller allowed to re-fetch what is already stored.
          return saveSelectedReal(full).then(function (value) {
            restoreSelection(); return value;
          }, function (err) {
            restoreSelection(); throw err;
          }).then(function () {
            var missing = full.filter(function (id) { return !storedIds[id]; });
            if (missing.length) {
              announce('Could not download again: ' + missing.map(nameOf).join(', ') + '.');
              toast({ title: 'Update incomplete',
                      msg: 'Could not download again: ' + missing.map(nameOf).join(', ') +
                           '. It will be tried again later.',
                      mode: 'done' });
              return;
            }
            announce('Downloaded again: ' + full.map(nameOf).join(', ') + '.');
            toast({ title: 'Update complete',
                    msg: 'Downloaded again: ' + full.map(nameOf).join(', ') + '.',
                    mode: 'done' });
            notifyWorkerCachesChanged();
          });
        });
      })
      .catch(function (err) {
        // A failed automatic check means offline, which is ordinary.
        report((err && err.message) || 'Check failed');
      })
      .then(function () {
        checkBusy = false;
        updateWriting = false;
        if (checkBtn && !activeDownload && !activeExport) { checkBtn.disabled = false; }
        return renderStorage();
      });
  }

  /** Save is offered only when it has something to fetch. */
  function updateSaveState() {
    var button = el('download-cache-btn');
    if (!button || activeDownload || activeExport || exportBusy) { return; }
    var b = selectionBytes();
    // Common alone is images with no pages to view them in.
    var anyWiki = selected().length > 0;
    button.disabled = !anyWiki || !b.toDownload;
    button.title = !anyWiki
      ? 'Select a wiki first'
      : (b.toDownload
          ? 'Downloads ' + fmt(b.toDownload)
          : 'Everything selected is already saved. Check for updates refreshes it.');
  }

  function updateExportState() {
    // While a download or an export owns the button, only they may write it.
    if (activeDownload || activeExport || exportBusy) { return; }
    var chosen = selected().length;
    var b = el('dl-single');
    if (!b) { return; }
    b.disabled = !chosen;
    b.title = chosen ? '' : 'Select at least one wiki first';
  }

  // The selection, limited to what is saved.
  function exportSelection() {
    var chosen = selected().map(function (c) { return c.value; });
    var have = chosen.filter(function (id) { return storedIds[id]; });
    var missing = chosen.filter(function (id) { return !storedIds[id]; });
    return { ids: have, missing: missing, chosen: chosen };
  }

  /** e.g. blimp-copter-rover-2026-08-08.html */
  function exportName(ids, extension) {
    var stamp = (CURRENT_BUILD || new Date().toISOString()).slice(0, 10);
    var base;
    if (!ids.length) {
      base = 'ardupilot';
    } else if (ids.length >= WIKIS.length) {
      base = 'ardupilot-all';
    } else {
      base = ids.slice().sort().join('-');
    }
    return base + '-' + stamp + extension;
  }

  // Anything selected but unsaved is downloaded first: one press, not two.
  var exportBusy = false;

  // Runs the update that stood aside for an export, when nothing owns
  // the caches any more. The flag survives a blocked attempt, so whichever
  // owner finishes last finds it and tries again.
  function resumeDeferredUpdate() {
    if (!updateDeferred) { return; }
    setTimeout(function () {
      // Re-checked here: an owner can claim the panel in the timer window,
      // narrower than the harness can produce; the flag then waits for it.
      if (!updateDeferred || activeExport || exportBusy ||
          activeDownload || checkBusy) { return; }
      updateDeferred = false;
      checkForUpdates(true);
    }, 0);
  }

  function buildExport(buttonId) {
    var link = el(buttonId);
    if (!link || !global.ArduPilotExport || !selected().length) { return; }
    // One export at a time, over its whole span including the pre-save.
    if (exportBusy) { return; }
    if (updateWriting) {
      if (!link.dataset.label) { link.dataset.label = link.textContent; }
      link.textContent = 'An update is being written; try again in a moment.';
      setTimeout(function () {
        // A later export owns the label by now.
        if (!exportBusy) { link.textContent = link.dataset.label || 'Save as .html'; }
      }, 5000);
      return;
    }
    exportBusy = true;

    var original = link.dataset.label || link.textContent;
    link.dataset.label = original;
    link.disabled = true;
    var release = null;

    var done = function (text, keep) {
      exportBusy = false;
      link.textContent = text;
      // A running download owns the button; its cleanup re-enables it.
      if (!activeDownload) { link.disabled = false; }
      if (!keep) {
        setTimeout(function () {
          // A later export owns the label by now.
          if (!exportBusy) { link.textContent = original; }
        }, 8000);
      }
    };

    var sel = exportSelection();
    // The shared images are required and carry no checkbox, so a broken
    // common cache is repaired here or the export could never succeed.
    var toSave = sel.missing.slice();
    if (!storedIds.common) { toSave.push('shared images'); }
    link.textContent = toSave.length
      ? 'Saving ' + toSave.join(', ') + '…'
      : 'Preparing…';

    if (toSave.length && window.ApOffline) {
      // A pre-save fills the same caches Save does; unserved caches would
      // leave the panel listing wikis nothing answers for.
      window.ApOffline.enable();
      renderOfflineMode();
    }
    var first = toSave.length ? saveSelectedReal() : Promise.resolve();

    // The repair re-renders the rows with every saved wiki ticked; the
    // reader's own choices are what the export honours and puts back.
    var chosenBefore = sel.chosen.slice();
    return first.then(function () {
      var boxes = document.querySelectorAll('.wiki-check');
      for (var bi = 0; bi < boxes.length; bi++) {
        boxes[bi].checked = chosenBefore.indexOf(boxes[bi].value) !== -1;
      }
      syncSelectAll();
      updateTotal();
      var ready = { ids: chosenBefore.filter(function (id) { return storedIds[id]; }) };
      if (!ready.ids.length) {
        throw new Error('Nothing was saved - check your connection');
      }
      var name = exportName(ready.ids, '.html');
      // Packing reads the caches for a while: it owns the panel just as a
      // download does, and its own button becomes the way out.
      activeExport = new AbortController();
      var held = ['clear-btn', 'check-btn', 'download-cache-btn'].map(el);
      held.forEach(function (b) { if (b) { b.disabled = true; } });
      release = function () {
        activeExport = null;
        exportBusy = false;
        resumeDeferredUpdate();
        // Hand back only what nothing else still owns.
        if (!activeDownload) {
          held.forEach(function (b) { if (b) { b.disabled = false; } });
          if (checkBusy) {
            var cb = el('check-btn');
            if (cb) { cb.disabled = true; }
          }
        }
        updateSaveState();
        updateExportState();
        renderStorage();
      };
      link.disabled = false;
      return global.ArduPilotExport.exportHtml(ready.ids, name,
        function (n, total) {
          link.textContent = 'Writing ' + n + ' / ' + total +
                             ' pages… (click to cancel)';
        }, undefined, activeExport.signal).then(function (r) {
          release();
          // A version that could not be rebuilt is not in the file; saying
          // only the page count would report a whole copy either way.
          done('Saved ' + name + ' (' + r.pages + ' pages)' +
               (r.leftOut ? ', ' + r.leftOut + ' parameter version' +
                            (r.leftOut === 1 ? '' : 's') + ' left out' : ''));
        });
    }).catch(function (err) {
      if (release) { release(); }
      else { activeExport = null; resumeDeferredUpdate(); }
      done(err && err.name === 'AbortError'
        ? 'Export cancelled'
        : (err && err.message) || 'Export failed');
    });
  }

  // Thirty minutes, jittered; the wiki rebuilds a few times a day.
  var AUTOUPDATE_MS = 30 * 60 * 1000;

  var autoTimer = null;
  var autoBusy = false;

  function autoUpdateOn() {
    var box = el('autoupdate');
    return !!(box && box.checked);
  }

  function autoUpdateTick() {
    if (!autoUpdateOn()) { return; }
    // A running check, download or export owns the panel and the network.
    if (autoBusy || activeDownload || activeExport) { return; }
    // Nothing saved means nothing to update and no reason to touch the network.
    if (!Object.keys(storedIds).length) { return; }
    // Not skipped while hidden: a background tab is the copy most likely behind.

    autoBusy = true;
    return checkForUpdates(true).then(function () { autoBusy = false; },
                                      function () { autoBusy = false; });
  }

  // Jittered +/-50% so a new build is not a stampede.
  function scheduleNextTick() {
    if (autoTimer) { window.clearTimeout(autoTimer); }
    var jittered = AUTOUPDATE_MS * (0.5 + Math.random());
    autoTimer = window.setTimeout(function () {
      var done = autoUpdateTick();
      if (done && typeof done.then === 'function') {
        done.then(scheduleNextTick, scheduleNextTick);
      } else {
        scheduleNextTick();
      }
    }, jittered);
  }

  function startAutoUpdate() {
    scheduleNextTick();
  }

  function exportHtmlFile() { return buildExport('dl-single'); }

  /* ---------- wiring ---------- */

  document.addEventListener('change', function (e) {
    if (e.target.classList.contains('wiki-check')) {
      syncSelectAll();
      updateTotal();
      updateExportState();
      updateSaveState();
    }
    if (e.target.id === 'select-all') { toggleAll(e.target.checked); }
    if (e.target.id === 'offline-mode' && global.ApOffline) {
      if (e.target.checked) {
        Promise.resolve(global.ApOffline.enable()).then(renderOfflineMode, renderOfflineMode);
      } else {
        offerTurnOff();
      }
    }
    if (e.target.id === 'autoupdate') {
      try {
        window.localStorage.setItem(AUTOUPDATE_KEY, e.target.checked ? '1' : '0');
      } catch (err) { /* private browsing */ }
      // Turning it on should mean something now rather than at the next tick.
      if (e.target.checked) { autoUpdateTick(); }
    }
  });

  document.addEventListener('click', function (e) {
    // closest(): the armed Remove all contains its own countdown bar.
    var hit = e.target && e.target.closest &&
              e.target.closest('#clear-btn, #download-cache-btn, #check-btn, #dl-single, ' +
                               '#offline-off-confirm, #offline-off-keep, #plain-params-btn');
    if (!hit) { return; }
    if (hit.id === 'plain-params-btn') {
      fetchPlainVersions().catch(function (err) {
        el('cache-progress').hidden = false;
        el('cache-progress').textContent = (err && err.message) || String(err);
      });
    }
    if (hit.id === 'clear-btn') { confirmClear(); }
    if (hit.id === 'offline-off-confirm') { turnOff(); }
    if (hit.id === 'offline-off-keep') { hideTurnOff(); renderOfflineMode(); }
    if (hit.id === 'download-cache-btn') {
      // Saving opts in to offline mode (pwa.js).
      if (window.ApOffline) { window.ApOffline.enable(); renderOfflineMode(); }
      Promise.resolve(saveSelectedReal(undefined, true)).catch(function (err) {
        var out = el('cache-progress');
        if (out) {
          out.hidden = false;
          out.textContent = (err && err.message) || 'Could not start the download.';
        }
      });
    }
    if (hit.id === 'check-btn') { checkForUpdates(); }
    if (hit.id === 'dl-single') {
      e.preventDefault();
      if (activeExport) { activeExport.abort(); } else { exportHtmlFile(); }
    }
  });

  // Off removes everything held, so say how much first, unless it is nothing.
  function busyWithWhat() {
    if (activeDownload) { return 'a download'; }
    if (exportBusy || activeExport) { return 'an export'; }
    if (checkBusy || updateWriting) { return 'an update check'; }
    return null;
  }

  function offerTurnOff() {
    var busy = busyWithWhat();
    if (busy) {
      renderOfflineMode();
      var out = el('check-result');
      if (out) {
        out.hidden = false;
        out.textContent = 'Still running: ' + busy +
          '. Wait for it or cancel it, then turn off.';
      }
      return Promise.resolve();
    }
    var note = el('offline-off-warning');
    return storage().then(function (s) {
      // estimate() can lag what the manifest says is held; never claim 0 MB.
      var used = Math.max(s.estimate.usage || 0, heldBytes());
      var holding = Object.keys(storedIds).length > 0 || used > 5 * 1048576;
      if (!note || !holding) { return turnOff(); }
      renderOfflineMode();
      el('offline-off-size').textContent = fmt(used);
      note.hidden = false;
    });
  }

  function hideTurnOff() {
    var note = el('offline-off-warning');
    if (note) { note.hidden = true; }
  }

  // pwa.js does the removing; this redraws what is left, which is nothing.
  function turnOff() {
    var busy = busyWithWhat();
    if (busy) {
      hideTurnOff();
      renderOfflineMode();
      var out = el('check-result');
      if (out) {
        out.hidden = false;
        out.textContent = 'Still running: ' + busy +
          '. Wait for it or cancel it, then turn off.';
      }
      return Promise.resolve();
    }
    hideTurnOff();
    return Promise.resolve(global.ApOffline.disable()).then(function () {
      notifyWorkerCachesChanged();
      return renderWikis().then(renderStorage);
    }).then(renderOfflineMode, renderOfflineMode);
  }

  // The switch reflects pwa.js's flag; pwa.js owns the registration itself.
  function renderOfflineMode() {
    var box = el('offline-mode'), state = el('offline-mode-state');
    if (!box || !global.ApOffline) { return; }
    var on = global.ApOffline.enabled();
    box.checked = on;
    if (state) { state.textContent = on ? 'on' : 'off'; }
  }

  function init() {
    renderOfflineMode();
    try {
      var pref = window.localStorage.getItem(AUTOUPDATE_KEY);
      if (pref === '0') { el('autoupdate').checked = false; }
    } catch (err) { /* private browsing */ }

    startAutoUpdate();
    // Returning to a long-hidden tab is when its copy is most likely behind.
    document.addEventListener('visibilitychange', function () {
      if (!document.hidden) { autoUpdateTick(); }
    });

    var standalone = window.matchMedia('(display-mode: standalone)').matches ||
                     window.navigator.standalone === true;
    el('install-state').textContent = standalone
      ? 'Already running as an installed app.'
      : '';

    if (!('caches' in window)) {
      el('storage-status').textContent =
        'This browser does not support offline storage.';
      return;
    }

    // renderStorage reads storedIds, so it follows renderWikis.
    renderWikis().then(renderStorage);

    // Sizes and counts come from the manifest; the constants are a fallback.
    fetch('/offline/offline-manifest.json', { cache: 'no-cache' })
      .then(function (r) { return r.ok ? r.json() : null; })
      .catch(function () { return null; })
      .then(function (manifest) {
        if (manifest && manifest.common && manifest.wikis) {
          COMMON = manifest.common;
          WIKIS = manifest.wikis;
          if (manifest.artifact_base) {
            ARTIFACT_BASE = manifest.artifact_base.replace(/\/$/, '');
          }
          CURRENT_BUILD = manifest.generated || null;
          if (manifest.generated) {
            el('build-date').textContent =
              'Build ' + manifest.generated.slice(0, 10);
          }
        }
        renderStorage();
        renderWikis();
      });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})(window);
