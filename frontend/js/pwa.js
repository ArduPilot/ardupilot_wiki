/*
 * Loaded on every page. Registers the service worker once the reader has opted
 * in (menu switch or saving a wiki), drives the install-as-app button, shows
 * the page-updated toast, and carries a few page-speed helpers: link
 * prefetching, anchor landing on huge pages, and live video embeds.
 */
(function () {
  'use strict';

  if (!('serviceWorker' in navigator)) {
    return;
  }

  var INSTALL_BUTTON_ID = 'ap-install-app';
  var INSTALLABLE_KEY = 'ap-installable';
  var deferredPrompt = null;
  var promptArrived = null;
  var signalPromptArrived = null;

  promptArrived = new Promise(function (resolve) {
    signalPromptArrived = resolve;
  });

  function remember(installable) {
    try {
      if (installable) {
        window.localStorage.setItem(INSTALLABLE_KEY, '1');
      } else {
        window.localStorage.removeItem(INSTALLABLE_KEY);
      }
    } catch (err) {
      /* private browsing; the button simply reverts to appearing late */
    }
  }

  function wasInstallable() {
    try {
      return window.localStorage.getItem(INSTALLABLE_KEY) === '1';
    } catch (err) {
      return false;
    }
  }

  function installButton() {
    return document.getElementById(INSTALL_BUTTON_ID);
  }

  function showInstallButton() {
    var button = installButton();
    if (button) {
      button.hidden = false;
    }
  }

  function hideInstallButton() {
    var button = installButton();
    if (button) {
      button.hidden = true;
    }
  }

  // beforeinstallprompt can fire before a deferred script attaches a listener.
  window.addEventListener('beforeinstallprompt', function (event) {
    event.preventDefault();
    deferredPrompt = event;
    remember(true);
    signalPromptArrived();
    showInstallButton();
  });

  window.addEventListener('appinstalled', function () {
    deferredPrompt = null;
    remember(false);
    hideInstallButton();
    installState('Installed.');
  });

  function installState(text) {
    var state = document.getElementById('install-state');
    if (state) { state.textContent = text; }
  }

  // Anything marked data-ap-install offers to install, not only the button.
  var INSTALL_SELECTOR = '#' + INSTALL_BUTTON_ID + ', [data-ap-install]';

  document.addEventListener('click', function (event) {
    var target = event.target.closest ? event.target.closest(INSTALL_SELECTOR) : null;
    if (!target) {
      return;
    }
    // A link would otherwise jump the page before the dialog is decided.
    if (target.tagName === 'A') {
      event.preventDefault();
    }

    // The button can be on screen before beforeinstallprompt fires; wait for it.
    var ready = deferredPrompt
      ? Promise.resolve()
      : Promise.race([
          promptArrived,
          new Promise(function (resolve) { setTimeout(resolve, 2000); }),
        ]);

    var isButton = target.tagName === 'BUTTON';
    if (isButton) { target.disabled = true; }
    ready.then(function () {
      if (isButton) { target.disabled = false; }
      if (!deferredPrompt) {
        // This browser will not prompt; point at its own menu instead.
        remember(false);
        installState('Your browser did not offer to install here. Look for ' +
                     '\u201cInstall\u201d or \u201cAdd to Home Screen\u201d in its menu.');
        // Fall back to where the link pointed.
        var href = !isButton && target.getAttribute('href');
        if (href && href.charAt(0) === '#') { window.location.hash = href; }
        return;
      }
      deferredPrompt.prompt();
      return deferredPrompt.userChoice.then(function (choice) {
        if (choice.outcome === 'accepted') {
          hideInstallButton();
        }
        // The event is single use whatever the reader chose.
        deferredPrompt = null;
      });
    });
  });

  function showUpdateToast() {
    if (document.getElementById('ap-update-toast')) {
      return;
    }
    var toast = document.createElement('div');
    toast.id = 'ap-update-toast';
    toast.setAttribute('role', 'status');
    toast.style.cssText = [
      'position:fixed', 'bottom:20px', 'left:50%', 'transform:translateX(-50%)',
      'background:#2980b9', 'color:#fff', 'padding:10px 16px', 'border-radius:4px',
      'font-family:sans-serif', 'font-size:14px', 'z-index:9999',
      'box-shadow:0 2px 8px rgba(0,0,0,0.3)',
    ].join(';');
    toast.textContent = 'This page has been updated. ';

    var reload = document.createElement('button');
    reload.type = 'button';
    reload.textContent = 'Reload';
    reload.style.cssText = 'margin-left:8px;background:#fff;color:#2980b9;border:0;' +
                           'padding:4px 10px;border-radius:3px;cursor:pointer';
    reload.addEventListener('click', function () {
      window.location.reload();
    });

    toast.appendChild(reload);
    document.body.appendChild(toast);
  }

  navigator.serviceWorker.addEventListener('message', function (event) {
    if (!event.data) {
      return;
    }
    if (event.data.type === 'PAGE_UPDATED' &&
        event.data.url === window.location.href.split('#')[0]) {
      showUpdateToast();
    }
    // The kill switch has run; stay off so it is not registered again.
    if (event.data.type === 'OFFLINE_KILLED') {
      disableOffline();
    }
  });


  function isStandalone() {
    return window.matchMedia('(display-mode: standalone)').matches ||
           window.navigator.standalone === true;
  }

  // Installable last visit: show the button at once.
  document.addEventListener('DOMContentLoaded', function () {
    if (!isStandalone() && wasInstallable()) {
      showInstallButton();
    }
  });

  // Nothing registers until the reader turns offline mode on or saves a wiki.
  var OFFLINE_KEY = 'ap-offline-enabled';
  // Shared with common_offline_page.js.
  var SAVED_IDS_KEY = 'ap-saved-ids';

  function offlineEnabled() {
    try {
      return window.localStorage.getItem(OFFLINE_KEY) === '1';
    } catch (err) {
      return false;
    }
  }

  function enableOffline() {
    try {
      window.localStorage.setItem(OFFLINE_KEY, '1');
    } catch (err) {
      /* private browsing; the registration below still holds for this tab */
    }
    registerServiceWorker();
  }

  // The whole opt-out: flag, saved list, every cache, then the registration.
  // The switch on the offline page and the kill switch (sw-kill.js) both end here.
  function disableOffline() {
    try {
      window.localStorage.removeItem(OFFLINE_KEY);
      window.localStorage.removeItem(SAVED_IDS_KEY);
    } catch (err) {
      /* private browsing */
    }
    var wipe = window.caches ? window.caches.keys().then(function (names) {
      return Promise.all(names.filter(function (name) {
        return name.indexOf('ardupilot-') === 0;
      }).map(function (name) { return window.caches.delete(name); }));
    }) : Promise.resolve();
    return wipe.catch(function () { /* storage gone already */ }).then(function () {
      return navigator.serviceWorker.getRegistration();
    }).then(function (registration) {
      return registration ? registration.unregister() : false;
    }).catch(function () { return false; });
  }

  // The switch on the offline page, and its Save button, drive these.
  window.ApOffline = { enabled: offlineEnabled, enable: enableOffline, disable: disableOffline };

  function startWhenOptedIn() {
    if (offlineEnabled()) {
      registerServiceWorker();
      return;
    }
    // An existing registration without the flag is honoured.
    navigator.serviceWorker.getRegistration().then(function (registration) {
      if (registration) { enableOffline(); }
    }).catch(function () { /* nothing registered, nothing to honour */ });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', startWhenOptedIn);
  } else {
    startWhenOptedIn();
  }

  function registerServiceWorker() {
    navigator.serviceWorker.register('/sw.js', { scope: '/' }).then(function (registration) {
      // Cannot succeed offline, and that is not an error.
      registration.update().catch(function (err) {
        console.debug('[pwa] worker update check skipped', err && err.name);
      });
    }).catch(function (err) {
      console.warn('[pwa] service worker registration failed', err);
    });

    // Already running as an installed app, so there is nothing to install.
    if (isStandalone()) {
      hideInstallButton();
    }
  }
})();

// Prefetch the page the pointer is heading for.
(function () {
  'use strict';

  // Speculative traffic is part of offline mode; a reader who has not opted in causes none.
  if (!(window.ApOffline && window.ApOffline.enabled())) {
    return;
  }

  var conn = navigator.connection || {};
  if (conn.saveData || /(^|-)2g$/.test(conn.effectiveType || '')) {
    return;
  }

  // Conservative: a pointer heuristic can turn one reader into a load generator.
  var MAX_BYTES = 2 * 1024 * 1024;   // the generated reference pages are 5.8MB
  // Past this the reader is scanning, not being led; guess nothing.
  var MAX_TRACKED = 250;
  var MAX_PER_PAGE = 5;              // total guesses allowed per page view
  var MIN_GAP_MS = 400;              // never two in quick succession
  var NEAR_PX = 72;                  // close enough to act on by itself
  var LOOKAHEAD_MS = 250;            // how far ahead the path is projected
  var SLOW_PX_MS = 0.25;             // slower than this counts as arriving

  var asked = new Set();
  var spent = 0;
  var lastAt = 0;
  var busy = false;
  var inFlight = null;
  var samples = [];                  // {x, y, t}, newest last
  var rects = null;
  var pending = null;

  function fetchable(a) {
    if (!a || !a.href || a.target === '_blank') { return null; }
    var u;
    try { u = new URL(a.href); } catch (e) { return null; }
    if (u.origin !== location.origin) { return null; }
    if (!/\.html?$|\/$/.test(u.pathname)) { return null; }
    u.hash = '';
    if (u.pathname === location.pathname || asked.has(u.href)) { return null; }
    return u.href;
  }

  // Recomputed on scroll and resize, never per pointer move.
  var tooMany = false;

  function measure() {
    rects = [];
    var all = document.querySelectorAll('a[href]');
    if (all.length > MAX_TRACKED) { tooMany = true; return; }
    var h = window.innerHeight, w = window.innerWidth;
    [].forEach.call(all, function (a) {
      var href = fetchable(a);
      if (!href) { return; }
      var r = a.getBoundingClientRect();
      if (r.bottom < 0 || r.top > h || r.right < 0 || r.left > w) { return; }
      rects.push({ href: href, r: r });
    });
  }

  function distanceTo(r, x, y) {
    var dx = Math.max(r.left - x, 0, x - r.right);
    var dy = Math.max(r.top - y, 0, y - r.bottom);
    return Math.sqrt(dx * dx + dy * dy);
  }

  function prefetch(href) {
    if (busy || asked.has(href)) { return; }

    // Already held costs nothing and spends no budget.
    if (window.caches && caches.match) {
      asked.add(href);
      // Exact match: ignoreSearch walks every cache, 300 ms with a dozen wikis.
      caches.match(href).then(function (hit) {
        if (hit) { return; }                 // free, and already done
        asked.delete(href);
        spend(href);
      }).catch(function () { asked.delete(href); spend(href); });
      return;
    }
    spend(href);
  }

  function spend(href) {
    var now = performance.now();
    if (busy || asked.has(href)) { return; }
    if (spent >= MAX_PER_PAGE) { return; }
    if (now - lastAt < MIN_GAP_MS) { return; }

    asked.add(href);
    spent++;
    lastAt = now;
    busy = true;

    // One request: the size is in the headers, so abandon anything large mid-flight.
    var ctl = new AbortController();
    inFlight = ctl;
    fetch(href, { credentials: 'same-origin', priority: 'low', signal: ctl.signal })
      .then(function (res) {
        var size = Number(res.headers.get('content-length') || 0);
        if (size > MAX_BYTES) {
          ctl.abort();
          return null;
        }
        return res.arrayBuffer();   // let the worker store it, then discard
      })
      .catch(function () { /* a speculative miss costs nothing */ })
      .then(function () { busy = false; inFlight = null; });
  }

  window.addEventListener('pagehide', function () {
    if (inFlight) { inFlight.abort(); }
  });

  function consider() {
    pending = null;
    if (tooMany) { return; }
    if (!rects) { measure(); }
    if (tooMany || !rects.length || samples.length < 3) { return; }

    var n = samples.length;
    var a = samples[n - 3], b = samples[n - 2], c = samples[n - 1];
    var dt1 = Math.max(b.t - a.t, 1), dt2 = Math.max(c.t - b.t, 1);

    // First derivative: where it is going, and how fast.
    var vx = (c.x - b.x) / dt2, vy = (c.y - b.y) / dt2;
    var speed = Math.sqrt(vx * vx + vy * vy);

    // Second derivative: whether it is winding down or still winding up.
    var prevSpeed = Math.sqrt(Math.pow((b.x - a.x) / dt1, 2) +
                              Math.pow((b.y - a.y) / dt1, 2));
    var slowing = speed < prevSpeed;

    // Where it will be shortly, if it carries on as it is.
    var px = c.x + vx * LOOKAHEAD_MS, py = c.y + vy * LOOKAHEAD_MS;

    var best = null, bestScore = 0;
    rects.forEach(function (item) {
      var now = distanceTo(item.r, c.x, c.y);
      var soon = distanceTo(item.r, px, py);

      var score = 0;
      if (now < NEAR_PX) { score += 1; }
      if (soon < now) { score += 1; }                       // heading for it
      if (soon < 12) { score += 1; }                        // path lands on it
      if (slowing && soon < NEAR_PX) { score += 2; }        // arriving at it
      if (speed < SLOW_PX_MS && now < NEAR_PX) { score += 1; }

      if (score > bestScore) { bestScore = score; best = item.href; }
    });

    // Proximity alone never reaches 3.
    if (best && bestScore >= 3) { prefetch(best); }
  }

  document.addEventListener('mousemove', function (e) {
    samples.push({ x: e.clientX, y: e.clientY, t: performance.now() });
    if (samples.length > 4) { samples.shift(); }
    if (!pending) { pending = requestAnimationFrame(consider); }
  }, { passive: true });

  // Touch has no approach to read, so the touch is the intent.
  document.addEventListener('touchstart', function (e) {
    var a = e.target && e.target.closest ? e.target.closest('a[href]') : null;
    var href = fetchable(a);
    if (href) { prefetch(href); }
  }, { passive: true });

  // 65 ms keeps a pointer crossing the menu from prefetching everything.
  var hoverTimer = null;
  document.addEventListener('mouseover', function (e) {
    var a = e.target && e.target.closest ? e.target.closest('a[href]') : null;
    var href = fetchable(a);
    if (!href) { return; }
    if (hoverTimer) { clearTimeout(hoverTimer); }
    hoverTimer = setTimeout(function () { prefetch(href); }, 65);
  }, { passive: true });
  document.addEventListener('mouseout', function () {
    if (hoverTimer) { clearTimeout(hoverTimer); hoverTimer = null; }
  }, { passive: true });

  // Next and previous are the likeliest clicks; take them without waiting.
  function prefetchNeighbours() {
    var picked = [];
    [].forEach.call(
      document.querySelectorAll('.rst-footer-buttons a[href], a[rel="next"], a[rel="prev"]'),
      function (a) {
        var href = fetchable(a);
        if (href && picked.indexOf(href) === -1) { picked.push(href); }
      });
    // One now, the other once that has finished, so they never race.
    if (picked[0]) { prefetch(picked[0]); }
    if (picked[1]) { setTimeout(function () { prefetch(picked[1]); }, MIN_GAP_MS + 50); }
  }

  if (document.readyState === 'complete') { setTimeout(prefetchNeighbours, 800); }
  else { window.addEventListener('load', function () { setTimeout(prefetchNeighbours, 800); }); }

  var remeasure = null;
  function invalidate() {
    clearTimeout(remeasure);
    remeasure = setTimeout(function () { rects = null; }, 150);
  }
  window.addEventListener('scroll', invalidate, { passive: true });
  window.addEventListener('resize', invalidate, { passive: true });
})();


// A section under content-visibility has no layout yet; reveal it before scrolling.
(function () {
  'use strict';

  function reveal() {
    if (!location.hash || location.hash.length < 2) { return; }
    var id;
    try {
      id = decodeURIComponent(location.hash.slice(1));
    } catch (err) {
      return; // a fragment like #% is nobody's anchor
    }
    var el = document.getElementById(id) ||
             document.getElementsByName(id)[0];
    if (!el) { return; }

    var revealed = false;
    for (var node = el; node && node !== document.body; node = node.parentElement) {
      // Only touch what is actually being skipped.
      if (getComputedStyle(node).contentVisibility === 'auto') {
        node.style.contentVisibility = 'visible';
        node.style.containIntrinsicSize = 'none';
        revealed = true;
      }
    }

    // Scroll only if something was revealed; otherwise the browser already did.
    if (revealed) { el.scrollIntoView(); }
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', reveal);
  } else {
    reveal();
  }
  window.addEventListener('hashchange', reveal);
})();

// Replace a saved wiki's video stills with the real player when online.
(function () {
  'use strict';

  var YT = /[?&]v=([\w-]{6,})/;
  var VIMEO = /^https?:\/\/vimeo\.com\/(\d+)/;

  function embedFor(href) {
    var m = YT.exec(href || '');
    if (m) {
      return { src: 'https://www.youtube-nocookie.com/embed/' + m[1] + '?rel=0',
               title: 'YouTube video' };
    }
    m = VIMEO.exec(href || '');
    if (m) {
      return { src: 'https://player.vimeo.com/video/' + m[1],
               title: 'Vimeo video' };
    }
    return null;
  }

  // The OS can report online with no route to the video host; a probe of
  // that host itself decides, so a dead iframe never replaces a good card.
  // Cached per origin: YouTube answering says nothing about Vimeo.
  var reachableUntil = {};
  function hostReachable(src) {
    var origin;
    try { origin = new URL(src).origin; } catch (err) { return Promise.resolve(false); }
    if (Date.now() < (reachableUntil[origin] || 0)) { return Promise.resolve(true); }
    return new Promise(function (resolve) {
      var timedOut = false;
      var timer = setTimeout(function () { timedOut = true; resolve(false); }, 2500);
      fetch(origin + '/favicon.ico', { mode: 'no-cors', cache: 'no-store' })
        .then(function () {
          clearTimeout(timer);
          // An answer that limped in after the timeout proves nothing fresh.
          if (timedOut) { return; }
          reachableUntil[origin] = Date.now() + 30000;
          resolve(true);
        }, function () { clearTimeout(timer); resolve(false); });
    });
  }

  function upgrade(a) {
    if (a.dataset.apLive) { return; }
    var embed = embedFor(a.getAttribute('href'));
    if (!embed) { return; }
    a.dataset.apLive = '1';
    if (navigator.onLine === false) { a.dataset.apLive = ''; return; }
    hostReachable(embed.src).then(function (ok) {
      // The connection can drop while the probe is in flight.
      if (!ok || navigator.onLine === false) { a.dataset.apLive = ''; return; }
      mountEmbed(a, embed);
    });
  }

  function mountEmbed(a, embed) {

    var img = a.querySelector('img');
    var still = img ? img.getAttribute('src') : null;

    var box = document.createElement('div');
    box.className = 'ap-video ap-video-live';
    box.style.cssText = 'position:relative;max-width:640px;margin:1em 0;' +
      'border-radius:4px;overflow:hidden;background:#2f2f2f' +
      (still ? ' url("' + still + '") center/cover no-repeat' : '');

    var ratio = document.createElement('span');
    ratio.style.cssText = 'display:block;padding-bottom:56.25%';

    var frame = document.createElement('iframe');
    frame.src = embed.src;
    frame.title = embed.title;
    frame.loading = 'lazy';
    frame.allowFullscreen = true;
    frame.setAttribute('allow',
      'accelerometer; encrypted-media; gyroscope; picture-in-picture');
    // Loaded hidden over the still and faded in on load, so no white flash.
    frame.style.cssText =
      'position:absolute;top:0;left:0;width:100%;height:100%;border:0;' +
      'opacity:0;transition:opacity 0.5s ease';

    frame.addEventListener('load', function () {
      frame.style.opacity = '1';
    });

    // Put the card back if the embed fails to load.
    frame.addEventListener('error', function () {
      if (box.parentNode) { box.parentNode.replaceChild(a, box); }
      a.dataset.apLive = '';
    });

    box.appendChild(ratio);
    box.appendChild(frame);
    if (a.parentNode) { a.parentNode.replaceChild(box, a); }
  }

  // An embed is about 1 MB of YouTube code.
  function whenIdle(fn) {
    if (document.readyState !== 'complete') {
      window.addEventListener('load', function () { whenIdle(fn); }, { once: true });
      return;
    }
    if (typeof window.requestIdleCallback === 'function') {
      window.requestIdleCallback(fn, { timeout: 2000 });
    } else {
      setTimeout(fn, 300);
    }
  }

  function start() {
    // Offline, the still and its link are the right thing to show.
    if (navigator.onLine === false) { return; }

    var cards = [].slice.call(document.querySelectorAll('a.ap-video'));
    if (!cards.length) { return; }

    // Pointer: immediately. On screen: once the page is done.
    cards.forEach(function (c) {
      c.addEventListener('pointerenter', function () { upgrade(c); });
    });

    if (typeof IntersectionObserver !== 'function') {
      whenIdle(function () { cards.forEach(upgrade); });
      return;
    }
    whenIdle(function () {
      // About a viewport of lead, so the fade has happened before the reader arrives.
      var seen = new IntersectionObserver(function (entries) {
        entries.forEach(function (e) {
          if (e.isIntersecting) { seen.unobserve(e.target); upgrade(e.target); }
        });
      }, { rootMargin: '600px' });
      cards.forEach(function (c) { seen.observe(c); });
    });
  }

  // A reader who was offline and comes back should get the players too.
  window.addEventListener('online', start);

  // An external link followed offline dies on a browser error page and
  // takes the reader with it; the click waits with a note instead.
  document.addEventListener('click', function (event) {
    var a = event.target && event.target.closest
      ? event.target.closest('a[data-ap-external]') : null;
    if (!a || navigator.onLine !== false) { return; }
    event.preventDefault();
    var note = document.getElementById('ap-offline-note');
    if (!note) {
      note = document.createElement('div');
      note.id = 'ap-offline-note';
      note.setAttribute('role', 'status');
      note.style.cssText = 'position:fixed;left:50%;bottom:24px;' +
        'transform:translateX(-50%);background:#2f2f2f;color:#fff;' +
        'padding:10px 16px;border-radius:6px;font-size:.95em;z-index:9999;' +
        'box-shadow:0 2px 12px rgba(0,0,0,.4)';
      document.body.appendChild(note);
    }
    note.textContent = 'This link needs a connection; it will work once you are back online.';
    note.hidden = false;
    clearTimeout(note._apTimer);
    note._apTimer = setTimeout(function () { note.hidden = true; }, 4000);
  }, true);

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', start);
  } else {
    start();
  }
})();
