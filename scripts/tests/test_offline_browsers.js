/*
 * Does the wiki read offline in a real browser, in every engine? Serves the
 * built tree locally, then STOPS THE SERVER: Playwright's setOffline does not
 * reach service worker fetches outside Chromium. Covers the read-as-you-browse
 * path; downloading a 439 MB archive per engine is not attempted.
 *
 *   node scripts/tests/test_offline_browsers.js [--browsers chromium,webkit] [--headed --keep]
 */

'use strict';

const path = require('path');
const { start, bumpWorker, serveKill } = require('./serve_wiki_tree');

const REPO = path.resolve(__dirname, '..', '..');

/* ------------------------------------------------------------- arguments -- */

const argv = process.argv.slice(2);
function flag(name) { return argv.includes('--' + name); }
function opt(name, fallback) {
  const i = argv.indexOf('--' + name);
  return i !== -1 && argv[i + 1] ? argv[i + 1] : fallback;
}

const HEADED = flag('headed');
const KEEP = flag('keep');           // leave the browser open on failure
const WANTED = opt('browsers', 'chromium,firefox,webkit').split(',')
  .map((s) => s.trim()).filter(Boolean);

// Pages in every local build; one is never visited online.
const VISITED = '/dev/docs/building-setup-linux.html';
const ALSO_VISITED = '/dev/index.html';
const NEVER_VISITED = '/dev/docs/apmcopter-programming-libraries.html';
const SEARCH_PAGE = '/dev/search.html';
// Never served by the server; only ever answered from a compressed cache entry.
const COMPRESSED_PROBE = '/dev/docs/ap-compressed-probe.html';

/* ---------------------------------------------------------------- harness -- */

const results = [];
let failures = 0;

// Every phase is timed and printed, so the cost is attributable.
const timings = [];
async function phase(engine, label, fn) {
  const at = Date.now();
  try {
    return await fn();
  } finally {
    timings.push({ engine, label, ms: Date.now() - at });
  }
}

function check(engine, name, ok, detail) {
  results.push({ engine, name, ok: !!ok, detail: detail || '' });
  if (!ok) { failures++; }
  console.log('  ' + (ok ? 'PASS  ' : 'FAIL  ') + name +
              (detail ? '   ' + detail : ''));
}

/* ---------------------------------------------------------------- helpers -- */

/** Wait until a worker is controlling this page, not merely registered. */
async function waitForControl(page, timeout = 30000) {
  const deadline = Date.now() + timeout;
  while (Date.now() < deadline) {
    const state = await page.evaluate(async () => {
      if (!('serviceWorker' in navigator)) { return { supported: false }; }
      const reg = await navigator.serviceWorker.getRegistration();
      return {
        supported: true,
        controlled: !!navigator.serviceWorker.controller,
        active: !!(reg && reg.active),
        state: reg && reg.active ? reg.active.state : null,
      };
    });
    if (!state.supported) { return state; }
    if (state.controlled) { return state; }
    // Reload once active, for engines that apply claim only to the next navigation.
    if (state.active) {
      await page.reload({ waitUntil: 'domcontentloaded' });
      const now = await page.evaluate(
        () => !!navigator.serviceWorker.controller);
      if (now) { return { supported: true, controlled: true, active: true }; }
    }
    await page.waitForTimeout(400);
  }
  return { supported: true, controlled: false, timedOut: true };
}

/** Scroll to the bottom so lazy images load, then count the ones that decoded. */
async function imageCount(page) {
  await page.evaluate(async () => {
    window.scrollTo(0, document.body.scrollHeight);
    document.querySelectorAll('img[loading="lazy"]').forEach((i) => {
      i.loading = 'eager';
    });
  });
  await page.waitForTimeout(1500);
  return page.evaluate(() => {
    const all = Array.from(document.images);
    return {
      total: all.length,
      loaded: all.filter((i) => i.naturalWidth > 0).length,
      missing: all.filter((i) => i.naturalWidth === 0)
        .map((i) => i.currentSrc || i.src),
    };
  });
}

/** Median wall time for a navigation, over several goes. */
async function navigationMs(page, url, runs = 5) {
  const times = [];
  for (let i = 0; i < runs; i++) {
    const at = Date.now();
    await page.goto(url, { waitUntil: 'load' });
    times.push(Date.now() - at);
  }
  times.sort((a, b) => a - b);
  return { median: times[Math.floor(times.length / 2)],
           worst: times[times.length - 1], all: times };
}

/** True when the document looks like a rendered wiki page, not an error. */
async function looksLikeWikiPage(page) {
  return page.evaluate(() => {
    const h1 = document.querySelector('h1');
    return {
      title: document.title,
      h1: h1 ? h1.textContent.trim().slice(0, 80) : null,
      hasNav: !!document.querySelector('.wy-nav-side, nav'),
      styleSheets: document.styleSheets.length,
      // A value the theme sets, since a failed stylesheet still counts.
      themed: getComputedStyle(document.body).fontFamily || '',
      bodyText: (document.body.innerText || '').length,
    };
  });
}


// A deploy with a page open: is the page controlled again after the swap?
async function checkUpdateWindow(name, browser, base) {
  const context = await browser.newContext({ serviceWorkers: 'allow' });
  // A reader who has opted in; the worker is dormant otherwise.
  await context.addInitScript(() => {
    window.localStorage.setItem('ap-offline-enabled', '1');
  });
  const page = await context.newPage();
  try {
    await page.goto(base + VISITED, { waitUntil: 'load' });
    const control = await waitForControl(page);
    if (!control.controlled) {
      check(name, 'a tab open across a worker update is controlled again',
            false, 'never took control to begin with');
      return;
    }

    const seeded = await page.evaluate(async (wikis) => {
      let total = 0;
      for (const w of wikis) {
        const cache = await caches.open('ardupilot-offline-' + w);
        await cache.put('/__ap_complete__', new Response('1'));
        const puts = [];
        for (let i = 0; i < 800; i++) {
          puts.push(cache.put('/' + w + '/docs/seed-' + i + '.html',
                              new Response('<html><body>seed</body></html>',
                                { headers: { 'Content-Type': 'text/html' } })));
        }
        await Promise.all(puts);
        total += 801;
      }
      return total;
    }, ['copter', 'plane', 'rover', 'sub', 'blimp', 'dev', 'antennatracker',
        'planner', 'planner2', 'ardupilot', 'mavproxy']);

    // Armed before the update. The old controller stays non-null throughout,
    // so only the handover events prove the new worker took the tab.
    const handover = page.evaluate(() => new Promise((resolve) => {
      const started = Date.now();
      const result = { updatefound: false, controllerchange: false, ms: -1 };
      const timer = setTimeout(() => resolve(result), 15000);
      navigator.serviceWorker.getRegistration().then((reg) => {
        reg.addEventListener('updatefound', () => { result.updatefound = true; });
      });
      navigator.serviceWorker.addEventListener('controllerchange', () => {
        result.controllerchange = true;
        result.ms = Date.now() - started;
        clearTimeout(timer);
        resolve(result);
      });
    }));
    bumpWorker();
    await page.evaluate(async () => {
      const reg = await navigator.serviceWorker.getRegistration();
      await reg.update();
    });
    const reclaimed = await handover;
    check(name, 'a tab open across a worker update is controlled again',
          reclaimed.updatefound && reclaimed.controllerchange,
          JSON.stringify(reclaimed) + ', with ' + seeded + ' entries saved');
  } finally {
    await context.close().catch(() => {});
  }
}

/* Two ways out, one end state: the switch, and the kill switch served as /sw.js. */

const OFFLINE_PAGE = '/dev/docs/common-offline.html';
const CLEAN = JSON.stringify({ registered: false, flag: null, saved: null, caches: [] });

// An opted-in reader on the offline page, holding two saved wikis.
async function optedInPage(browser, base) {
  const context = await browser.newContext({ serviceWorkers: 'allow' });
  const page = await context.newPage();
  await page.goto(base + OFFLINE_PAGE, { waitUntil: 'load' });
  await page.evaluate(async () => {
    window.localStorage.setItem('ap-offline-enabled', '1');
    window.localStorage.setItem('ap-saved-ids', JSON.stringify(['common', 'copter']));
    // Markers as the download writes them, and enough bytes to register as held.
    const build = (await (await fetch('/offline/offline-manifest.json')).json()).generated;
    for (const w of ['common', 'copter']) {
      const cache = await caches.open('ardupilot-offline-' + w);
      await cache.put('/__ap_complete__',
        new Response(JSON.stringify({ build, saved: Date.now(), id: w })));
      await cache.put('/' + w + '/docs/seed.html',
        new Response('<html>' + 'x'.repeat(3 << 20) + '</html>',
          { headers: { 'Content-Type': 'text/html' } }));
    }
  });
  await page.reload({ waitUntil: 'load' });
  const control = await waitForControl(page);
  return { context, page, controlled: control.controlled };
}

function optOutState(page) {
  return page.evaluate(async () => {
    const reg = await navigator.serviceWorker.getRegistration();
    const names = (await caches.keys()).filter((n) => n.startsWith('ardupilot-'));
    return { registered: !!reg,
             flag: window.localStorage.getItem('ap-offline-enabled'),
             saved: window.localStorage.getItem('ap-saved-ids'),
             caches: names };
  });
}

async function waitForClean(page, timeout = 10000) {
  const started = Date.now();
  let state = await optOutState(page);
  while (JSON.stringify(state) !== CLEAN && Date.now() - started < timeout) {
    await page.waitForTimeout(150);
    state = await optOutState(page);
  }
  return state;
}

async function checkOptOut(name, browser, base) {
  let ctx = await optedInPage(browser, base);
  try {
    if (!ctx.controlled) {
      check(name, 'switch off leaves a never-opted-in browser', false, 'never took control');
    } else {
      const { page } = ctx;
      await page.click('.apo-switch');
      // The warning waits on storage.estimate(), which WebKit answers slowly.
      await page.waitForSelector('#offline-off-warning:not([hidden])', { timeout: 5000 })
        .catch(() => {});
      const warned = await page.evaluate(() => ({
        shown: !document.getElementById('offline-off-warning').hidden,
        stillOn: document.getElementById('offline-mode').checked,
        size: document.getElementById('offline-off-size').textContent,
      }));
      check(name, 'switch off warns first and stays on until confirmed',
            warned.shown && warned.stillOn && /\d/.test(warned.size), JSON.stringify(warned));
      await page.click('#offline-off-confirm');
      const state = await waitForClean(page);
      check(name, 'switch off leaves a never-opted-in browser',
            JSON.stringify(state) === CLEAN, JSON.stringify(state));
    }
  } finally {
    await ctx.context.close().catch(() => {});
  }

  ctx = await optedInPage(browser, base);
  try {
    if (!ctx.controlled) {
      check(name, 'the kill switch leaves the same never-opted-in browser', false,
            'never took control');
    } else {
      const { page } = ctx;
      let loads = 0;
      page.on('load', () => { loads += 1; });
      serveKill(true);
      await page.evaluate(async () => {
        const reg = await navigator.serviceWorker.getRegistration();
        await reg.update();
      });
      let state = await waitForClean(page);
      const spontaneous = loads;
      let when = 'in the open tab';
      if (JSON.stringify(state) !== CLEAN) {
        // WebKit activates the replacement only when the tab next navigates.
        await page.reload({ waitUntil: 'load' });
        state = await waitForClean(page);
        when = 'on the next visit';
      }
      check(name, 'the kill switch leaves the same never-opted-in browser',
            JSON.stringify(state) === CLEAN, JSON.stringify(state) + ', ' + when);
      await page.reload({ waitUntil: 'load' });
      await page.waitForTimeout(1500);
      const after = await optOutState(page);
      check(name, 'the next visit after a kill registers nothing, and nothing reloaded by itself',
            JSON.stringify(after) === CLEAN && spontaneous === 0,
            JSON.stringify(after) + ', ' + spontaneous + ' spontaneous load(s)');
    }
  } finally {
    serveKill(false);
    await ctx.context.close().catch(() => {});
  }
}

/* ------------------------------------------------------------- one engine -- */

async function runEngine(name, launcher, base) {
  console.log('\n' + name);
  const browser = await phase(name, 'launch browser',
    () => launcher.launch({ headless: !HEADED }));
  const context = await browser.newContext({ serviceWorkers: 'allow' });
  const page = await context.newPage();

  // Only our own errors count, filtered by origin rather than by wording.
  const consoleErrors = [];
  page.on('console', (m) => {
    if (m.type() !== 'error') { return; }
    const from = (m.location() && m.location().url) || '';
    if (from && !from.startsWith(base)) { return; }
    consoleErrors.push(m.text().slice(0, 300));
  });
  const pageErrors = [];
  page.on('pageerror', (e) => pageErrors.push(String(e.message).slice(0, 200)));

  let serverHandle = null;
  try {
    /* ---- online: register the worker and read a few pages -------------- */

    // Opt-in: a reader who has not asked must get no registration at all.
    const dormant = await phase(name, 'no worker without opt-in', async () => {
      await page.goto(base + VISITED, { waitUntil: 'load' });
      await page.waitForTimeout(1200);
      return page.evaluate(async () => {
        const reg = await navigator.serviceWorker.getRegistration();
        // pwa.js must have run, or a 404 on it would pass this check.
        return { registered: !!reg, pwaLoaded: typeof window.ApOffline === 'object' };
      });
    });
    check(name, 'a reader who has not opted in gets no service worker',
          !dormant.registered && dormant.pwaLoaded, JSON.stringify(dormant));
    const switchOff = await phase(name, 'switch reads off', async () => {
      await page.goto(base + '/dev/docs/common-offline.html', { waitUntil: 'load' });
      await page.waitForTimeout(800);
      return page.evaluate(() => ({
        present: !!document.getElementById('offline-mode'),
        checked: !!(document.getElementById('offline-mode') || {}).checked,
        label: (document.getElementById('offline-mode-state') || {}).textContent,
      }));
    });
    check(name, 'the offline page shows the switch off before opting in',
          switchOff.present && !switchOff.checked && switchOff.label === 'off',
          JSON.stringify(switchOff));

    const control = await phase(name, 'opt in, register + take control', async () => {
      await page.evaluate(() => {
        window.localStorage.setItem('ap-offline-enabled', '1');
      });
      await page.reload({ waitUntil: 'load' });
      return waitForControl(page);
    });
    check(name, 'service worker takes control',
          control.controlled,
          control.supported === false ? 'no serviceWorker in navigator'
                                      : JSON.stringify(control));
    if (!control.controlled) {
      // Nothing below can mean anything without a controller.
      throw new Error('no controlling service worker');
    }

    const scope = await page.evaluate(async () => {
      const reg = await navigator.serviceWorker.getRegistration();
      return reg ? reg.scope : null;
    });
    check(name, 'worker scope is the whole origin',
          !!scope && new URL(scope).pathname === '/', String(scope));

    // Pins the harness, not production: it must model the documented nginx
    // config (no-cache on /sw.js), or the kill switch could never reach anyone.
    const swHeaders = await page.evaluate(async () => {
      const r = await fetch('/sw.js', { method: 'HEAD' });
      return { cc: r.headers.get('cache-control'),
               allowed: r.headers.get('service-worker-allowed') };
    });
    check(name, 'the harness serves sw.js no-cache, as production must',
          /no-cache|no-store|max-age=0/.test(swHeaders.cc || ''),
          'Cache-Control: ' + swHeaders.cc);

    // Reload the first: its initial load predates the worker.
    const onlineImages = await phase(name, 'read pages online', async () => {
      await page.goto(base + VISITED, { waitUntil: 'load' });
      const imgs = await imageCount(page);
      await page.goto(base + ALSO_VISITED, { waitUntil: 'load' });
      return imgs;
    });
    // Only search.html fetches searchindex.js.
    await phase(name, 'load search index', async () => {
      await page.goto(base + SEARCH_PAGE, { waitUntil: 'load' });
      // Give the stale-while-revalidate writes a moment to land.
      await page.waitForTimeout(2500);
    });

    const cached = await page.evaluate(async (p) => {
      const hit = await caches.match(location.origin + p);
      return !!hit;
    }, VISITED);
    check(name, 'visited page is in Cache Storage', cached);

    const quota = await page.evaluate(async () => {
      const out = { estimate: null, persisted: null, canPersist: false };
      if (navigator.storage && navigator.storage.estimate) {
        const e = await navigator.storage.estimate();
        out.estimate = { quota: e.quota, usage: e.usage };
      }
      if (navigator.storage && navigator.storage.persisted) {
        out.persisted = await navigator.storage.persisted();
        out.canPersist = typeof navigator.storage.persist === 'function';
      }
      return out;
    });
    const quotaGB = quota.estimate
      ? (quota.estimate.quota / 1e9).toFixed(1) + ' GB' : 'unreported';
    check(name, 'storage quota fits the 697 MB archive set',
          !!(quota.estimate && quota.estimate.quota > 750e6),
          'quota ' + quotaGB + ', persist API ' +
          (quota.canPersist ? 'present' : 'MISSING'));

    // Online, a page already read should cost about what it costs offline.
    const onlineSpeed = await navigationMs(page, base + VISITED);
    check(name, 'an already-read page is not slowed down by being online',
          onlineSpeed.median < 1500,
          'median ' + onlineSpeed.median + ' ms, worst ' + onlineSpeed.worst +
          ' ms of [' + onlineSpeed.all.join(', ') + ']');

    // Stored gzipped, as a saved wiki stores pages.
    const compressedSeed = await page.evaluate(async (probe) => {
      const html = '<!doctype html><html><head><title>Compressed probe</title>' +
        '</head><body><h1 id="probe">INFLATED-FROM-CACHE</h1><p>' +
        'padding '.repeat(400) + '</p></body></html>';
      const packed = await new Response(
        new Blob([html]).stream().pipeThrough(new CompressionStream('gzip'))
      ).arrayBuffer();
      const cache = await caches.open('ardupilot-offline-dev');
      await cache.put('/__ap_complete__', new Response('1'));
      await cache.put(probe, new Response(packed, {
        headers: { 'Content-Type': 'text/html; charset=utf-8',
                   'x-ap-encoding': 'gzip' },
      }));
      return { raw: html.length, packed: packed.byteLength };
    }, COMPRESSED_PROBE);

    // The panel first draws before the manifest arrives.
    const picker = await (async () => {
      const p = await context.newPage();
      try {
        await p.goto(base + '/ardupilot/docs/common-offline.html', { waitUntil: 'load' });
        await p.waitForTimeout(3500);
        return await p.evaluate(() => ({
          switchOn: !!(document.getElementById('offline-mode') || {}).checked,
          vehicles: document.querySelectorAll('.apo-param-toggle').length,
          versions: document.querySelectorAll('.param-check').length,
          ticked: document.querySelectorAll('.param-check:checked').length,
        }));
      } finally { await p.close().catch(() => {}); }
    })();
    check(name, 'the offline page shows the switch on once opted in',
          picker.switchOn, JSON.stringify({ switchOn: picker.switchOn }));
    check(name, 'each vehicle offers its versions with the newest of each series ticked',
          picker.vehicles > 0 && picker.versions > 0 &&
          picker.ticked === picker.versions,
          picker.vehicles + ' vehicles, ' + picker.versions + ' versions, ' +
          picker.ticked + ' ticked');

    /* ---- go offline for real ------------------------------------------- */

    serverHandle = global.__server;
    await serverHandle.close();
    global.__serverClosed = true;

    /* ---- a page read while online, read again with no server ----------- */

    let navError = null;
    try {
      await page.goto(base + VISITED, { waitUntil: 'load', timeout: 20000 });
    } catch (err) {
      navError = String(err.message).split('\n')[0];
    }
    check(name, 'offline navigation to a visited page does not hit the network',
          !navError, navError || '');

    const shown = await looksLikeWikiPage(page);
    check(name, 'visited page renders its real content offline',
          !!shown.h1 && /build environment/i.test(shown.title || ''),
          'title=' + JSON.stringify((shown.title || '').slice(0, 60)));
    check(name, 'theme CSS is served from cache offline',
          shown.styleSheets > 0 && /roboto|lato|slab|sans/i.test(shown.themed),
          shown.styleSheets + ' stylesheets, font ' +
          JSON.stringify(shown.themed.slice(0, 40)));
    check(name, 'sidebar navigation survives offline', shown.hasNav);

    const offlineImages = await imageCount(page);
    check(name, 'every image that loaded online still loads offline',
          offlineImages.loaded >= onlineImages.loaded,
          offlineImages.loaded + '/' + offlineImages.total + ' offline vs ' +
          onlineImages.loaded + '/' + onlineImages.total + ' online' +
          (offlineImages.missing.length
            ? '; missing ' + offlineImages.missing.slice(0, 2).join(', ') : ''));

    // A loose budget that notices a per-asset round trip creeping back.
    const offlineSpeed = await navigationMs(page, base + VISITED);
    check(name, 'a stored page is served offline well inside the budget',
          offlineSpeed.median < 1200,
          'median ' + offlineSpeed.median + ' ms, worst ' + offlineSpeed.worst +
          ' ms of [' + offlineSpeed.all.join(', ') + ']');

    // Fetched once online above; must resolve with no network.
    const searchIndex = await page.evaluate(async () => {
      try {
        const r = await fetch('/dev/searchindex.js');
        const t = await r.text();
        return { ok: r.ok, bytes: t.length };
      } catch (err) {
        return { ok: false, error: String(err.message) };
      }
    });
    check(name, 'the search index is reachable offline',
          searchIndex.ok && searchIndex.bytes > 1000,
          searchIndex.error || (searchIndex.bytes + ' bytes'));

    /* ---- a page never visited: the fallback, not a browser error ------- */

    let fallbackError = null;
    try {
      await page.goto(base + NEVER_VISITED,
                      { waitUntil: 'load', timeout: 20000 });
    } catch (err) {
      fallbackError = String(err.message).split('\n')[0];
    }
    check(name, 'offline navigation to an unsaved page is answered by the worker',
          !fallbackError, fallbackError || '');

    if (!fallbackError) {
      const fb = await page.evaluate(() => ({
        text: (document.body.innerText || '').slice(0, 200),
        title: document.title,
      }));
      check(name, 'unsaved page shows the offline fallback, not a dead end',
            /offline/i.test(fb.text) || /offline/i.test(fb.title),
            JSON.stringify(fb.title));
    }


    /* ---- a compressed cache entry, read back offline -------------------- */

    let probeErr = null;
    try {
      await page.goto(base + COMPRESSED_PROBE, { waitUntil: 'load', timeout: 20000 });
    } catch (err) {
      probeErr = String(err.message).split('\n')[0];
    }
    const probeText = probeErr ? null : await page.evaluate(() => {
      const h = document.getElementById('probe');
      return h ? h.textContent : (document.body.innerText || '').slice(0, 40);
    });
    check(name, 'a gzipped cache entry is inflated, not served as bytes',
          probeText === 'INFLATED-FROM-CACHE',
          probeErr || JSON.stringify(String(probeText).slice(0, 30)) +
          '  (' + compressedSeed.raw + ' -> ' + compressedSeed.packed + ' bytes)');

    /* ---- the offline panel itself -------------------------------------- */

    let panelError = null;
    try {
      await page.goto(base + '/ardupilot/docs/common-offline.html',
                      { waitUntil: 'load', timeout: 20000 });
    } catch (err) {
      panelError = String(err.message).split('\n')[0];
    }
    // The panel is network-only by design; assert only that it falls back.
    check(name, 'offline panel degrades to the fallback rather than an error',
          !panelError, panelError || '');

    /* ---- back online ---------------------------------------------------- */

    const restarted = await start(serverHandle.port);
    global.__server = restarted;
    global.__serverClosed = false;
    serverHandle = null;

    await page.goto(base + NEVER_VISITED, { waitUntil: 'load', timeout: 20000 });
    const back = await looksLikeWikiPage(page);
    check(name, 'the same page loads normally once the network returns',
          !!back.h1, 'h1=' + JSON.stringify(back.h1));

    // WebKit logs its own failed sw.js re-fetch with the origin down.
    const fatal = consoleErrors
      .concat(pageErrors)
      .filter((t) => !/favicon|Failed to load resource/i.test(t))
      .filter((t) => !/\/sw\.js/.test(t))
      // A message naming only third-party URLs is about a third party.
      .filter((t) => {
        const urls = t.match(/https?:\/\/[^\s"')]+/g);
        return !urls || urls.some((u) => u.startsWith(base));
      });
    check(name, 'no unexplained console errors', fatal.length === 0,
          fatal.slice(0, 3).join(' | '));

    // Last, and in a context of its own: see checkUpdateWindow.
    await phase(name, 'deploy check (seeds 8,811 entries)',
                () => checkUpdateWindow(name, browser, base));
    await phase(name, 'opt-out paths (switch, kill switch)',
                () => checkOptOut(name, browser, base));
  } catch (err) {
    check(name, 'engine run completed', false, String(err.message));
  } finally {
    if (!(KEEP && failures)) {
      await context.close().catch(() => {});
      await browser.close().catch(() => {});
    }
  }
}

/* ------------------------------------------------------------------- main -- */

async function main() {
  let playwright;
  try {
    playwright = require('playwright');
  } catch (err) {
    console.error('playwright is not installed.\n\n' +
                  '  npm install --save-dev playwright\n' +
                  '  npx playwright install chromium firefox webkit\n');
    process.exit(2);
  }

  const engines = { chromium: playwright.chromium, firefox: playwright.firefox,
                    webkit: playwright.webkit };

  const handle = await start(0);
  global.__server = handle;
  const base = 'http://localhost:' + handle.port;
  console.log('serving the built tree on ' + base +
              ' (from ' + REPO + ')\n');

  for (const name of WANTED) {
    if (!engines[name]) {
      console.log('\n' + name + '\n  SKIP  unknown engine');
      continue;
    }
    // Each engine gets a live server; runEngine stops and restarts it.
    if (global.__serverClosed) {
      global.__server = await start(handle.port);
      global.__serverClosed = false;
    }
    try {
      await runEngine(name, engines[name], base);
    } catch (err) {
      check(name, 'engine launched', false, String(err.message));
      if (global.__serverClosed) {
        global.__server = await start(handle.port);
        global.__serverClosed = false;
      }
    }
  }

  if (global.__server && !global.__serverClosed) {
    await global.__server.close();
  }

  /* A matrix, because the point of this suite is comparing engines. */
  const names = [];
  results.forEach((r) => {
    if (!names.includes(r.name)) { names.push(r.name); }
  });
  const engineList = WANTED.filter((e) => results.some((r) => r.engine === e));
  const width = Math.max.apply(null, names.map((n) => n.length).concat([20]));

  console.log('\n' + ' '.repeat(width) + '  ' +
              engineList.map((e) => e.padEnd(9)).join(''));
  names.forEach((n) => {
    const cells = engineList.map((e) => {
      const r = results.find((x) => x.engine === e && x.name === n);
      return (r ? (r.ok ? 'pass' : 'FAIL') : '-').padEnd(9);
    });
    console.log(n.padEnd(width) + '  ' + cells.join(''));
  });

  // Where the time went.
  const labels = [];
  timings.forEach((t) => {
    if (!labels.includes(t.label)) { labels.push(t.label); }
  });
  if (labels.length) {
    const lw = Math.max.apply(null, labels.map((l) => l.length).concat([20]));
    console.log('\ntime spent, seconds\n');
    console.log(' '.repeat(lw) + '  ' +
                engineList.map((e) => e.padEnd(9)).join('') + 'total');
    labels.forEach((l) => {
      let row = 0;
      const cells = engineList.map((e) => {
        const t = timings.find((x) => x.engine === e && x.label === l);
        if (!t) { return '-'.padEnd(9); }
        row += t.ms;
        return (t.ms / 1000).toFixed(1).padEnd(9);
      });
      console.log(l.padEnd(lw) + '  ' + cells.join('') + (row / 1000).toFixed(1));
    });
    const grand = timings.reduce((a, t) => a + t.ms, 0);
    console.log('\n' + 'measured total'.padEnd(lw) + '  ' +
                ' '.repeat(9 * engineList.length) + (grand / 1000).toFixed(1) +
                '  (the rest is per-check evaluation)');
  }

  console.log(failures ? '\n' + failures + ' CHECK(S) FAILED\n'
                       : '\nall checks passed\n');
  process.exit(failures ? 1 : 0);
}

main().catch((err) => { console.error(err); process.exit(1); });
