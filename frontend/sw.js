/*
 * The wiki's service worker. Registered by pwa.js once a reader opts in, it
 * sits between every page and the network: pages are served from cache and
 * refreshed behind (stale-while-revalidate), images and fingerprinted assets
 * cache-first, and saved wikis are answered from their ardupilot-offline-*
 * caches. It also streams single-file exports to disk via /__export__/<id>.
 */

// Bump when cached content can no longer be trusted; saved wikis are unaffected.
const CACHE_VERSION = 'v11';
const PAGE_CACHE = `ardupilot-pages-${CACHE_VERSION}`;
const IMAGE_CACHE = `ardupilot-images-${CACHE_VERSION}`;
const STATIC_CACHE = `ardupilot-static-${CACHE_VERSION}`;
// Cross-origin assets that never change.
const THIRD_PARTY_CACHE = `ardupilot-thirdparty-${CACHE_VERSION}`;
const THIRD_PARTY_STATIC =
  /^https:\/\/(i\.creativecommons\.org\/|licensebuttons\.net\/|www\.paypalobjects\.com\/)/;
// Network-only: the offline page and its scripts must always match each other.
const APP_ASSET =
  /(^\/sw\.js$|common_offline(\.css|_page\.js|_export\.js|_document_builder\.js|_unpack\.js|_update\.js)$|common-offline(\.html)?$)/;
// Marks a differential-update request, which must not be answered from cache.
const UPDATE_PARAM = 'ap-update';
const THIRD_PARTY_FRESH = /^https:\/\/firmware\.ardupilot\.org\/useralerts\//;
const CURRENT_CACHES = [PAGE_CACHE, IMAGE_CACHE, STATIC_CACHE, THIRD_PARTY_CACHE];
// Saved wikis, unversioned so they outlive worker updates.
const OFFLINE_CACHE_PREFIX = 'ardupilot-offline-';
// The wiki roots; other single-segment paths are the site's own short links.
const WIKI_ROOT = /^\/(copter|plane|rover|sub|blimp|dev|antennatracker|planner|planner2|ardupilot|mavproxy)$/;

// Deliberately short; everything else is cached as it is visited.
const SHELL = [
  '/',
  '/offline-fallback.html',
  '/manifest.json',
  '/android-icon-192x192.png',
  '/icon-512x512.png',
  '/apple-icon.png',
  '/js/pwa.js',
];

// Network wait for a page that is not cached yet.
const NETWORK_TIMEOUT_MS = 5000;

self.addEventListener('install', (event) => {
  self.skipWaiting();

  event.waitUntil((async () => {
    const cache = await caches.open(STATIC_CACHE);
    // One at a time: addAll() rejects the batch on a single 404.
    await Promise.all(SHELL.map((url) =>
      cache.add(url).catch((err) => console.warn('[sw] shell precache failed', url, err))
    ));
  })());
});

// Theme assets every page loads, warmed once after activation.
const WARM_PER_WIKI = [
  '_static/css/theme.css',
  '_static/js/theme.js',
  '_static/jquery.js',
  '_static/doctools.js',
  '_static/sphinx_highlight.js',
  '_static/common_theme_override.css',
];

// Third-party decoration on every page; the donate button alone was 138 ms.
const WARM_THIRD_PARTY = [
  'https://www.paypalobjects.com/en_US/i/btn/btn_donate_LG.gif',
  'https://i.creativecommons.org/l/by-sa/3.0/88x31.png',
];

async function warmThirdParty() {
  const cache = await caches.open(THIRD_PARTY_CACHE);
  await Promise.all(WARM_THIRD_PARTY.map(async (url) => {
    try {
      if (await cache.match(url)) { return; }
      const response = await fetch(url, { mode: 'no-cors' });
      // An opaque response is stored with megabytes of padded quota;
      // decoration is not worth that, so only real responses are kept.
      if (response && response.ok) { await cache.put(url, response.clone()); }
    } catch (err) { /* decoration; never worth failing activation for */ }
  }));
}

async function warmTheme() {
  const wikis = (await caches.keys())
    .filter((n) => n.startsWith(OFFLINE_CACHE_PREFIX))
    .map((n) => n.slice(OFFLINE_CACHE_PREFIX.length))
    .filter((n) => n !== 'common');
  if (!wikis.length) {
    return;
  }
  const cache = await caches.open(STATIC_CACHE);
  await Promise.all(wikis.flatMap((wiki) => WARM_PER_WIKI.map(async (rel) => {
    const url = `/${wiki}/${rel}`;
    if (await cache.match(url)) {
      return;
    }
    const held = await heldOffline(new Request(url));
    // A saved wiki is a source of bytes, not a trusted one.
    if (held && plausibleBody(new Request(url), held)) {
      await keep(STATIC_CACHE, url, held, true);
    }
  })));
}

self.addEventListener('activate', (event) => {
  event.waitUntil((async () => {
    const names = await caches.keys();
    // Only the versioned caches are disposable; saved wikis survive a bump.
    await Promise.all(
      names
        .filter((name) => name.startsWith('ardupilot-') &&
                          !name.startsWith(OFFLINE_CACHE_PREFIX) &&
                          !CURRENT_CACHES.includes(name))
        .map((name) => caches.delete(name))
    );
    await self.clients.claim();
    // An update run by an uncontrolled Offline page leaves stale promoted
    // copies no CACHES_CHANGED reached; a fresh worker clears them.
    await evictPromotedSavedCopies();
    await warmTheme().catch(() => undefined);
    await warmThirdParty().catch(() => undefined);
  })());
});

// Streaming exports: the page hands over a ReadableStream, answered at /__export__/<id>.
const EXPORTS = new Map();

const EXPORT_TIMEOUT_MS = 60000;

self.addEventListener('message', (event) => {
  const data = event.data;
  if (!data) {
    return;
  }
  if (data.type === 'SKIP_WAITING') {
    self.skipWaiting();
    return;
  }
  if (data.type === 'CACHES_CHANGED') {
    knownCacheNames = null;
    cacheNamesGeneration++;
    openedCaches.clear();
    markerChecked.clear();
    // A saved wiki that just updated leaves stale promoted copies in the
    // browsing caches; drop them so the next read re-promotes fresh bytes.
    event.waitUntil(evictPromotedSavedCopies());
    return;
  }
  if (data.type === 'OFFLINE_OFF') {
    // Pages this instance already controls keep it until they reload;
    // from here it neither caches nor answers, it only passes through.
    offCommanded = true;
    offlineOff = true;
    const done = caches.open(OFF_SENTINEL).catch(() => undefined);
    event.waitUntil(done.then(() => {
      if (data.port) { data.port.postMessage({ type: 'OFFLINE_OFF_ACK' }); }
    }));
    return;
  }
  if (data.type === 'OFFLINE_ON') {
    offCommanded = true;
    offlineOff = false;
    event.waitUntil(caches.delete(OFF_SENTINEL).catch(() => undefined));
    return;
  }
  if (data.type === 'EXPORT_START') {
    // Hold this instance alive until the download is collected.
    let collected;
    const untilCollected = new Promise((resolve) => { collected = resolve; });
    EXPORTS.set(data.id, {
      stream: data.stream, filename: data.filename, collected: collected,
    });
    event.waitUntil(Promise.race([
      untilCollected,
      new Promise((resolve) => setTimeout(resolve, EXPORT_TIMEOUT_MS)),
    ]));
    setTimeout(() => EXPORTS.delete(data.id), EXPORT_TIMEOUT_MS);
  }
});

function isImage(url) {
  return /\/(_images|images)\//.test(url.pathname) ||
         /\.(png|jpe?g|gif|webp|svg|ico)$/i.test(url.pathname);
}

// Named extensions: page names like common-msp-osd-overview-4.2 end in ".2".
const ASSET_EXT_RE =
  /\.(html?|css|m?js|json|xml|txt|map|png|jpe?g|gif|webp|svg|ico|woff2?|ttf|otf|eot|pdf|zip|gz|tgz|tar|mp4|webm)$/i;

// Every shape one URL can have been stored under.
function storedShapes(url) {
  const path = url.pathname;
  const out = [path];

  if (path.endsWith('/')) {
    out.push(path + 'index.html', path.slice(0, -1) + '.html');
  } else if (!ASSET_EXT_RE.test(path)) {
    out.push(path + '.html', path + '/index.html');
  }

  const shared = path.replace(/^\/[^/]+\/_images\//, '/_common/_images/');
  if (shared !== path) {
    out.push(shared);
  }
  return out;
}


// Kept in step with FOLD_INTO_COMMON (build), and the copies in
// common_offline_page.js and common_offline_unpack.js.
const FOLDED_INTO_COMMON = new Set(['ardupilot']);

function likelyCacheName(path) {
  if (path.startsWith('/_common/')) {
    return OFFLINE_CACHE_PREFIX + 'common';
  }
  const first = path.split('/')[1];
  if (!first) {
    return null;
  }
  return OFFLINE_CACHE_PREFIX + (FOLDED_INTO_COMMON.has(first) ? 'common' : first);
}

// A promoted copy is a saved-wiki page or image that cacheFirst copied into
// a browsing cache for speed; once the saved wiki updates, those copies are
// stale and must be dropped so the fresh saved bytes are promoted next time.
async function evictPromotedSavedCopies() {
  try {
    // Only caches that already exist: caches.open() would create an empty
    // one, resurrecting a browsing cache a just-completed opt-out deleted.
    const present = new Set(await caches.keys());
    for (const runtime of [PAGE_CACHE, IMAGE_CACHE, STATIC_CACHE]) {
      if (!present.has(runtime)) { continue; }
      const cache = await caches.open(runtime);
      const requests = await cache.keys();
      await Promise.all(requests.map(async (request) => {
        // Exactly the copies this worker promoted from a saved wiki carry
        // the mark; a page the reader merely browsed does not, and a stale
        // shared image does because it was promoted through the same path.
        const hit = await cache.match(request);
        if (hit && hit.headers && hit.headers.get(PROMOTED_HEADER)) {
          await cache.delete(request);
        }
      }));
    }
  } catch (err) {
    // Best-effort: a failure here only means a slower next read.
  }
}

// caches.open() creates a missing cache, so real names are checked first.
let knownCacheNames = null;
// Bumped by every invalidation, so a refresh that awaited across one
// stands down instead of resurrecting the list it was told to forget.
let cacheNamesGeneration = 0;
const openedCaches = new Map();

// A cache without its /__ap_complete__ marker is an aborted download.
const markerChecked = new Map();

async function isComplete(name) {
  if (!markerChecked.has(name)) {
    markerChecked.set(name, (async () => {
      const cache = await caches.open(name);
      return !!(await cache.match('/__ap_complete__'));
    })());
  }
  return markerChecked.get(name);
}

async function offlineCacheFor(path) {
  const name = likelyCacheName(path);
  if (!name) {
    return undefined;
  }
  if (!knownCacheNames) {
    const generation = cacheNamesGeneration;
    const fresh = new Set(await caches.keys());
    if (generation === cacheNamesGeneration) {
      knownCacheNames = fresh;
    } else {
      // Invalidated mid-read: this snapshot predates the change, and
      // answering from it could resurrect a cache the reader deleted.
      // The exhaustive scan behind this lookup stays correct, just slower.
      return undefined;
    }
  }
  if (!knownCacheNames.has(name)) {
    return undefined;
  }
  if (!(await isComplete(name))) {
    return undefined;
  }
  if (!openedCaches.has(name)) {
    openedCaches.set(name, caches.open(name));
  }
  return openedCaches.get(name);
}

// The unpacker stores text gzipped; no engine honours Content-Encoding on a
// body a worker hands back, so inflate here.
const AP_ENCODED = 'x-ap-encoding';

function inflate(response) {
  if (!response || !response.headers ||
      response.headers.get(AP_ENCODED) !== 'gzip') {
    return response;
  }
  if (typeof DecompressionStream !== 'function') {
    console.warn('[sw] stored compressed but this browser cannot inflate');
    return undefined;
  }
  const headers = new Headers(response.headers);
  headers.delete(AP_ENCODED);
  return new Response(
    response.body.pipeThrough(new DecompressionStream('gzip')),
    { status: 200, statusText: 'OK', headers }
  );
}

// Exact matches only: ignoreSearch walks the whole cache (0.2 ms vs 300 ms).
// savedOnly limits the search to complete saved wikis: their copies are
// rewritten in place by updates, which is what makes them authoritative.
async function heldOffline(request, cache, savedOnly) {
  const shapes = storedShapes(new URL(request.url));

  if (cache) {
    for (const path of shapes) {
      const hit = await cache.match(path);
      if (hit) {
        return inflate(hit);
      }
    }
    return undefined;
  }

  for (const path of shapes) {
    const only = await offlineCacheFor(path);
    if (only) {
      const hit = await only.match(path);
      if (hit) {
        return inflate(hit);
      }
    }
  }

  // Not caches.match(): it would answer from a half-written download.
  const names = await caches.keys();
  for (const name of names) {
    if (savedOnly && !name.startsWith(OFFLINE_CACHE_PREFIX)) {
      continue;
    }
    if (name.startsWith(OFFLINE_CACHE_PREFIX) && !(await isComplete(name))) {
      continue;
    }
    const candidate = await caches.open(name);
    for (const path of shapes) {
      const hit = await candidate.match(path);
      if (hit) {
        return inflate(hit);
      }
    }
  }
  return undefined;
}

function isPage(url) {
  return /\.html?$/.test(url.pathname) || url.pathname.endsWith('/');
}

// The version index behind the dropdown on every parameters page.
const PARAM_INDEX = /^\/([^/]+)\/_static\/parameters-[A-Za-z0-9_]+\.json$/;

// Offline, the version index lists only the versions held. Nothing filtered is stored.
async function paramIndex(request, url) {
  try {
    const fresh = await fetch(request);
    if (fresh && fresh.ok) {
      await keep(STATIC_CACHE, request, fresh.clone());
      return fresh;
    }
  } catch (err) {
    // Offline; fall through to the stored index.
  }

  const held = await heldOffline(request);
  if (!held) {
    // Nothing stored: hand back the server's real answer, 404s included.
    return fetch(request);
  }

  let index;
  try {
    index = await held.clone().json();
  } catch (err) {
    return held;               // not the shape we expected; do not mangle it
  }

  const wiki = url.pathname.split('/')[1];
  const out = {};
  for (const label of Object.keys(index)) {
    // Values are bare filenames relative to docs/.
    const target = new URL('/' + wiki + '/docs/' + index[label], url.origin);
    if (await heldOffline(new Request(target.href))) {
      out[label] = index[label];
    }
  }

  // An empty dropdown is worse than an over-full one.
  if (!Object.keys(out).length) {
    return held;
  }

  return new Response(JSON.stringify(out), {
    status: 200,
    statusText: 'OK',
    headers: { 'Content-Type': 'application/json' }
  });
}

function isStatic(url) {
  return /\/(_static|fonts)\//.test(url.pathname);
}

/** Tell every open page under this scope that a resource changed. */
async function notifyClients(message) {
  const clients = await self.clients.matchAll({ type: 'window' });
  clients.forEach((client) => client.postMessage(message));
}

// Serve the cached copy, refresh behind, announce a difference.
async function staleWhileRevalidate(request, cacheName, announceChanges, event) {
  const cache = await caches.open(cacheName);
  // A saved wiki's page is rewritten and would always read as "changed".
  const fromPageCache = await heldOffline(request, cache);
  // The saved copy leads: updates rewrite it in place, while the browsing
  // cache holds whatever was read last, which may be older. savedOnly, or
  // an ordinary browsing hit would masquerade as a saved one and silence
  // the changed-page announcement for readers who never saved anything.
  const fromSaved = await heldOffline(request, undefined, true);
  const cached = fromSaved || fromPageCache;

  // Clone before the browser consumes the body. Announced only when the
  // browsing copy was the one served: the saved copy is rewritten and
  // would read as changed forever.
  const cachedForCompare = (announceChanges && !fromSaved && fromPageCache)
    ? fromPageCache.clone() : null;

  // Revalidate with the server, not the HTTP cache; see networkOnly.
  const refresh = cached
    ? new Request(request.url, { cache: 'no-cache', credentials: 'same-origin' })
    : request;

  const network = fetch(refresh).then(async (response) => {
    if (!response || !response.ok) {
      if (response && (response.status === 404 || response.status === 410) &&
          fromPageCache) {
        // Gone from the server: the browsing copy goes too. Saved wikis
        // live in their own caches and are never touched here.
        const evict = await caches.open(cacheName);
        await evict.delete(request);
      }
      return response;
    }
    if (cachedForCompare) {
      const [oldText, newText] = await Promise.all([
        cachedForCompare.text(),
        response.clone().text(),
      ]);
      if (oldText !== newText) {
        // Awaited, so the announcement cannot outlive the waitUntil promise.
        await notifyClients({ type: 'PAGE_UPDATED', url: request.url });
      }
    }
    if (plausibleBody(request, response)) {
      await keep(cacheName, request, response);
    }
    return response;
  }).catch((err) => {
    console.warn('[sw] revalidate failed for', request.url, err && err.message);
    return undefined;
  });

  keepAlive(event, network, request.url);

  if (cached) {
    return unredirect(cached);
  }

  const timeout = new Promise((resolve) => setTimeout(resolve, NETWORK_TIMEOUT_MS));
  const response = await Promise.race([network, timeout]);
  if (response) {
    return unredirect(response);
  }

  return (await caches.match('/offline-fallback.html')) ||
         new Response('Offline and this page has not been saved.', {
           status: 503,
           headers: { 'Content-Type': 'text/plain' },
         });
}

// A navigation may not be answered with a redirected response.
function unredirect(response) {
  if (!response || !response.redirected) {
    return response;
  }
  return new Response(response.body, {
    status: response.status,
    statusText: response.statusText,
    headers: response.headers
  });
}

/** Always ask the network; use a cached copy only if there is no network. */
async function networkOnly(request) {
  try {
    // Untouched: a rebuilt 'navigate' Request throws.
    const response = await fetch(request);
    if (response && response.ok && plausibleBody(request, response)) {
      await keep(PAGE_CACHE, request, response);
    }
    return unredirect(response);
  } catch (err) {
    return (await heldOffline(request)) ||
           (await caches.match('/offline-fallback.html')) ||
           new Response('Offline.', { status: 503 });
  }
}

// Stale-while-revalidate keyed without the cache-busting query.
async function freshBehind(request, cacheName, event) {
  const cache = await caches.open(cacheName);
  const key = new URL(request.url);
  key.search = '';
  const stored = await cache.match(key.href);

  const network = fetch(request)
    .then(async (response) => {
      if (response && (response.ok || response.type === 'opaque') &&
          plausibleBody(request, response)) {
        await keep(cacheName, key.href, response);
      }
      return response;
    })
    .catch(() => undefined);

  keepAlive(event, network, request.url);

  if (stored) {
    return stored;
  }
  return (await network) || new Response('', { status: 504 });
}

// Same-URL files change across builds; each is re-asked once per worker life.
const revalidated = new Set();

function maybeRevalidate(request, cacheName, event) {
  // A query is a fingerprint and needs no second look.
  if (revalidated.has(request.url) || new URL(request.url).search !== '') {
    return;
  }
  revalidated.add(request.url);
  keepAlive(event, fetch(request).then(async (response) => {
    // The page is gone upstream; serving it forever would be lying.
    if (response && (response.status === 404 || response.status === 410)) {
      await (await caches.open(cacheName)).delete(request);
      return;
    }
    if (response && response.ok && plausibleBody(request, response)) {
      return keep(cacheName, request, response);
    }
  }).catch(() => { /* offline; the stored copy stands */ }), request.url);
}

async function cacheFirst(request, cacheName, event) {
  // heldOffline matches by path and cannot find a cross-origin URL.
  const exact = await (await caches.open(cacheName)).match(request);
  if (exact) {
    maybeRevalidate(request, cacheName, event);
    return exact;
  }

  // A query means a fingerprint: the saved wiki may hold another build's
  // bytes, so the network answers first and the fallback is never promoted.
  const fingerprinted = new URL(request.url).search !== '';
  if (!fingerprinted) {
    const held = await heldOffline(request);
    if (held) {
      // Promote into the named cache; a saved wiki is not a trusted source.
      if (plausibleBody(request, held)) {
        await keep(cacheName, request, held, true);
      }
      return held;
    }
  }
  try {
    const response = await fetch(request);
    // Opaque cross-origin responses report status 0 and are still usable.
    if (response && (response.ok || response.type === 'opaque') &&
        plausibleBody(request, response)) {
      await keep(cacheName, request, response);
    }
    return response;
  } catch (err) {
    return (await heldOffline(request)) || new Response('', { status: 504 });
  }
}

// Refuse a body that contradicts the request: captive wifi serves login pages at 200.
const CONTENT_EXPECTATIONS = [
  [/\.css$/i, /^text\/css/],
  [/\.m?js$/i, /^(?:application|text)\/(?:x-)?(?:java|ecma)script/],
  [/\.(?:png|jpe?g|gif|webp|avif|svg|ico)$/i, /^image\//],
  [/\.json$/i, /^application\/json/],
];

function plausibleBody(request, response) {
  if (!response || response.type === 'opaque') { return true; }
  const ct = (response.headers && response.headers.get('Content-Type') || '')
    .toLowerCase();
  if (!ct) { return true; }
  let path;
  try {
    path = new URL(typeof request === 'string' ? request : request.url).pathname;
  } catch (err) {
    return true;
  }
  for (const [pattern, expected] of CONTENT_EXPECTATIONS) {
    if (pattern.test(path)) {
      if (expected.test(ct)) { return true; }
      console.warn('[sw] refusing to cache', path, 'served as', ct);
      return false;
    }
  }
  return true;
}

function keepAlive(event, promise, url) {
  if (!event || typeof event.waitUntil !== 'function') {
    return;
  }
  try {
    event.waitUntil(promise);
  } catch (err) {
    console.warn('[sw] waitUntil refused for', url, err && err.name);
  }
}

// A failed store (QuotaExceededError) must not change what the reader gets.
// A network response never carries the unpacker's private encoding marker,
// and a stored redirected response would fail every navigation it answers.
function sanitizeForCache(response) {
  const marked = response.headers && response.headers.get &&
                 response.headers.get('x-ap-encoding');
  if (!response.redirected && !marked) {
    return response;
  }
  const headers = new Headers();
  if (response.headers && response.headers.forEach) {
    response.headers.forEach((value, name) => {
      if (name.toLowerCase() !== 'x-ap-encoding') { headers.append(name, value); }
    });
  }
  return new Response(response.body, {
    status: response.status,
    statusText: response.statusText,
    headers,
  });
}

// Stamped on a copy promoted from a saved wiki, so eviction is an exact
// test of what this worker put there rather than a guess from the path.
const PROMOTED_HEADER = 'x-ap-promoted';

async function keep(cacheName, key, response, promoted) {
  try {
    await offRestored;
    if (offlineOff) { return; }
    const cache = await caches.open(cacheName);
    let toStore = sanitizeForCache(response.clone());
    if (promoted) {
      const headers = new Headers(toStore.headers);
      headers.set(PROMOTED_HEADER, '1');
      toStore = new Response(toStore.body, {
        status: toStore.status, statusText: toStore.statusText, headers });
    }
    await cache.put(key, toStore);
  } catch (err) {
    console.warn('[sw] could not store', String(key && key.url ? key.url : key),
                 err && err.name);
  }
}

// Never store the archives or the manifest, or anything far larger than a search index.
const NEVER_STORE = /^\/offline\//;
const STORE_LIMIT_BYTES = 12 * 1024 * 1024;

function storable(url, response) {
  if (NEVER_STORE.test(url.pathname)) {
    return false;
  }
  const len = Number(response.headers.get('Content-Length'));
  return !(len > STORE_LIMIT_BYTES);
}

function safely(handler, request) {
  return handler.catch(async (err) => {
    console.warn('[sw] handler failed, passing through', err);
    try {
      return await fetch(request);
    } catch (netErr) {
      return (await heldOffline(request)) ||
             (await caches.match('/offline-fallback.html')) ||
             new Response('Offline.', { status: 503 });
    }
  });
}

// Set when the reader turns offline mode off while pages are still open.
// The browser freely terminates and restarts idle workers, so the flag is
// also persisted as a sentinel cache outside the ardupilot- wipe prefix,
// and restored before anything is stored.
const OFF_SENTINEL = 'ap-offline-off';
let offlineOff = false;
// An explicit command outranks the startup restore: an OFFLINE_ON landing
// while the sentinel read is in flight must not be overwritten by it.
let offCommanded = false;
const offRestored = Promise.resolve()
  .then(() => caches.has(OFF_SENTINEL))
  .then((off) => { if (off && !offCommanded) { offlineOff = true; } })
  .catch(() => undefined);

self.addEventListener('fetch', (event) => {
  const request = event.request;

  // respondWith must be claimed synchronously, so this reads the flag as
  // it stands; a restarted instance may answer its first request before
  // the sentinel is read, but keep() awaits the restore, so nothing is
  // ever stored during that window, and the wipe has usually left nothing
  // to answer with anyway.
  if (offlineOff || request.method !== 'GET') {
    return;
  }

  const url = new URL(request.url);

  if (url.origin !== self.location.origin) {
    // User alerts must stay current.
    if (THIRD_PARTY_STATIC.test(url.href)) {
      event.respondWith(safely(cacheFirst(request, THIRD_PARTY_CACHE, event), request));
    } else if (THIRD_PARTY_FRESH.test(url.href)) {
      event.respondWith(safely(freshBehind(request, THIRD_PARTY_CACHE, event), request));
    }
    return;
  }

  if (url.pathname.startsWith('/__export__/')) {
    const id = url.pathname.slice('/__export__/'.length);
    const entry = EXPORTS.get(id);
    if (entry) {
      EXPORTS.delete(id);
      // Collected: the response itself keeps the worker alive while it streams.
      if (entry.collected) { entry.collected(); }
      event.respondWith(new Response(entry.stream, {
        headers: {
          'Content-Type': 'application/octet-stream',
          'Content-Disposition':
            // Header values are Latin-1 and single-line; anything else in a
            // name would make the Response constructor throw the export away.
            'attachment; filename="' +
            String(entry.filename).replace(/[^\x20-\x7e]/g, '_')
              .replace(/"/g, '') + '"'
        }
      }));
    } else {
      event.respondWith(new Response('Export expired.', { status: 410 }));
    }
    return;
  }
  // An update must reach the server and never fall back to the copy it replaces.
  if (url.searchParams.has(UPDATE_PARAM)) {
    event.respondWith(fetch(request));
    return;
  }

  // The theme requests an image it does not ship; see KNOWN_UPSTREAM_ISSUES.md.
  if (url.pathname.endsWith('/_static/images/mainnav-sep-2.gif')) {
    event.respondWith(new Response(
      Uint8Array.from(atob('R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7'),
                      (c) => c.charCodeAt(0)),
      { headers: { 'Content-Type': 'image/gif',
                   'Cache-Control': 'public, max-age=31536000' } }));
    return;
  }

  if (url.pathname === '/js/pwa.js') {
    event.respondWith(safely(staleWhileRevalidate(request, STATIC_CACHE, false, event), request));
    return;
  }

  if (APP_ASSET.test(url.pathname)) {
    event.respondWith(safely(networkOnly(request), request));
    return;
  }

  // A wiki root without its slash: send it to the slash form, as the server
  // does, or every relative link on the page resolves one level up.
  if (request.mode === 'navigate' && WIKI_ROOT.test(url.pathname)) {
    event.respondWith(new Response(null, {
      status: 301, headers: { Location: url.pathname + '/' + url.search }
    }));
    return;
  }

  // Routed on the URL too: a prefetch arrives as mode "cors" with no destination.
  if (request.mode === 'navigate' || request.destination === 'document' ||
      isPage(url)) {
    event.respondWith(safely(staleWhileRevalidate(request, PAGE_CACHE, true, event), request));
    return;
  }

  if (isImage(url)) {
    event.respondWith(safely(cacheFirst(request, IMAGE_CACHE, event), request));
    return;
  }

  // Before isStatic: this one must try the network every time.
  if (PARAM_INDEX.test(url.pathname)) {
    event.respondWith(safely(paramIndex(request, url), request));
    return;
  }

  if (isStatic(url)) {
    // Fingerprinted (?v=5d32c60e), so a stored copy is never the wrong one.
    event.respondWith(safely(cacheFirst(request, STATIC_CACHE, event), request));
    return;
  }

  // Everything else, notably searchindex.js and objects.inv.
  event.respondWith((async () => {
    try {
      const response = await fetch(request);
      if (response && response.ok && storable(url, response) &&
          plausibleBody(request, response)) {
        await keep(STATIC_CACHE, request, response);
      }
      return response;
    } catch (err) {
      return (await heldOffline(request)) || new Response('', { status: 504 });
    }
  })());
});
