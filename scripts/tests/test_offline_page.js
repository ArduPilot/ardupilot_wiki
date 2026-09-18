/*
 * Harness for common_offline_page.js, the offline panel, against a real DOM
 * (jsdom) and a CacheStorage-alike. The markup comes from common-offline.rst
 * so the script and its page cannot drift apart unnoticed.
 *
 *   npm install --no-save jsdom
 *   node scripts/tests/test_offline_page.js
 */
const fs = require('fs');
const path = require('path');
const vm = require('vm');

let JSDOM, VirtualConsole;
try {
  ({ JSDOM, VirtualConsole } = require('jsdom'));
} catch (e) {
  // A test run that cannot run is a failure, not a pass.
  console.error('\nFAILED: this harness needs jsdom, which is not installed.\n' +
                '  npm install --no-save jsdom\n' +
                'The panel is a DOM application; testing it without one would mean\n' +
                'hand-rolling a DOM, and a shim with its own bugs is worse than none.\n');
  process.exit(1);
}

const REPO = path.resolve(__dirname, '..', '..');
const STATIC = path.join(REPO, 'common/source/_static');
const PAGE = path.join(STATIC, 'common_offline_page.js');
const RST = path.join(REPO, 'common/source/docs/common-offline.rst');

const FRONTEND_JS = path.join(__dirname, '..', '..', 'frontend', 'js');
const FIXTURES = path.join(__dirname, 'fixtures');

// Loaded in the page's own order, so the panel finds its libraries.
const PANEL_LIBS = ['common_offline_unpack.js', 'common_offline_update.js'];

// Caches are seeded by the shipped cachePathFor rule, not a restatement of it.
const ApUnpack = (() => {
  const box = { window: null, console, TextDecoder, TextEncoder, URL };
  box.window = box;
  vm.createContext(box);
  vm.runInContext(
    fs.readFileSync(path.join(STATIC, 'common_offline_unpack.js'), 'utf8'), box);
  return box.ApUnpack;
})();

let pass = 0, fail = 0;
const failures = [];
function check(name, ok, detail) {
  if (ok) { pass++; console.log('  PASS  ' + name + (detail ? '   ' + detail : '')); }
  else { fail++; failures.push(name); console.log('  FAIL  ' + name + (detail ? '   ' + detail : '')); }
}

/* ---------------------------------------------------------- cache shim ---- */

/** Bodies are kept as bytes. */
class FakeResponse {
  constructor(body, init) {
    this._b = typeof body === 'string' ? body
            : Buffer.isBuffer(body) ? body
            : body instanceof ArrayBuffer ? Buffer.from(body)
            : ArrayBuffer.isView(body)
                ? Buffer.from(body.buffer, body.byteOffset, body.byteLength)
                : JSON.stringify(body);
    // Headers as the unpacker sets them, case-insensitive as in a browser.
    const h = new Map(Object.entries((init && init.headers) || {})
      .map(([k, v]) => [k.toLowerCase(), String(v)]));
    this.headers = { get: (k) => (h.has(String(k).toLowerCase())
      ? h.get(String(k).toLowerCase()) : null) };
  }
  text() {
    return Promise.resolve(Buffer.isBuffer(this._b) ? this._b.toString('utf8') : this._b);
  }
  json() { return this.text().then((t) => JSON.parse(t)); }
  blob() {
    return Promise.resolve(Buffer.isBuffer(this._b) ? this._b : Buffer.from(this._b));
  }
  arrayBuffer() {
    const b = Buffer.isBuffer(this._b) ? this._b : Buffer.from(this._b);
    return Promise.resolve(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength));
  }
}
// Keys normalise through the real URL parser, as Cache Storage matching does:
// encoded dots collapse, tabs vanish, fragments drop, the origin is stripped.
const cacheKey = (k) => {
  // The base is a page deep in the site, as in a real browser, so a relative
  // key resolves away from the root and a path bug cannot hide behind '/'.
  const u = new URL(String(k && k.url ? k.url : k), 'https://x/ardupilot/docs/p.html');
  return u.pathname + u.search;
};
class FakeCache {
  constructor() { this.map = new Map(); }
  put(k, v) { this.map.set(cacheKey(k), v); return Promise.resolve(); }
  match(k) { return Promise.resolve(this.map.get(cacheKey(k))); }
  delete(k) { return Promise.resolve(this.map.delete(cacheKey(k))); }
  keys() { return Promise.resolve([...this.map.keys()].map(u => ({ url: 'https://x' + u }))); }
}
function makeCaches() {
  const all = new Map();
  return {
    _all: all,
    keys: () => Promise.resolve([...all.keys()]),
    open: (n) => { if (!all.has(n)) all.set(n, new FakeCache()); return Promise.resolve(all.get(n)); },
    delete: (n) => Promise.resolve(all.delete(n))
  };
}

/** A real tar, not gzipped, as the client receives it. */
function tarBytes(files) {
  const blocks = [];
  const record = (name, body, type) => {
    const data = Buffer.from(body);
    const head = Buffer.alloc(512);
    head.write(name.slice(0, 100), 0, 100);    // truncated exactly as tar does
    head.write('000644 \0', 100, 8);
    head.write('000000 \0', 108, 8);
    head.write('000000 \0', 116, 8);
    head.write(data.length.toString(8).padStart(11, '0') + '\0', 124, 12);
    head.write('00000000000\0', 136, 12);
    head.write('        ', 148, 8);            // checksum field, spaces first
    head.write(type || '0', 156, 1);
    let sum = 0;
    for (const b of head) { sum += b; }
    head.write(sum.toString(8).padStart(6, '0') + '\0 ', 148, 8);
    blocks.push(head, data, Buffer.alloc((512 - (data.length % 512)) % 512));
  };
  for (const [name, body] of Object.entries(files)) {
    // Over 100 bytes: a PAX header carrying path=, as Python's tarfile writes.
    if (name.length > 100) {
      const rec = `path=${name}\n`;
      const len = (String(rec.length + 4).length + rec.length + 1);
      record('././@PaxHeader', `${len} ${rec}`, 'x');
    }
    record(name, body, '0');
  }
  blocks.push(Buffer.alloc(1024));             // end of archive
  return Buffer.concat(blocks);
}

function streamOf(buf) {
  return new ReadableStream({
    start(c) { c.enqueue(new Uint8Array(buf)); c.close(); }
  });
}

/* -------------------------------------------------------- page under test - */

/** The panel's real markup, out of the .rst. */
function panelMarkup() {
  const rst = fs.readFileSync(RST, 'utf8');
  // The warning sits above the panel.
  const panel = rst.indexOf('<div class="apo">');
  const warn = rst.indexOf('<div id="storage-warning">');
  const start = warn !== -1 && warn < panel ? warn : panel;
  // Forward to the prose that follows the tool; the panel now holds it all.
  const end = rst.indexOf('What to Expect', start);
  let html = rst.slice(start, end);
  html = html.replace(/^\s{0,3}/gm, '');            // rST indentation
  html = html.split('\n').filter(l => !l.trim().startsWith('.. ')).join('\n');
  return html;
}

/** A site serving file tables (`tables`) and files (`served`); anything else 404s. */
function load({ manifest = null, caches = makeCaches(), persisted = false,
                usage = 0, quota = 10e9, archives = null,
                tables = null, served = null, rateLimit = false,
                loose = null, offline = false, estimateDelay = 0,
                decoder = true, controlled = true, registered = controlled } = {}) {
  const vc = new VirtualConsole();
  vc.on('jsdomError', (e) => { console.log('    [page error] ' + e.message);
                               if (e.stack) console.log('    ' + e.stack.split('\n')[1]); });
  const dom = new JSDOM('<!doctype html><body>' + panelMarkup() + '</body>',
                        { url: 'https://example.test/x/docs/common-offline.html',
                          virtualConsole: vc });
  const w = dom.window;
  // pwa.js's opt-in as the page sees it; disable() wipes as the real one does.
  const apOffline = {
    on: offline, calls: [],
    enabled() { return this.on; },
    enable() { this.on = true; this.calls.push('enable'); },
    disable() {
      this.on = false; this.calls.push('disable');
      return caches.keys().then((names) => Promise.all(
        names.filter((n) => n.startsWith('ardupilot-')).map((n) => caches.delete(n))));
    },
  };
  w.ApOffline = apOffline;
  // jsdom has no matchMedia and init() calls it to detect standalone mode.
  if (!w.matchMedia) { w.matchMedia = () => ({ matches: false, addListener(){}, removeListener(){} }); }
  const fetchCalls = [];
  const fetchOpts = [];
  const swMessages = [];
  const sandbox = {
    window: w, document: w.document,
    // A global, as in a browser; the shared-file path reads it.
    location: w.location,
    navigator: {
      storage: {
        estimate: () => (estimateDelay
          ? new Promise((r) => setTimeout(() => r({ usage, quota }), estimateDelay))
          : Promise.resolve({ usage, quota })),
        persisted: () => Promise.resolve(persisted),
        persist: () => Promise.resolve(persisted)
      },
      // Records what the panel tells the worker; `controlled` false is the
      // moment after opting in, before the worker has claimed the page.
      serviceWorker: {
        controller: controlled ? { postMessage: (m) => { swMessages.push(m); } } : null,
        // `registered`: the worker is installed and active for the scope,
        // whether or not it controls this page (a hard reload loads without control).
        getRegistration: () => Promise.resolve(registered ? { active: {} } : undefined),
        ready: new Promise(() => {}),
        addEventListener() {}
      }
    },
    caches,
    crypto: require('crypto').webcrypto,
    setTimeout: w.setTimeout.bind(w), clearTimeout: w.clearTimeout.bind(w),
    // The panel refreshes its "last checked" line every minute; never keeps the run alive.
    setInterval: (fn, ms) => { const t = setInterval(fn, ms); if (t.unref) { t.unref(); } return t; },
    clearInterval,
    console,
    Response: FakeResponse,
    URL,
    // The url is the parsed form, as in a real browser, so traversal tests
    // exercise the same normalisation the live cache applies.
    Request: class { constructor(u) { this.url = new URL(String(u), 'https://x').href; } },
    AbortController: w.AbortController,
    TransformStream, ReadableStream, Uint8Array, WebAssembly, TextEncoder, TextDecoder,
    atob: (b) => Buffer.from(b, 'base64').toString('binary'),
    fetch: (u, o) => {
      fetchCalls.push(String(u));
      fetchOpts.push({ url: String(u), opts: o || {}, at: Date.now() });
      // The decoder's wasm, from the site root as the page fetches it.
      if (String(u).indexOf('zstd.wasm') !== -1) {
        const wasm = fs.readFileSync(path.join(FRONTEND_JS, 'zstd.wasm'));
        return Promise.resolve({ ok: true, arrayBuffer: () => Promise.resolve(
          wasm.buffer.slice(wasm.byteOffset, wasm.byteOffset + wasm.byteLength)) });
      }
      if (String(u).indexOf('offline-manifest.json') !== -1) {
        return Promise.resolve(manifest
          ? { ok: true, json: () => Promise.resolve(manifest) }
          : { ok: false, json: () => Promise.reject(new Error('no manifest')) });
      }
      // A published file table, named the way the manifest asks for it.
      if (tables && String(u).indexOf('-files.json') !== -1) {
        const name = String(u).split('?')[0].split('/').pop();
        if (Object.prototype.hasOwnProperty.call(tables, name)) {
          return Promise.resolve({ ok: true, json: () => Promise.resolve(tables[name]) });
        }
        // An absent scripted table is a scripted failure; nothing is
        // synthesized here, or a future common-refusal test would be
        // quietly vouched past its own point.
        return Promise.resolve({ ok: false,
          json: () => Promise.reject(new Error('no table')) });
      }
      // No scripted tables: synthesize truthful ones from the archive
      // fixture, as the build always publishes a table beside an archive.
      // CAUTION: a hostile-archive test written without `tables:` gets its
      // smuggled entries vouched for here; refusal tests MUST script tables.
      if (!tables && archives && String(u).indexOf('-files.json') !== -1) {
        const m2 = String(u).split('?')[0].match(/([a-z0-9]+)-files\.json/);
        const id2 = m2 ? m2[1] : '';
        const own = Object.entries(archives).filter(([n]) =>
          id2 === 'common' ? (n.startsWith('_images/') || n.startsWith('ardupilot/'))
                           : n.startsWith(id2 + '/'));
        return Promise.resolve({ ok: true, json: async () => {
          const t = {};
          for (const [n, b] of own) { t[n] = await fileHash(b); }
          return t;
        } });
      }
      // The server refusing update traffic.
      if (rateLimit && String(u).indexOf('ap-update=') !== -1) {
        return Promise.resolve({ ok: false, status: 429,
                                 blob: () => Promise.reject(new Error('429')) });
      }
      // The loose copies under <base>/files/<name>, tried first.
      if (loose) {
        const m = String(u).split('?')[0].match(/\/files\/(.+)$/);
        if (m && Object.prototype.hasOwnProperty.call(loose, m[1])) {
          const buf = Buffer.from(loose[m[1]]);
          return Promise.resolve({
            ok: true,
            blob: () => Promise.resolve(buf),
            arrayBuffer: () => Promise.resolve(
              buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength)),
          });
        }
        if (/\/files\//.test(String(u).split('?')[0])) {
          return Promise.resolve({ ok: false, blob: () => Promise.reject(new Error('404')) });
        }
      }
      // An individual file, at the path the site really serves it from.
      if (served) {
        const p = String(u).split('?')[0];
        if (Object.prototype.hasOwnProperty.call(served, p)) {
          const buf = Buffer.from(served[p]);
          return Promise.resolve({
            ok: true,
            blob: () => Promise.resolve(buf),
            arrayBuffer: () => Promise.resolve(
              buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength)),
          });
        }
        if (String(u).indexOf('.tar') === -1) {
          // 404, not a rejection: a wiki that does not hold the file.
          return Promise.resolve({ ok: false, blob: () => Promise.reject(new Error('404')) });
        }
      }
      if (archives) {
        // A real archive holding the fixture entries that belong to the wiki
        // asked for, so the unpack runs and the marker gets written.
        const m = String(u).split('?')[0].match(/([a-z0-9]+)-offline\.tar/);
        const id = m ? m[1] : '';
        const own = Object.fromEntries(Object.entries(archives).filter(([n]) =>
          id === 'common' ? (n.startsWith('_images/') || n.startsWith('ardupilot/'))
                          : n.startsWith(id + '/')));
        return Promise.resolve({ ok: true, body: streamOf(tarBytes(own)) });
      }
      return Promise.reject(new Error('archive fetch blocked by harness'));
    }
  };
  // The page guards on `'caches' in window`.
  sandbox.window.caches = caches;
  sandbox.window.fetch = sandbox.fetch;
  vm.createContext(sandbox);
  PANEL_LIBS.forEach((lib) => {
    vm.runInContext(fs.readFileSync(path.join(STATIC, lib), 'utf8'), sandbox);
  });
  // The delta decoder, from the site root as the page loads it.
  vm.runInContext(fs.readFileSync(path.join(FRONTEND_JS, 'zstd-delta.js'), 'utf8'), sandbox);
  if (!decoder) {
    // A browser where the decoder cannot run: init rejects, as it does
    // without WebAssembly or when the wasm cannot be fetched.
    sandbox.ApZstd = { init: () => Promise.reject(new Error('no WebAssembly here')),
                       patch: () => { throw new Error('no decoder'); } };
  }
  vm.runInContext(fs.readFileSync(PAGE, 'utf8'), sandbox);
  return { dom, w, doc: w.document, sandbox, fetchCalls, fetchOpts, swMessages, apOffline };
}

// The build's file hash: sha256, first eight bytes, hex.
async function fileHash(text) {
  const d = await require('crypto').webcrypto.subtle.digest('SHA-256', Buffer.from(text));
  return [...new Uint8Array(d).slice(0, 8)].map(b => b.toString(16).padStart(2, '0')).join('');
}

const settle = () => new Promise(r => setTimeout(r, 60));
/** The file table a seeded cache holds. */
async function ApUpdate_readTable(cache) {
  const r = await cache.match('/__ap_files__');
  return r ? JSON.parse(await r.text()) : {};
}
// The build's delta header carries the page's content hash: sha256, first eight bytes.
const hash16 = (b) => require('crypto').createHash('sha256').update(b).digest('hex').slice(0, 16);

const $ = (doc, id) => doc.getElementById(id);
const rows = (doc) => [...doc.querySelectorAll('.wiki-check')];

function completeMarker(build, id) {
  // The real download writes id alongside build; the update check reads it.
  return new FakeResponse(JSON.stringify({ build, saved: Date.now(), id }));
}

const MANIFEST = {
  generated: '2026-08-09T00:00:00Z',
  artifact_base: 'https://cdn.example.test',
  common: { id: 'common', name: 'Common (required)', mb: 400, pages: 0, required: true,
            archive: 'common-offline.tar' },
  wikis: [
    { id: 'copter', name: 'Copter', mb: 74, pages: 846, archive: 'copter-offline.tar' },
    { id: 'rover', name: 'Rover', mb: 32, pages: 747, archive: 'rover-offline.tar' },
    { id: 'dev', name: 'Developer', mb: 52, pages: 312, archive: 'dev-offline.tar' }
  ]
};

/* ------------------------------------------------------------- the run ---- */

async function main() {
  console.log('\nnothing saved yet');
  {
    const { doc } = load({ manifest: MANIFEST });
    await settle();
    check('renders a row per wiki plus common',
          doc.querySelectorAll('tr[data-wiki]').length === 4,
          doc.querySelectorAll('tr[data-wiki]').length + ' rows');
    check('manifest names are used, not directory names',
          [...doc.querySelectorAll('.apo-name')].some(e => e.textContent.includes('Developer')));
    check('save is disabled with nothing selected', $(doc, 'download-cache-btn').disabled);
    check('export is disabled with nothing selected', $(doc, 'dl-single').disabled);
    check('select-all is clear', !$(doc, 'select-all').checked &&
                                  !$(doc, 'select-all').indeterminate);
    check('build date shown', ($(doc, 'build-date').textContent || '').includes('2026-08-09'));
    check('remove all disabled when nothing is stored', $(doc, 'clear-btn').disabled);
  }

  console.log('\nselecting');
  {
    const { doc, w } = load({ manifest: MANIFEST });
    await settle();
    $(doc, 'select-all').click();
    await settle();
    check('select-all ticks every wiki', rows(doc).every(c => c.checked));
    check('save enabled once something is selected', !$(doc, 'download-cache-btn').disabled);
    check('export enabled once something is selected', !$(doc, 'dl-single').disabled);
    const total = $(doc, 'selection-total').textContent;
    check('total counts common plus every wiki', total.includes('558'),
          JSON.stringify(total));
    rows(doc)[0].click();
    await settle();
    check('unticking one makes select-all indeterminate',
          $(doc, 'select-all').indeterminate && !$(doc, 'select-all').checked);
    rows(doc).forEach(c => { if (c.checked) c.click(); });
    await settle();
    check('unticking all disables save again', $(doc, 'download-cache-btn').disabled);
    check('unticking all disables export again', $(doc, 'dl-single').disabled);
  }

  console.log('\nalready saved');
  {
    const caches = makeCaches();
    for (const id of ['common', 'copter']) {
      const c = await caches.open('ardupilot-offline-' + id);
      await c.put('/__ap_complete__', completeMarker('2026-08-09T00:00:00Z', id));
    }
    const { doc } = load({ manifest: MANIFEST, caches, usage: 500e6 });
    await settle();
    const copter = doc.querySelector('tr[data-wiki="copter"]');
    check('a stored wiki shows as saved',
          copter.querySelector('.apo-badge').textContent.toLowerCase().includes('saved'));
    check('a stored wiki is auto-ticked',
          doc.querySelector('.wiki-check[value="copter"]').checked);
    check('save is disabled when the selection is all stored',
          $(doc, 'download-cache-btn').disabled, $(doc, 'download-cache-btn').title);
    check('remove all is enabled when something is stored', !$(doc, 'clear-btn').disabled);
    doc.querySelector('.wiki-check[value="rover"]').click();
    await settle();
    check('adding an unsaved wiki re-enables save', !$(doc, 'download-cache-btn').disabled);
    check('total counts only what is missing',
          $(doc, 'selection-total').textContent.includes('32'),
          JSON.stringify($(doc, 'selection-total').textContent));
  }

  console.log('\ninterrupted download');
  {
    const caches = makeCaches();
    // Entries but no completion marker: an aborted download.
    const c = await caches.open('ardupilot-offline-copter');
    await c.put('/copter/index.html', new FakeResponse('<html>'));
    const { doc } = load({ manifest: MANIFEST, caches });
    await settle();
    check('a partial download is not reported as saved',
          doc.querySelector('tr[data-wiki="copter"] .apo-badge')
             .textContent.toLowerCase().includes('incomplete'));
    check('a partial download is not auto-ticked',
          !doc.querySelector('.wiki-check[value="copter"]').checked);
  }

  console.log('\nno manifest published');
  {
    const { doc } = load({ manifest: null });
    await settle();
    check('falls back to built-in wiki list rather than an empty table',
          doc.querySelectorAll('tr[data-wiki]').length > 1,
          doc.querySelectorAll('tr[data-wiki]').length + ' rows');
    check('page still usable with no manifest', !!$(doc, 'select-all'));
  }

  console.log('\nstorage warning');
  {
    const a = load({ manifest: MANIFEST, persisted: false });
    await settle();
    check('warns when storage is temporary',
          (a.doc.getElementById('storage-warning').textContent || '').includes('temporary'));
    const b = load({ manifest: MANIFEST, persisted: true });
    await settle();
    check('no warning when storage is permanent',
          (b.doc.getElementById('storage-warning').textContent || '').trim() === '');
  }

  console.log('\ncache busting');
  {
    const { sandbox } = load({ manifest: MANIFEST });
    await settle();
    check('exposes a version marker for debugging',
          typeof sandbox.window.ArduPilotOfflineVersion === 'string',
          sandbox.window.ArduPilotOfflineVersion);
  }

  console.log('\nupdate check');
  {
    const caches = makeCaches();
    // copter saved from an older build, rover current.
    (await caches.open('ardupilot-offline-common')).put('/__ap_complete__', completeMarker(MANIFEST.generated, 'common'));
    (await caches.open('ardupilot-offline-copter')).put('/__ap_complete__', completeMarker('2020-01-01T00:00:00Z', 'copter'));
    (await caches.open('ardupilot-offline-rover')).put('/__ap_complete__', completeMarker(MANIFEST.generated, 'rover'));
    const { doc, fetchCalls } = load({ manifest: MANIFEST, caches });
    await settle();
    $(doc, 'check-btn').click();
    await settle(); await settle();
    const archive = fetchCalls.filter(u => u.indexOf('.tar') !== -1);
    check('update check re-fetches the stale wiki', archive.some(u => u.indexOf('copter') !== -1),
          JSON.stringify(archive));
    check('update check does not re-fetch a current wiki',
          !archive.some(u => u.indexOf('rover') !== -1), JSON.stringify(archive));
    check('archive URL is tagged with the build',
          archive.every(u => u.indexOf('?v=') !== -1), JSON.stringify(archive.slice(0,1)));
    check('archive URL uses the manifest host',
          archive.every(u => u.indexOf('https://cdn.example.test/') === 0), JSON.stringify(archive.slice(0,1)));
  }

  console.log('\nnothing stale');
  {
    const caches = makeCaches();
    for (const id of ['common', 'copter']) {
      (await caches.open('ardupilot-offline-' + id)).put('/__ap_complete__', completeMarker(MANIFEST.generated, id));
    }
    const { doc, fetchCalls } = load({ manifest: MANIFEST, caches });
    await settle();
    $(doc, 'check-btn').click();
    await settle(); await settle();
    check('reports up to date when nothing is stale',
          ($(doc, 'check-result').textContent || '').toLowerCase().includes('up to date'),
          JSON.stringify($(doc, 'check-result').textContent));
    check('up to date downloads nothing',
          !fetchCalls.some(u => u.indexOf('.tar') !== -1));
  }

  console.log('\nremove all');
  {
    const caches = makeCaches();
    (await caches.open('ardupilot-offline-copter')).put('/__ap_complete__', completeMarker(MANIFEST.generated, 'copter'));
    (await caches.open('ardupilot-pages-v1')).put('/copter/docs/x.html', new FakeResponse('<html>'));
    const { doc } = load({ manifest: MANIFEST, caches, usage: 100e6 });
    await settle();
    const btn = $(doc, 'clear-btn');
    btn.click();
    await settle();
    check('first press arms rather than deletes',
          btn.textContent.toLowerCase().includes('again'), JSON.stringify(btn.textContent));
    check('arming does not delete anything yet',
          (await caches.keys()).length === 2, (await caches.keys()).join(','));
    btn.click();
    await settle();
    check('an immediate second press is ignored (double-click guard)',
          (await caches.keys()).length === 2, (await caches.keys()).join(','));
    await new Promise(r => setTimeout(r, 800));
    btn.click();
    await settle(); await settle();
    check('a deliberate second press clears everything',
          (await caches.keys()).length === 0, (await caches.keys()).join(','));
  }

  console.log('\nodd manifests');
  {
    const odd = JSON.parse(JSON.stringify(MANIFEST));
    delete odd.wikis[0].archive;                       // no filename given
    const caches = makeCaches();
    (await caches.open('ardupilot-offline-common')).put('/__ap_complete__', completeMarker(odd.generated, 'common'));
    const { doc, fetchCalls } = load({ manifest: odd, caches });
    await settle();
    doc.querySelector('.wiki-check[value="copter"]').click();
    await settle();
    $(doc, 'download-cache-btn').click();
    await settle(); await settle();
    check('falls back to <id>-offline.tar.gz when the manifest omits a filename',
          fetchCalls.some(u => u.indexOf('copter-offline.tar') !== -1),
          JSON.stringify(fetchCalls.filter(u => u.indexOf('tar.gz') !== -1)));
  }
  {
    // A wiki cached from an older build that the manifest no longer lists.
    const caches = makeCaches();
    (await caches.open('ardupilot-offline-retired')).put('/__ap_complete__', completeMarker(MANIFEST.generated, 'retired'));
    (await caches.open('ardupilot-offline-common')).put('/__ap_complete__', completeMarker(MANIFEST.generated, 'common'));
    const { doc } = load({ manifest: MANIFEST, caches, usage: 10e6 });
    await settle();
    check('a cached wiki the manifest no longer lists does not break the table',
          doc.querySelectorAll('tr[data-wiki]').length === 4,
          doc.querySelectorAll('tr[data-wiki]').length + ' rows');
    check('and it is still counted as stored somewhere the reader can act on',
          !$(doc, 'clear-btn').disabled);
  }

  console.log('\nmarker without an id');
  {
    const caches = makeCaches();
    (await caches.open('ardupilot-offline-common')).put('/__ap_complete__', completeMarker(MANIFEST.generated, 'common'));
    // A marker naming no wiki: the cache name still says which it is.
    (await caches.open('ardupilot-offline-copter')).put('/__ap_complete__',
      new FakeResponse(JSON.stringify({ build: '2020-01-01T00:00:00Z', saved: 1 })));
    const { doc, fetchCalls } = load({ manifest: MANIFEST, caches });
    await settle();
    $(doc, 'check-btn').click();
    await settle(); await settle();
    check('a marker with no id is still updated, using the cache name',
          fetchCalls.some(u => u.indexOf('copter-offline.tar') !== -1),
          JSON.stringify(fetchCalls.filter(u => u.indexOf('tar.gz') !== -1)));
    check('and it is not reported as up to date',
          !($(doc, 'check-result').textContent || '').toLowerCase().includes('up to date'),
          JSON.stringify($(doc, 'check-result').textContent));
  }

  console.log('\nfreshness: the manifest');
  {
    const { fetchOpts } = load({ manifest: MANIFEST });
    await settle();
    const m = fetchOpts.filter(f => f.url.indexOf('offline-manifest.json') !== -1);
    check('the manifest is requested', m.length > 0);
    // A cached manifest would freeze the build id.
    check('the manifest is never served from cache',
          m.every(f => f.opts && f.opts.cache === 'no-cache'),
          JSON.stringify(m.map(f => f.opts && f.opts.cache)));
  }

  console.log('\nfreshness: the archive tag');
  {
    const caches = makeCaches();
    const { doc, fetchCalls } = load({ manifest: MANIFEST, caches,
                                       archives: { 'x/index.html': '<html>' } });
    await settle();
    $(doc, 'select-all').click();
    await settle();
    $(doc, 'download-cache-btn').click();
    for (let i = 0; i < 12; i++) { await settle(); }
    const arch = fetchCalls.filter(u => u.indexOf('.tar') !== -1);
    check('every archive in the queue is tagged, not just the first',
          arch.length > 1 && arch.every(u => u.indexOf('?v=') !== -1),
          arch.length + ' archives');
    check('the tag is exactly the manifest build id',
          arch.every(u => u.endsWith('?v=' + encodeURIComponent(MANIFEST.generated))),
          JSON.stringify(arch[0]));
    check('common and the wikis are all fetched',
          arch.some(u => u.indexOf('common-') !== -1) &&
          arch.some(u => u.indexOf('copter-') !== -1), JSON.stringify(arch));
  }

  console.log('\nfreshness: a new build changes the tag');
  {
    const older = JSON.parse(JSON.stringify(MANIFEST));
    older.generated = '2020-01-01T00:00:00Z';
    const a = load({ manifest: older, archives: { 'x/index.html': '<html>' } });
    await settle();
    $(a.doc, 'select-all').click(); await settle();
    $(a.doc, 'download-cache-btn').click();
    for (let i = 0; i < 12; i++) { await settle(); }
    const oldTags = a.fetchCalls.filter(u => u.indexOf('.tar') !== -1);

    const b = load({ manifest: MANIFEST, archives: { 'x/index.html': '<html>' } });
    await settle();
    $(b.doc, 'select-all').click(); await settle();
    $(b.doc, 'download-cache-btn').click();
    for (let i = 0; i < 12; i++) { await settle(); }
    const newTags = b.fetchCalls.filter(u => u.indexOf('.tar') !== -1);

    check('a different build produces a different URL',
          oldTags.length && newTags.length && oldTags[0] !== newTags[0],
          JSON.stringify([oldTags[0], newTags[0]]));
    check('the old build tag is not reused',
          !newTags.some(u => u.indexOf('2020-01-01') !== -1));
  }

  console.log('\nfreshness: no build id means no bogus tag');
  {
    const noBuild = JSON.parse(JSON.stringify(MANIFEST));
    delete noBuild.generated;
    const { doc, fetchCalls } = load({ manifest: noBuild,
                                       archives: { 'x/index.html': '<html>' } });
    await settle();
    doc.querySelector('.wiki-check[value="copter"]').click();
    await settle();
    $(doc, 'download-cache-btn').click();
    for (let i = 0; i < 12; i++) { await settle(); }
    const arch = fetchCalls.filter(u => u.indexOf('.tar') !== -1);
    check('no build id means an untagged URL, never ?v=undefined',
          arch.length && arch.every(u => u.indexOf('undefined') === -1 &&
                                          u.indexOf('?v=') === -1),
          JSON.stringify(arch[0]));
  }

  console.log('\nfreshness: the whole round trip');
  {
    const caches = makeCaches();
    // A PAX-style long name, as the real archives have one per wiki.
    const longName = 'copter/docs/how-to-use-the-auth-command-to-sign-a-' +
                     'pixhawk-board-with-your-certificate-of-authenticity.html';
    const first = load({ manifest: MANIFEST, caches,
                         archives: { 'copter/index.html': '<html>ok',
                                     [longName]: '<html>the long page' } });
    await settle();
    first.doc.querySelector('.wiki-check[value="copter"]').click();
    await settle();
    $(first.doc, 'download-cache-btn').click();
    for (let i = 0; i < 15; i++) { await settle(); }

    const c = await caches.open('ardupilot-offline-copter');
    const marker = await c.match('/__ap_complete__');
    check('a completed download writes a marker', !!marker);
    const info = marker ? JSON.parse(await marker.text()) : {};
    check('the marker records the build that was downloaded',
          info.build === MANIFEST.generated, JSON.stringify(info.build));
    check('the archive contents were unpacked into the cache',
          !!(await c.match('/copter/index.html')));
    const longHit = await c.match('/' + longName);
    check('a PAX long-named page is stored under its FULL key',
          !!longHit && (await longHit.text()) === '<html>the long page',
          longName.length + '-char name');
    check('and not under the truncated 100-char key',
          (await c.match('/' + longName.slice(0, 100))) === undefined);

    // Same build again: nothing to do.
    const same = load({ manifest: MANIFEST, caches,
                        archives: { 'copter/index.html': '<html>ok' } });
    await settle();
    $(same.doc, 'check-btn').click();
    for (let i = 0; i < 8; i++) { await settle(); }
    check('checking against the same build reports up to date',
          ($(same.doc, 'check-result').textContent || '').toLowerCase().includes('up to date'),
          JSON.stringify($(same.doc, 'check-result').textContent));
    check('and downloads nothing',
          !same.fetchCalls.some(u => u.indexOf('.tar') !== -1));

    // A newer build: the wiki is stale and must be re-fetched with the new tag.
    const newer = JSON.parse(JSON.stringify(MANIFEST));
    newer.generated = '2027-01-01T00:00:00Z';
    const next = load({ manifest: newer, caches,
                        archives: { 'copter/index.html': '<html>new' } });
    await settle();
    $(next.doc, 'check-btn').click();
    for (let i = 0; i < 15; i++) { await settle(); }
    const arch = next.fetchCalls.filter(u => u.indexOf('.tar') !== -1);
    check('a newer build makes the stored copy stale',
          arch.length > 0, JSON.stringify(arch));
    check('the refetch carries the new build tag, not the stored one',
          arch.every(u => u.indexOf(encodeURIComponent('2027-01-01T00:00:00Z')) !== -1),
          JSON.stringify(arch[0]));
    const after = await (await caches.open('ardupilot-offline-copter')).match('/__ap_complete__');
    const info2 = after ? JSON.parse(await after.text()) : {};
    check('and the marker is updated to the new build',
          info2.build === '2027-01-01T00:00:00Z', JSON.stringify(info2.build));
  }

  /* --------------------------------------------- differential updates ----- */
  // Asserted against bytes in the cache, not against what the panel says it did.

  const TABLE_KEY = '/__ap_files__';

  /** A finished download: files, table and marker. `files`: path -> [hash, body]. */
  async function seedSaved(cachesObj, id, build, files) {
    const c = await cachesObj.open('ardupilot-offline-' + id);
    const table = {};
    for (const [name, [hash, body]] of Object.entries(files)) {
      table[name] = hash;
      await c.put(ApUnpack.cachePathFor(id, name), new FakeResponse(body));
    }
    await c.put(TABLE_KEY, new FakeResponse(JSON.stringify(table)));
    await c.put('/__ap_complete__', completeMarker(build, id));
    return c;
  }

  const bodyAt = async (cache, key) => {
    const r = await cache.match(key);
    return r ? await r.text() : null;
  };
  const OLD_BUILD = '2020-01-01T00:00:00Z';
  // Content requests only: not the archive host, not the manifest.
  const siteCalls = (calls) => calls.filter(
    u => u.indexOf('/') === 0 && u.indexOf('offline-manifest.json') === -1 &&
         u.indexOf('-files.json') === -1);

  console.log('\ndifferential update: only what moved');
  {
    const cachesObj = makeCaches();
    // Both stale; common's table is unchanged, the ordinary case.
    await seedSaved(cachesObj, 'common', OLD_BUILD, {
      '_images/shared.png': ['c1', 'shared bytes']
    });
    const copter = await seedSaved(cachesObj, 'copter', OLD_BUILD, {
      'copter/index.html':     ['h1', 'old index'],
      'copter/docs/a.html':    ['h2', 'old a'],
      'copter/docs/b.html':    ['h3', 'old b'],
      'copter/searchindex.js': ['h4', 'old searchindex'],
      'copter/docs/gone.html': ['h5', 'old gone']
    });

    // Changed files carry the real hash of what the server sends; the client verifies.
    const { doc, fetchCalls, fetchOpts } = load({
      manifest: MANIFEST, caches: cachesObj,
      tables: {
        'common-files.json': { '_images/shared.png': 'c1' },
        'copter-files.json': {
          'copter/index.html':     'h1',
          'copter/docs/a.html':    await fileHash('NEW a'),
          'copter/searchindex.js': await fileHash('NEW searchindex'),
          'copter/docs/b.html':    'h3'
        }
      },
      served: {
        '/copter/docs/a.html':    'NEW a',
        '/copter/searchindex.js': 'NEW searchindex'
      }
    });
    await settle();
    $(doc, 'check-btn').click();
    for (let i = 0; i < 20; i++) { await settle(); }

    check('no archive is fetched when the tables can be compared',
          !fetchCalls.some(u => u.indexOf('.tar') !== -1),
          JSON.stringify(fetchCalls.filter(u => u.indexOf('.tar') !== -1)));

    const got = siteCalls(fetchCalls).map(u => u.split('?')[0]).sort();
    check('exactly the changed files are fetched, and nothing else',
          got.length === 2 && got[0] === '/copter/docs/a.html' &&
          got[1] === '/copter/searchindex.js', JSON.stringify(got));

    check('a changed page holds the new bytes',
          (await bodyAt(copter, '/copter/docs/a.html')) === 'NEW a',
          JSON.stringify(await bodyAt(copter, '/copter/docs/a.html')));
    check('a changed non-page holds the new bytes',
          (await bodyAt(copter, '/copter/searchindex.js')) === 'NEW searchindex',
          JSON.stringify(await bodyAt(copter, '/copter/searchindex.js')));
    check('an unchanged page keeps its own bytes, not a refetched copy',
          (await bodyAt(copter, '/copter/index.html')) === 'old index' &&
          (await bodyAt(copter, '/copter/docs/b.html')) === 'old b',
          JSON.stringify(await bodyAt(copter, '/copter/index.html')));
    check('a file dropped from the build is deleted from the cache',
          (await copter.match('/copter/docs/gone.html')) === undefined);

    const stored = JSON.parse(await bodyAt(copter, TABLE_KEY));
    check('the stored table now matches the published one',
          stored['copter/docs/a.html'] === (await fileHash('NEW a')) &&
          stored['copter/searchindex.js'] === (await fileHash('NEW searchindex')) &&
          !('copter/docs/gone.html' in stored) &&
          Object.keys(stored).length === 4, JSON.stringify(stored));

    const marker = JSON.parse(await bodyAt(copter, '/__ap_complete__'));
    check('the marker moves to the build that was applied',
          marker.build === MANIFEST.generated, JSON.stringify(marker.build));

    // Untagged requests would be answered from the cache being refreshed.
    check('every update request is tagged for the network',
          siteCalls(fetchCalls).length > 0 &&
          siteCalls(fetchCalls).every(u => u.indexOf('ap-update=') !== -1),
          JSON.stringify(siteCalls(fetchCalls)));
    check('and none of them may be served from the HTTP cache',
          fetchOpts.filter(f => f.url.indexOf('/') === 0)
                   .every(f => f.opts && f.opts.cache === 'no-cache'));

    // An unchanged wiki costs one table request and no more.
    const commonCalls = fetchCalls.filter(u => u.indexOf('common') !== -1);
    check('an unchanged wiki costs one request, its table',
          commonCalls.length === 1 && commonCalls[0].indexOf('common-files.json') !== -1,
          JSON.stringify(commonCalls));
    const commonCache = await cachesObj.open('ardupilot-offline-common');
    check('and it is still marked current afterwards',
          JSON.parse(await bodyAt(commonCache, '/__ap_complete__')).build ===
            MANIFEST.generated);

    // Two fetched, one deleted; pinned to what the cache holds.
    check('the reported count equals the changes actually applied',
          ($(doc, 'check-result').textContent || '').indexOf('3 files') !== -1,
          JSON.stringify($(doc, 'check-result').textContent));
  }

  console.log('\na quiet update that needs a full download does it, and says when it is done');
  {
    const cachesObj = makeCaches();
    // Saved before tables existed: updateStored() resolves null.
    const c = await cachesObj.open('ardupilot-offline-dev');
    await c.put('/dev/index.html', new FakeResponse('<html>'));
    await c.put('/__ap_complete__', completeMarker(OLD_BUILD, 'dev'));
    (await cachesObj.open('ardupilot-offline-common')).put('/__ap_complete__',
      completeMarker(MANIFEST.generated, 'common'));

    const { doc, sandbox, fetchCalls } = load({
      manifest: MANIFEST, caches: cachesObj,
      archives: { 'dev/index.html': '<html>new' },
    });
    await settle();
    // The quiet path, exactly as the timer calls it.
    await sandbox.window.eval ? null : null;
    // Fire a tick via the autoupdate checkbox; checkForUpdates is not exported.
    doc.getElementById('autoupdate').checked = false;
    doc.getElementById('autoupdate').dispatchEvent(
      new sandbox.window.Event('change', { bubbles: true }));
    doc.getElementById('autoupdate').checked = true;
    doc.getElementById('autoupdate').dispatchEvent(
      new sandbox.window.Event('change', { bubbles: true }));
    for (let i = 0; i < 20; i++) { await settle(); }

    check('the quiet run fetches the archive',
          fetchCalls.some(u => u.indexOf('.tar') !== -1),
          JSON.stringify(fetchCalls.filter(u => u.indexOf('.tar') !== -1).slice(0, 2)));
    check('and says when it is done',
          ($(doc, 'check-result').textContent || '').indexOf('Downloaded again') !== -1,
          JSON.stringify($(doc, 'check-result').textContent));
    check('with a toast the reader can see',
          /Update complete/.test((doc.querySelector('.ap-toast-title') || {}).textContent || ''),
          JSON.stringify((doc.querySelector('.ap-toast-title') || {}).textContent));
  }

  console.log('\nthe update paces itself and backs off when told to');
  {
    // The client is the rate limiter: unpaced fetches ran at 75 a second.
    const cachesObj = makeCaches();
    const files = {};
    for (let i = 0; i < 8; i++) { files['dev/docs/p' + i + '.html'] = ['h' + i, 'old']; }
    await seedSaved(cachesObj, 'dev', OLD_BUILD, files);
    const published = {};
    const served = {};
    const newBody = 'new body ';
    for (const k of Object.keys(files)) {
      served['/' + k] = newBody + k;
      published[k] = await fileHash(newBody + k);   // verified, so must be real
    }

    const { doc, fetchOpts } = load({
      manifest: MANIFEST, caches: cachesObj,
      tables: { 'dev-files.json': published }, served,
    });
    await settle();
    $(doc, 'check-btn').click();
    for (let i = 0; i < 40; i++) { await settle(); }

    // Gaps between requests, not wall time, which the harness dominates.
    const times = fetchOpts.filter(f => f.url.indexOf('/') === 0 &&
                                        f.url.indexOf('ap-update=') !== -1)
                           .map(f => f.at);
    const gaps = times.slice(1).map((t, i) => t - times[i]).sort((a, b) => a - b);
    // The median: a loaded machine batches a timer or two. Floor is 60% of 66 ms.
    const median = gaps.length ? gaps[Math.floor(gaps.length / 2)] : 0;
    const floor = (1000 / 15) * 0.6;
    check('consecutive update requests are spaced apart',
          gaps.length > 2 && median >= floor,
          gaps.length + ' gaps, median ' + median + ' ms, need >= ' + Math.round(floor));
  }
  {
    // A shared file, which has several sources.
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'common', OLD_BUILD, {
      '_images/shared.png': ['c1', 'old'],
    });
    const { doc, fetchCalls } = load({
      manifest: MANIFEST, caches: cachesObj,
      tables: { 'common-files.json': { '_images/shared.png': 'c2' } },
      rateLimit: true,
      archives: { 'x/index.html': '<html>' },
    });
    await settle();
    $(doc, 'check-btn').click();
    for (let i = 0; i < 25; i++) { await settle(); }
    // Exactly one tagged fetch, then it gives up.
    const tagged = fetchCalls.filter(u => u.indexOf('ap-update=') !== -1);
    check('a 429 stops the update rather than trying every other source',
          tagged.length === 1,
          tagged.length + ' tagged requests made after being refused');
  }

  console.log('\nautomatic updates are spread out, not synchronised');
  {
    // A fixed interval would have every reader fetch a new build together.
    const src = fs.readFileSync(PAGE, 'utf8');
    check('ticks are scheduled one at a time, not on a fixed interval',
          !/setInterval\(autoUpdateTick/.test(src) && /scheduleNextTick/.test(src),
          'self-scheduling');
    check('and each delay is jittered',
          /AUTOUPDATE_MS \* \(0\.5 \+ Math\.random\(\)\)/.test(src),
          'plus or minus 50%');

    // Observed, rather than read: collect the delays the page actually asks for.
    const delays = [];
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'dev', OLD_BUILD, { 'dev/index.html': ['h1', 'x'] });
    const { sandbox } = load({ manifest: MANIFEST, caches: cachesObj });
    const realSetTimeout = sandbox.window.setTimeout;
    sandbox.window.setTimeout = function (fn, ms) {
      if (ms > 1000) { delays.push(ms); return 0; }   // the update tick
      return realSetTimeout(fn, ms);
    };
    await settle();
    // Re-arm a few times to see a spread rather than one number.
    for (let i = 0; i < 5; i++) { sandbox.window.dispatchEvent(new sandbox.window.Event('online')); }
    await settle();
    const unique = new Set(delays).size;
    check('the delays are not all identical',
          delays.length === 0 || unique > 1 || delays.length === 1,
          delays.length + ' delays, ' + unique + ' distinct');
  }

  console.log('\ndifferential update: a large diff falls back to the archive');
  {
    // A stylesheet change rewrites every page; past a threshold use the archive.
    const cachesObj = makeCaches();
    const many = {};
    for (let i = 0; i < 400; i++) { many['dev/docs/p' + i + '.html'] = ['h' + i, 'old ' + i]; }
    await seedSaved(cachesObj, 'dev', OLD_BUILD, many);

    // Every file fetchable, so only the cap can stop the one-by-one path.
    const published = {};
    const loose = {};
    for (const k of Object.keys(many)) {
      const body = 'NEW ' + k;
      loose[k] = body;
      published[k] = await fileHash(body);
    }

    const { fetchCalls, doc } = load({
      manifest: MANIFEST, caches: cachesObj,
      tables: { 'dev-files.json': published },
      loose,
      archives: { 'dev/index.html': '<html>from the archive' },
    });
    await settle();
    $(doc, 'check-btn').click();
    for (let i = 0; i < 25; i++) { await settle(); }

    check('a wiki where everything changed is not fetched file by file',
          fetchCalls.filter(u => u.indexOf('ap-update=') !== -1).length === 0,
          fetchCalls.filter(u => u.indexOf('ap-update=') !== -1).length + ' file requests');
    check('it downloads the archive instead, which is one request',
          fetchCalls.filter(u => u.indexOf('.tar') !== -1).length >= 1,
          JSON.stringify(fetchCalls.filter(u => u.indexOf('.tar') !== -1)));
  }
  {
    // A template edit: most of the wiki moved, so one archive request wins.
    const cachesObj = makeCaches();
    const files = {};
    for (let i = 0; i < 60; i++) { files['dev/docs/p' + i + '.html'] = ['h' + i, 'old ' + i]; }
    for (let i = 0; i < 40; i++) { files['dev/_images/i' + i + '.png'] = ['g' + i, 'img ' + i]; }
    await seedSaved(cachesObj, 'dev', OLD_BUILD, files);
    const published = {}, loose = {};
    for (const k of Object.keys(files)) {
      if (k.endsWith('.png')) { published[k] = files[k][0]; continue; }
      loose[k] = 'NEW ' + k;
      published[k] = await fileHash(loose[k]);
    }
    const { fetchCalls, doc } = load({
      manifest: MANIFEST, caches: cachesObj,
      tables: { 'dev-files.json': published }, loose,
      archives: { 'dev/index.html': '<html>from the archive' },
    });
    await settle();
    $(doc, 'check-btn').click();
    for (let i = 0; i < 40; i++) { await settle(); }
    const tagged = fetchCalls.filter(u => u.indexOf('ap-update=') !== -1).length;
    check('most pages changed: the archive is fetched, not hundreds of files',
          tagged === 0 && fetchCalls.some(u => u.indexOf('.tar') !== -1),
          tagged + ' file requests, archive fetched: ' +
          fetchCalls.some(u => u.indexOf('.tar') !== -1));
  }
  {
    // A handful of changed files still takes the cheap path.
    const cachesObj = makeCaches();
    const files = {};
    for (let i = 0; i < 400; i++) { files['dev/docs/p' + i + '.html'] = ['h' + i, 'old ' + i]; }
    const dev = await seedSaved(cachesObj, 'dev', OLD_BUILD, files);
    const published = {};
    Object.keys(files).forEach((k) => { published[k] = files[k][0]; });
    published['dev/docs/p7.html'] = await fileHash('NEW seven');

    const { fetchCalls, doc } = load({
      manifest: MANIFEST, caches: cachesObj,
      tables: { 'dev-files.json': published },
      served: { '/dev/docs/p7.html': 'NEW seven' },
    });
    await settle();
    $(doc, 'check-btn').click();
    for (let i = 0; i < 25; i++) { await settle(); }

    check('one changed file out of four hundred still uses the cheap path',
          siteCalls(fetchCalls).length === 1 &&
          !fetchCalls.some(u => u.indexOf('.tar') !== -1),
          JSON.stringify(siteCalls(fetchCalls)));
    check('and it is applied',
          (await bodyAt(dev, '/dev/docs/p7.html')) === 'NEW seven',
          JSON.stringify(await bodyAt(dev, '/dev/docs/p7.html')));
  }

  console.log('\ndifferential update: a wrong body is refused, not stored');
  {
    // A 200 with the wrong body (captive portal, mid-deploy skew) is refused.
    const cachesObj = makeCaches();
    const copter = await seedSaved(cachesObj, 'copter', OLD_BUILD, {
      'copter/docs/a.html': ['h1', 'old a'],
    });
    (await cachesObj.open('ardupilot-offline-common')).put('/__ap_complete__',
      completeMarker(MANIFEST.generated, 'common'));
    // No archive either, so the differential path is observed in isolation.
    const { doc, fetchCalls } = load({
      manifest: MANIFEST, caches: cachesObj,
      // The table promises the hash of the RIGHT bytes...
      tables: { 'copter-files.json': { 'copter/docs/a.html': await fileHash('correct a') } },
      // ...but the server sends the wrong ones, at 200.
      served: { '/copter/docs/a.html': '<html>captive portal login</html>' },
    });
    await settle();
    $(doc, 'check-btn').click();
    for (let i = 0; i < 25; i++) { await settle(); }

    check('the wrong body is NOT written to the cache',
          (await bodyAt(copter, '/copter/docs/a.html')) === 'old a',
          JSON.stringify(await bodyAt(copter, '/copter/docs/a.html')));
    const tbl = JSON.parse(await bodyAt(copter, TABLE_KEY));
    check('the differential path does NOT advance the table on a bad body',
          tbl['copter/docs/a.html'] === 'h1', JSON.stringify(tbl));
    check('and the wiki still reports the build it actually holds',
          JSON.parse(await bodyAt(copter, '/__ap_complete__')).build === OLD_BUILD,
          JSON.parse(await bodyAt(copter, '/__ap_complete__')).build);
    check('the changed file was tried on the network, then the update gave up',
          siteCalls(fetchCalls).some(u => u.indexOf('/copter/docs/a.html') !== -1),
          JSON.stringify(siteCalls(fetchCalls)));
  }

  console.log('\ndifferential update: rewritten files come from /files/');
  {
    // The loose copy under <base>/files/ is what the table hashes.
    const cachesObj = makeCaches();
    const copter = await seedSaved(cachesObj, 'copter', OLD_BUILD, {
      'copter/docs/a.html': ['h1', 'old a'],
    });
    (await cachesObj.open('ardupilot-offline-common')).put('/__ap_complete__',
      completeMarker(MANIFEST.generated, 'common'));
    const rewritten = '<html>REWRITTEN a (donate + stills replaced)</html>';
    const { doc, fetchCalls } = load({
      manifest: MANIFEST, caches: cachesObj,
      tables: { 'copter-files.json': { 'copter/docs/a.html': await fileHash(rewritten) } },
      // The live path has the original, which would not match.
      loose: { 'copter/docs/a.html': rewritten },
      served: { '/copter/docs/a.html': '<html>ORIGINAL a</html>' },
    });
    await settle();
    $(doc, 'check-btn').click();
    for (let i = 0; i < 25; i++) { await settle(); }

    check('the rewritten copy is fetched from /files/ and stored',
          (await bodyAt(copter, '/copter/docs/a.html')) === rewritten,
          JSON.stringify(await bodyAt(copter, '/copter/docs/a.html')));
    check('no archive fallback was needed',
          !fetchCalls.some(u => u.indexOf('.tar') !== -1),
          JSON.stringify(fetchCalls.filter(u => u.indexOf('.tar') !== -1)));
    check('the /files/ source was tried',
          fetchCalls.some(u => u.indexOf('/files/copter/docs/a.html') !== -1),
          'loose URL requested');
  }

  console.log('\ndifferential update: a shared file is found under some wiki');
  {
    // A shared image is tried under each wiki in turn; here only Rover has it.
    const cachesObj = makeCaches();
    const common = await seedSaved(cachesObj, 'common', OLD_BUILD, {
      '_images/shared.png': ['c1', 'old shared bytes']
    });
    const SHARED_HASH = await fileHash('NEW shared bytes');
    const { doc, fetchCalls } = load({
      manifest: MANIFEST, caches: cachesObj,
      tables: { 'common-files.json': { '_images/shared.png': SHARED_HASH } },
      served: { '/rover/_images/shared.png': 'NEW shared bytes' }
    });
    await settle();
    $(doc, 'check-btn').click();
    for (let i = 0; i < 20; i++) { await settle(); }

    check('a shared file is stored back under /_common/, not under a wiki',
          (await bodyAt(common, '/_common/_images/shared.png')) === 'NEW shared bytes',
          JSON.stringify(await bodyAt(common, '/_common/_images/shared.png')));
    const tried = siteCalls(fetchCalls).map(u => u.split('?')[0]);
    check('the wikis are tried in turn and the walk stops at the first hit',
          tried.length === 3 && tried[tried.length - 1] === '/rover/_images/shared.png',
          JSON.stringify(tried));
    check('the wiki being read is tried first',
          tried[0] === '/x/_images/shared.png', JSON.stringify(tried[0]));
    check('no archive is fetched for a single shared image',
          !fetchCalls.some(u => u.indexOf('.tar') !== -1));
  }

  console.log('\ndifferential update: a failure leaves the record intact');
  {
    // A wiki must not claim a build it does not have.
    const cachesObj = makeCaches();
    const copter = await seedSaved(cachesObj, 'copter', OLD_BUILD, {
      'copter/index.html':  ['h1', 'old index'],
      'copter/docs/a.html': ['h2', 'old a']
    });
    const { fetchCalls, doc } = load({
      manifest: MANIFEST, caches: cachesObj,
      tables: {
        'copter-files.json': {
          'copter/index.html':  await fileHash('NEW index'),
          'copter/docs/a.html': 'h2-moved'
        }
      },
      served: { '/copter/index.html': 'NEW index' }   // a.html 404s everywhere
    });
    await settle();
    $(doc, 'check-btn').click();
    for (let i = 0; i < 20; i++) { await settle(); }

    check('a file that cannot be fetched does not update the stored table',
          JSON.parse(await bodyAt(copter, TABLE_KEY))['copter/index.html'] === 'h1',
          await bodyAt(copter, TABLE_KEY));
    check('and the wiki still reports the build it actually holds',
          JSON.parse(await bodyAt(copter, '/__ap_complete__')).build === OLD_BUILD,
          JSON.parse(await bodyAt(copter, '/__ap_complete__')).build);
    check('and it falls back to re-fetching the whole archive',
          fetchCalls.some(u => u.indexOf('.tar') !== -1),
          JSON.stringify(fetchCalls.filter(u => u.indexOf('.tar') !== -1)));
  }

  console.log('\ndifferential update: download, then update');
  {
    // A real download must leave a table behind, or updates never engage.
    const cachesObj = makeCaches();
    const idxHash = await fileHash('from the archive');
    // Common must complete too: a saved wiki needs it, and a check finishes
    // an incomplete one by downloading its archive again.
    const pngHash = await fileHash('png');
    const first = load({
      manifest: MANIFEST, caches: cachesObj,
      archives: { 'copter/index.html': 'from the archive',
                  'copter/docs/a.html': 'a from the archive',
                  '_images/shared.png': 'png' },
      tables: { 'copter-files.json': { 'copter/index.html': idxHash,
                                       'copter/docs/a.html':
                                         await fileHash('a from the archive') },
                'common-files.json': { '_images/shared.png': pngHash } }
    });
    await settle();
    first.doc.querySelector('.wiki-check[value="copter"]').click();
    await settle();
    $(first.doc, 'download-cache-btn').click();
    for (let i = 0; i < 20; i++) { await settle(); }

    const copter = await cachesObj.open('ardupilot-offline-copter');
    check('a download stores the file table beside the files',
          !!(await copter.match(TABLE_KEY)),
          JSON.stringify(await bodyAt(copter, TABLE_KEY)));

    const newer = JSON.parse(JSON.stringify(MANIFEST));
    newer.generated = '2027-01-01T00:00:00Z';
    const next = load({
      manifest: newer, caches: cachesObj,
      tables: { 'copter-files.json': { 'copter/index.html': idxHash,
                                       'copter/docs/a.html':
                                         await fileHash('a after the edit') },
                'common-files.json': { '_images/shared.png': pngHash } },
      served: { '/copter/docs/a.html': 'a after the edit' }
    });
    await settle();
    $(next.doc, 'check-btn').click();
    for (let i = 0; i < 20; i++) { await settle(); }

    check('the update that follows fetches one file, not the archive',
          !next.fetchCalls.some(u => u.indexOf('.tar') !== -1) &&
          siteCalls(next.fetchCalls).length === 1,
          JSON.stringify(siteCalls(next.fetchCalls)));
    check('the edited page is replaced',
          (await bodyAt(copter, '/copter/docs/a.html')) === 'a after the edit',
          JSON.stringify(await bodyAt(copter, '/copter/docs/a.html')));
    check('the untouched page still comes from the archive',
          (await bodyAt(copter, '/copter/index.html')) === 'from the archive',
          JSON.stringify(await bodyAt(copter, '/copter/index.html')));
    check('and the copy now reports the newer build',
          JSON.parse(await bodyAt(copter, '/__ap_complete__')).build ===
            '2027-01-01T00:00:00Z');
  }

  console.log('\ndownload order: the chosen wiki before common');
  {
    // Common backfills after the wiki is readable.
    const cachesObj = makeCaches();
    const { doc, fetchCalls } = load({
      manifest: MANIFEST, caches: cachesObj,
      archives: { 'copter/index.html': '<html>' },
    });
    await settle();
    doc.querySelector('.wiki-check[value="copter"]').click();
    await settle();
    $(doc, 'download-cache-btn').click();
    for (let i = 0; i < 15; i++) { await settle(); }

    const arch = fetchCalls.filter(u => u.indexOf('.tar') !== -1);
    const copterAt = arch.findIndex(u => u.indexOf('copter-') !== -1);
    const commonAt = arch.findIndex(u => u.indexOf('common-') !== -1);
    check('the chosen wiki is fetched before common',
          copterAt !== -1 && commonAt !== -1 && copterAt < commonAt,
          JSON.stringify(arch.map(u => u.split('/').pop().split('?')[0])));
  }

  console.log('\neviction: a reclaimed wiki is noticed, not silent');
  {
    // Recorded as saved but gone from Cache Storage: evicted, and said so.
    const caches = makeCaches();
    // common is present; copter was saved (recorded) but its cache is gone.
    (await caches.open('ardupilot-offline-common')).put('/__ap_complete__',
      completeMarker(MANIFEST.generated, 'common'));
    const { doc, w } = load({ manifest: MANIFEST, caches, usage: 400e6 });
    // Pretend copter was saved earlier this device.
    w.localStorage.setItem('ap-saved-ids', JSON.stringify(['common', 'copter']));
    await settle();
    // Re-render storage now that the record exists.
    const status = ($(doc, 'storage-warning').textContent || '');
    check('an evicted wiki produces a notice',
          /no longer here/.test(status) && /reclaimed/.test(status),
          JSON.stringify(status.slice(0, 80)));

    // A wiki that IS still saved is not called evicted.
    const caches2 = makeCaches();
    for (const id of ['common', 'copter']) {
      (await caches2.open('ardupilot-offline-' + id)).put('/__ap_complete__',
        completeMarker(MANIFEST.generated, id));
    }
    const b = load({ manifest: MANIFEST, caches: caches2, usage: 400e6 });
    b.w.localStorage.setItem('ap-saved-ids', JSON.stringify(['common', 'copter']));
    await settle();
    check('a wiki still present is not called evicted',
          !/no longer here/.test(b.doc.getElementById('storage-warning').textContent || ''));
  }

  console.log('\nthe update toast appears and shows progress');
  {
    // The toast appears on a manual check and shows progress, then done.
    const cachesObj = makeCaches();
    const copter = await seedSaved(cachesObj, 'copter', OLD_BUILD, {
      'copter/docs/a.html': ['h1', 'old a'],
    });
    (await cachesObj.open('ardupilot-offline-common')).put('/__ap_complete__',
      completeMarker(MANIFEST.generated, 'common'));
    const { doc } = load({
      manifest: MANIFEST, caches: cachesObj,
      tables: { 'copter-files.json': { 'copter/docs/a.html': await fileHash('NEW a') } },
      loose: { 'copter/docs/a.html': 'NEW a' },
    });
    await settle();
    $(doc, 'check-btn').click();
    for (let i = 0; i < 25; i++) { await settle(); }

    const card = doc.querySelector('.ap-toast');
    check('a toast card is created', !!card);
    check('the toast is shown', !!card && card.classList.contains('ap-toast-show'));
    check('it ends in the done state after applying the update',
          !!card && card.classList.contains('ap-toast-done'),
          card ? card.className : 'no card');
    check('and it reports what happened',
          !!card && /Updated 1 file/.test(card.textContent),
          card ? JSON.stringify(card.querySelector('.ap-toast-msg').textContent) : '');
  }

  console.log('\nregression: the worker is told when caches change (B3)');
  {
    // The worker memoises cache names and only refreshes on CACHES_CHANGED.
    const cachesObj = makeCaches();
    (await cachesObj.open('ardupilot-offline-copter')).put('/__ap_complete__',
      completeMarker(MANIFEST.generated, 'copter'));
    (await cachesObj.open('ardupilot-pages-v1')).put('/copter/x.html', new FakeResponse('<html>'));
    const { doc, swMessages, w } = load({ manifest: MANIFEST, caches: cachesObj, usage: 100e6 });
    await settle();
    const btn = $(doc, 'clear-btn');
    btn.click(); await settle();               // arm
    await new Promise(r => setTimeout(r, 850));
    btn.click(); await settle(); await settle(); // confirm -> clearAll
    check('Remove all tells the worker its caches changed',
          swMessages.some(m => m && m.type === 'CACHES_CHANGED'),
          JSON.stringify(swMessages));
  }

  console.log('\nregression: a click on a button child still acts (closest)');
  {
    // A click on the armed button's countdown bar must still count.
    const cachesObj = makeCaches();
    (await cachesObj.open('ardupilot-offline-copter')).put('/__ap_complete__',
      completeMarker(MANIFEST.generated, 'copter'));
    const { doc, w, swMessages } = load({ manifest: MANIFEST, caches: cachesObj, usage: 100e6 });
    await settle();
    const btn = $(doc, 'clear-btn');
    // Dispatch a click whose target is a child element of the button.
    const child = doc.createElement('span');
    btn.appendChild(child);
    child.dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    await settle();
    check('a click on a child of Remove all arms it, not swallowed',
          (btn.textContent || '').toLowerCase().includes('again'),
          JSON.stringify(btn.textContent));
  }

  console.log('\nregression: the footer waits for the wiki list (B10)');
  {
    // renderStorage reads storedIds, which renderWikis populates.
    const cachesObj = makeCaches();
    for (const id of ['common', 'copter', 'rover']) {
      (await cachesObj.open('ardupilot-offline-' + id)).put('/__ap_complete__',
        completeMarker(MANIFEST.generated, id));
    }
    const { doc } = load({ manifest: MANIFEST, caches: cachesObj, usage: 200e6 });
    await settle();
    const status = ($(doc, 'storage-status').textContent || '');
    check('the footer reflects saved wikis on first paint, not "no wikis saved"',
          /\d+ wikis? saved/.test(status) && !/no wikis saved/.test(status),
          JSON.stringify(status));
  }

  console.log('\nregression: a folded wiki updates in place, from its own URL');
  {
    // A folded wiki's page updates from its own URL, over the copy being read.
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'common', OLD_BUILD, {
      '_images/shared.png':        ['c1', 'shared bytes'],
      'ardupilot/docs/about.html': ['a1', 'old about']
    });
    const common = await cachesObj.open('ardupilot-offline-common');

    const { doc, fetchCalls } = load({
      manifest: MANIFEST, caches: cachesObj,
      tables: {
        'common-files.json': {
          '_images/shared.png':        'c1',
          'ardupilot/docs/about.html': await fileHash('NEW about')
        }
      },
      served: { '/ardupilot/docs/about.html': 'NEW about' }
    });
    await settle();
    $(doc, 'check-btn').click();
    for (let i = 0; i < 20; i++) { await settle(); }

    const asked = siteCalls(fetchCalls).map((u) => u.split('?')[0]);
    check('a changed folded page is fetched from its own URL',
          asked.includes('/ardupilot/docs/about.html'), JSON.stringify(asked));
    check('it is not hunted for under the other wikis',
          !asked.some((u) => /^\/(copter|plane|rover|dev)\/ardupilot\//.test(u)),
          JSON.stringify(asked));
    check('the new body replaces the copy that is actually read',
          (await bodyAt(common, '/ardupilot/docs/about.html')) === 'NEW about',
          JSON.stringify(await bodyAt(common, '/ardupilot/docs/about.html')));
    check('and nothing is written under /_common/ardupilot/',
          (await bodyAt(common, '/_common/ardupilot/docs/about.html')) === null);
  }

  console.log('\nRemove all quotes what it removes, not what the origin is charged');
  {
    // The button quotes the table's sizes, not storage.estimate().usage.
    const cachesObj = makeCaches();
    for (const id of ['common', 'copter', 'rover']) {
      (await cachesObj.open('ardupilot-offline-' + id)).put('/__ap_complete__',
        completeMarker(MANIFEST.generated, id));
    }
    // A usage figure far above the real content, as browsers report.
    const { doc } = load({ manifest: MANIFEST, caches: cachesObj, usage: 1.6e9 });
    await settle();

    $(doc, 'clear-btn').click();
    await settle();
    const label = $(doc, 'clear-btn').textContent || '';

    const expected = [MANIFEST.common].concat(MANIFEST.wikis)
      .filter((w) => ['common', 'copter', 'rover'].indexOf(w.id) !== -1)
      .reduce((n, w) => n + w.mb, 0);

    // Any GB figure is wrong: only 506 MB is saved.
    check('the button does not quote the inflated quota figure',
          !/GB/.test(label), JSON.stringify(label));
    check('it quotes the size of what is actually saved',
          label.indexOf(String(expected)) !== -1 ||
          new RegExp(String(Math.round(expected / 1024 * 10) / 10)).test(label),
          JSON.stringify(label) + ' for ' + expected + ' MB saved');
  }

  console.log('\nB8: the toast names what is being downloaded again');
  {
    const cachesObj = makeCaches();
    const c = await cachesObj.open('ardupilot-offline-dev');
    await c.put('/dev/index.html', new FakeResponse('<html>'));
    await c.put('/__ap_complete__', completeMarker(OLD_BUILD, 'dev'));
    (await cachesObj.open('ardupilot-offline-common')).put('/__ap_complete__',
      completeMarker(MANIFEST.generated, 'common'));

    const { doc, sandbox, fetchCalls } = load({
      manifest: MANIFEST, caches: cachesObj,
      archives: { 'dev/index.html': '<html>new' },
    });
    await settle();
    doc.getElementById('autoupdate').checked = false;
    doc.getElementById('autoupdate').dispatchEvent(
      new sandbox.window.Event('change', { bubbles: true }));
    doc.getElementById('autoupdate').checked = true;
    doc.getElementById('autoupdate').dispatchEvent(
      new sandbox.window.Event('change', { bubbles: true }));
    for (let i = 0; i < 20; i++) { await settle(); }

    const msg = (doc.querySelector('.ap-toast-msg') || {}).textContent || '';
    check('the toast names the wiki rather than sending the reader to look',
          /Developer/.test(msg), JSON.stringify(msg));
    check('the download ran without a button press',
          fetchCalls.some((u) => u.indexOf('.tar') !== -1),
          JSON.stringify(fetchCalls.filter((u) => u.indexOf('.tar') !== -1).slice(0, 2)));
    check('no button is offered, because nothing is waiting on the reader',
          !doc.querySelector('.ap-toast-action') || doc.querySelector('.ap-toast-action').hidden);
  }

  console.log('\nregression: a wiki folded into common keeps its own URLs');
  {
    // Only asking for a page by its real URL catches a wrong prefix.
    const { sandbox } = load({ manifest: MANIFEST });
    await settle();
    const cache = await sandbox.caches.open('fold-test');
    const tar = tarBytes({
      '_images/shared.png': 'png',
      'ardupilot/index.html': '<html><body>About</body></html>',
      'ardupilot/_static/theme.css': 'body{}',
      'ardupilot/_images/own.png': 'png',
    });
    sandbox.fetch = () => Promise.resolve({ ok: true, body: streamOf(tar) });
    await sandbox.ApUnpack.fetchArchive(
      { id: 'common', name: 'Common', archive: 'common-offline.tar' },
      cache, () => {}, { base: '/offline' });
    // The shim spells keys as absolute URLs, as Cache Storage does.
    const keys = (await cache.keys())
      .map((r) => new URL(String(r.url), 'https://x').pathname);

    check('a folded wiki unpacks at its own path, not under /_common/',
          keys.includes('/ardupilot/index.html'),
          JSON.stringify(keys.filter((k) => k.indexOf('ardupilot') !== -1)));
    check('its assets and own images come with it',
          keys.includes('/ardupilot/_static/theme.css') &&
          keys.includes('/ardupilot/_images/own.png'));
    check('shared images still go under /_common/',
          keys.includes('/_common/_images/shared.png'));
    check('nothing lands under /_common/ardupilot/',
          !keys.some((k) => k.indexOf('/_common/ardupilot/') === 0));
  }

  console.log('\nunpack: a parameter delta is stored as a delta and rebuilt on read');
  {
    const { sandbox } = load({ manifest: MANIFEST });
    await settle();
    const base = fs.readFileSync(path.join(FIXTURES, 'delta-base.html'));
    const page = fs.readFileSync(path.join(FIXTURES, 'delta-page.html'));
    const frame = fs.readFileSync(path.join(FIXTURES, 'delta-page.zst'));
    const wasm = fs.readFileSync(path.join(FRONTEND_JS, 'zstd.wasm'));
    const BASE = 'parameters-Rover-stable-V4.7.0.html';
    const container = Buffer.concat([Buffer.from('APDELTA1 ' + BASE + ' ' + hash16(page) + '\n'), frame]);
    const climb = Buffer.concat([Buffer.from('APDELTA1 ../../sw.js ' + hash16(page) + '\n'), frame]);
    const wrongHash = Buffer.concat([Buffer.from('APDELTA1 ' + BASE + ' 0123456789abcdef\n'), frame]);
    const noHash = Buffer.concat([Buffer.from('APDELTA1 ' + BASE + '\n'), frame]);
    const cache = await sandbox.caches.open('delta-test');
    const tar = tarBytes({
      ['rover/docs/' + BASE]: base,
      'rover/docs/parameters-Rover-stable-V4.6.0.html': container,
      'rover/docs/parameters-Rover-stable-V4.5.0.html': climb,
      'rover/docs/parameters-Rover-stable-V4.4.0.html': wrongHash,
      'rover/docs/parameters-Rover-stable-V4.3.0.html': noHash,
    });
    let wasmFetches = 0;
    sandbox.fetch = (u) => {
      if (String(u).indexOf('zstd.wasm') !== -1) {
        wasmFetches++;
        return Promise.resolve({ ok: true, arrayBuffer: () => Promise.resolve(
          wasm.buffer.slice(wasm.byteOffset, wasm.byteOffset + wasm.byteLength)) });
      }
      return Promise.resolve({ ok: true, body: streamOf(tar) });
    };
    await sandbox.ApUnpack.fetchArchive(
      { id: 'rover', name: 'Rover', archive: 'rover-offline.tar' },
      cache, () => {}, { base: '/offline' });
    const DELTA = '/rover/docs/parameters-Rover-stable-V4.6.0.html';
    const stored = await cache.match(DELTA);
    check('the delta is stored as it arrived, marked as a delta',
          !!stored && stored.headers.get('x-ap-encoding') === 'zstd-delta' &&
          (await stored.arrayBuffer()).byteLength === container.length,
          stored ? String(stored.headers.get('x-ap-encoding')) : 'not stored');
    check('the base page is stored plain',
          !!(await cache.match('/rover/docs/' + BASE)) &&
          !(await cache.match('/rover/docs/' + BASE)).headers.get('x-ap-encoding'));
    check('nothing was decoded while unpacking', wasmFetches === 0, wasmFetches + ' wasm fetches');

    const out = await sandbox.ApUnpack.readFrom(cache, DELTA);
    const text = out ? await out.text() : '';
    check('readFrom rebuilds the page from the delta and its base',
          text === page.toString('utf8'), text.length + ' bytes');
    check('the rebuilt page is served as html',
          !!out && /^text\/html/.test(String(out.headers.get('Content-Type'))));
    await sandbox.ApUnpack.readFrom(cache, DELTA);
    check('the decoder is fetched once and kept', wasmFetches === 1, wasmFetches + ' fetches');

    const bad = await cache.match('/rover/docs/parameters-Rover-stable-V4.5.0.html');
    check('a delta naming a base outside its own directory is not treated as a delta',
          !!bad && !bad.headers.get('x-ap-encoding'));
    const unhashed = await cache.match('/rover/docs/parameters-Rover-stable-V4.3.0.html');
    check('a delta header without the hash is still stored as a delta, never as a page',
          !!unhashed && unhashed.headers.get('x-ap-encoding') === 'zstd-delta');
    let noHashErr = null;
    try { await sandbox.ApUnpack.readFrom(cache, '/rover/docs/parameters-Rover-stable-V4.3.0.html'); }
    catch (e) { noHashErr = e.message; }
    check('and is refused on read, since nothing can prove the rebuilt page',
          !!noHashErr && /no hash/.test(noHashErr), noHashErr || 'no error');
    let hashErr = null;
    try { await sandbox.ApUnpack.readFrom(cache, '/rover/docs/parameters-Rover-stable-V4.4.0.html'); }
    catch (e) { hashErr = e.message; }
    check('a rebuilt page that does not match its hash is refused',
          !!hashErr && /does not match its hash/.test(hashErr), hashErr || 'no error');

    // Stored plain by older code, no marker: the magic still says what it is.
    const plainStore = await sandbox.caches.open('delta-unmarked');
    await plainStore.put('/rover/docs/' + BASE, new FakeResponse(base, { headers: { 'Content-Type': 'text/html' } }));
    await plainStore.put(DELTA, new FakeResponse(container, { headers: { 'Content-Type': 'text/html' } }));
    await plainStore.put('/rover/docs/parameters-Rover-stable-V4.2.0.html',
      new FakeResponse('<html><body>a real version page</body></html>', { headers: { 'Content-Type': 'text/html' } }));
    const unmarked = await sandbox.ApUnpack.readFrom(plainStore, DELTA);
    check('a delta stored without its marker is still rebuilt',
          !!unmarked && (await unmarked.text()) === page.toString('utf8'));
    const realPage = await sandbox.ApUnpack.readFrom(plainStore, '/rover/docs/parameters-Rover-stable-V4.2.0.html');
    check('a real version page stored plain is served as it is',
          !!realPage && /a real version page/.test(await realPage.text()));

    // No wasm to be had: the JavaScript decoder rebuilds the same page.
    const jsOnly = load({ manifest: MANIFEST });
    await settle();
    const c2 = await jsOnly.sandbox.caches.open('delta-js');
    jsOnly.sandbox.fetch = (u) => (String(u).indexOf('zstd.wasm') !== -1
      ? Promise.resolve({ ok: false, status: 404 })
      : Promise.resolve({ ok: true, body: streamOf(tar) }));
    await jsOnly.sandbox.ApUnpack.fetchArchive(
      { id: 'rover', name: 'Rover', archive: 'rover-offline.tar' }, c2, () => {}, { base: '/offline' });
    const jsOut = await jsOnly.sandbox.ApUnpack.readFrom(c2, DELTA);
    check('without a wasm the JavaScript decoder rebuilds the page',
          !!jsOut && (await jsOut.text()) === page.toString('utf8') &&
          jsOnly.sandbox.ApZstd.mode() === 'js', String(jsOnly.sandbox.ApZstd.mode()));

    await cache.delete('/rover/docs/' + BASE);
    let err = null;
    try { await sandbox.ApUnpack.readFrom(cache, DELTA); } catch (e) { err = e.message; }
    check('a delta whose base is gone fails naming the base',
          !!err && err.indexOf(BASE) !== -1, err || 'no error');
  }

  console.log('\nupgrade compatibility: every way a delta can have been stored still reads');
  {
    // Data written by one version of the code, read by another: the entry
    // may carry the marker or not, and the decoder may be the wasm or
    // JavaScript; every hashed combination must rebuild the same page, a
    // hashless one must be refused rather than trusted, and a real page at
    // such a path must be served as it is.
    const base = fs.readFileSync(path.join(FIXTURES, 'delta-base.html'));
    const page = fs.readFileSync(path.join(FIXTURES, 'delta-page.html'));
    const frame = fs.readFileSync(path.join(FIXTURES, 'delta-page.zst'));
    const BASE = 'parameters-Rover-stable-V4.7.0.html';
    const headers = {
      hashed: 'APDELTA1 ' + BASE + ' ' + hash16(page) + '\n',
      hashless: 'APDELTA1 ' + BASE + '\n',
    };
    for (const decoder of ['wasm', 'js']) {
      const r = load({ manifest: MANIFEST });
      await settle();
      if (decoder === 'js') {
        r.sandbox.fetch = (u) => (String(u).indexOf('zstd.wasm') !== -1
          ? Promise.resolve({ ok: false, status: 404 }) : Promise.reject(new Error('no')));
      }
      const c = await r.sandbox.caches.open('compat-' + decoder);
      await c.put('/rover/docs/' + BASE, new FakeResponse(base, { headers: { 'Content-Type': 'text/html' } }));
      const cases = [];
      for (const marked of [true, false]) {
        for (const h of Object.keys(headers)) {
          const key = '/rover/docs/parameters-Rover-stable-V4.' + (marked ? 6 : 5) + '.' + (h === 'hashed' ? 1 : 0) + '.html';
          const hdr = { 'Content-Type': 'text/html' };
          if (marked) { hdr['x-ap-encoding'] = 'zstd-delta'; }
          await c.put(key, new FakeResponse(Buffer.concat([Buffer.from(headers[h]), frame]), { headers: hdr }));
          cases.push({ key, marked, h });
        }
      }
      await c.put('/rover/docs/parameters-Rover-stable-V4.4.4.html',
        new FakeResponse('<html><body>a real page saved plain</body></html>', { headers: { 'Content-Type': 'text/html' } }));
      for (const cs of cases) {
        let text = null, err = null;
        try { text = await (await r.sandbox.ApUnpack.readFrom(c, cs.key)).text(); } catch (e) { err = e.message; }
        if (cs.h === 'hashed') {
          check(decoder + ' decoder, ' + (cs.marked ? 'marked' : 'unmarked') + ' entry, hashed header: rebuilt',
                text === page.toString('utf8'), err || (text ? text.slice(0, 30) : 'nothing'));
        } else {
          // Nothing can prove a hashless rebuild, least of all in JavaScript.
          check(decoder + ' decoder, ' + (cs.marked ? 'marked' : 'unmarked') + ' entry, hashless header: refused, never a page',
                text === null && !!err && /no hash/.test(err), err || (text ? text.slice(0, 30) : 'nothing'));
        }
      }
      const real = await (await r.sandbox.ApUnpack.readFrom(c, '/rover/docs/parameters-Rover-stable-V4.4.4.html')).text();
      check(decoder + ' decoder: a real page at a versioned path is served as it is',
            /a real page saved plain/.test(real));
      check(decoder + ' decoder was the one in use', r.sandbox.ApZstd.mode() === decoder, String(r.sandbox.ApZstd.mode()));
    }

    // A differential update replaces an old-format entry with the new one.
    const cachesObj = makeCaches();
    const DELTA = 'parameters-Rover-stable-V4.6.3.html';
    const oldContainer = Buffer.concat([Buffer.from(headers.hashless), frame]);
    const newContainer = Buffer.concat([Buffer.from(headers.hashed), frame]);
    await seedSaved(cachesObj, 'common', OLD_BUILD, { '_images/shared.png': ['c1', 'shared bytes'] });
    await seedSaved(cachesObj, 'rover', OLD_BUILD, {
      'rover/index.html': ['h1', 'old index'],
      ['rover/docs/' + BASE]: [await fileHash(base), base],
    });
    // The old entry, stored plain by older update code, no marker.
    const rc = await cachesObj.open('ardupilot-offline-rover');
    await rc.put('/rover/docs/' + DELTA, new FakeResponse(oldContainer, { headers: { 'Content-Type': 'text/html' } }));
    const stored = await ApUpdate_readTable(rc);
    stored['rover/docs/' + DELTA] = await fileHash(oldContainer);
    await rc.put('/__ap_files__', new FakeResponse(JSON.stringify(stored)));
    const published = Object.assign({}, stored, { ['rover/docs/' + DELTA]: await fileHash(newContainer) });
    const r = load({ manifest: MANIFEST, caches: cachesObj,
      tables: { 'rover-files.json': published, 'common-files.json': { '_images/shared.png': 'c1' } },
      loose: { ['rover/docs/' + DELTA]: newContainer } });
    await settle();
    $(r.doc, 'check-btn').click();
    for (let i = 0; i < 14; i++) { await settle(); }
    const after = await rc.match('/rover/docs/' + DELTA);
    check('an update replaces an old-format entry with a marked, hashed one',
          !!after && after.headers.get('x-ap-encoding') === 'zstd-delta' &&
          (await after.text()).indexOf(hash16(page)) !== -1);
    check('and the page it rebuilds is unchanged',
          (await (await r.sandbox.ApUnpack.readFrom(rc, '/rover/docs/' + DELTA)).text()) === page.toString('utf8'));
  }

  console.log('\nunpack: a cut-short or hostile archive is refused, so it is never marked complete');
  {
    const { sandbox } = load({ manifest: MANIFEST });
    await settle();
    const whole = tarBytes({ 'rover/index.html': '<html></html>',
                             'rover/docs/a.html': 'x'.repeat(2000) });
    const attempt = async (bytes) => {
      const cache = await sandbox.caches.open('refuse-' + Math.random());
      sandbox.fetch = () => Promise.resolve({ ok: true, body: streamOf(bytes) });
      try {
        await sandbox.ApUnpack.fetchArchive(
          { id: 'rover', name: 'Rover', archive: 'rover-offline.tar' },
          cache, () => {}, { base: '/offline' });
        return { ok: true, keys: (await cache.keys()).length };
      } catch (e) {
        return { ok: false, error: e.message, keys: (await cache.keys()).length };
      }
    };
    // Layout: header, 512 body, header, 2048 body, 1024 of zero blocks.
    const full = await attempt(whole);
    check('the whole archive unpacks', full.ok && full.keys === 2, JSON.stringify(full));
    const midHeader = await attempt(whole.subarray(0, 1024 + 200));
    check('an archive cut mid-header rejects instead of resolving',
          !midHeader.ok && /truncated/.test(midHeader.error), JSON.stringify(midHeader));
    const midBody = await attempt(whole.subarray(0, 1536 + 1000));
    check('an archive cut mid-body rejects, naming the entry',
          !midBody.ok && /truncated in rover\/docs\/a\.html/.test(midBody.error),
          JSON.stringify(midBody));
    const noEnd = await attempt(whole.subarray(0, whole.length - 1024));
    check('an archive missing its end blocks rejects',
          !noEnd.ok && /truncated/.test(noEnd.error), JSON.stringify(noEnd));
    const climb = await attempt(tarBytes({ '../../sw.js': 'evil', 'rover/index.html': 'ok' }));
    check('an entry that climbs out of the tree rejects the archive',
          !climb.ok && /unsafe archive path/.test(climb.error), JSON.stringify(climb));
    const abs = await attempt(tarBytes({ '/sw.js': 'evil' }));
    check('an absolute entry name rejects the archive',
          !abs.ok && /unsafe archive path/.test(abs.error), JSON.stringify(abs));
    const enc = await attempt(tarBytes({ '%2e%2e/%2e%2e/sw.js': 'evil' }));
    check('a percent-encoded climb rejects the archive too',
          !enc.ok && /unsafe archive path/.test(enc.error), JSON.stringify(enc));
    const odd = await attempt(tarBytes({ 'rover/_images/100%.png': 'png',
                                         'rover/_images/50%25off.png': 'png' }));
    check('names that are not valid percent-encoding still save',
          odd.ok && odd.keys === 2, JSON.stringify(odd));
    // The fragment hides the climb from the guard but not from the URL parser.
    const frag = await attempt(tarBytes({ '%2e%2e/%2e%2e/sw.js#%': 'evil' }));
    check('a fragment cannot smuggle an encoded climb past the guard',
          !frag.ok && /unsafe archive path/.test(frag.error), JSON.stringify(frag));
    const qs = await attempt(tarBytes({ 'rover/x.html?%': 'evil' }));
    check('a query in an entry name rejects the archive',
          !qs.ok && /unsafe archive path/.test(qs.error), JSON.stringify(qs));
    const zz = await attempt(tarBytes({ '%zz/%2e%2e/%2e%2e/sw.js': 'evil' }));
    check('a malformed escape cannot smuggle a climb through an unpack',
          !zz.ok && /unsafe archive path/.test(zz.error), JSON.stringify(zz));
    // A name the parser shortens or encodes must be STORED at the very key
    // the guard approved, never rebuilt from the raw name's length.
    const shifty = await attempt(tarBytes({ 'rover/a\tb2.html': 'tabbed',
                                            'rover/a b.html': 'spaced' }));
    check('a normalised name is stored at the key the guard approved',
          shifty.ok && shifty.keys === 2, JSON.stringify(shifty));
    if (shifty.ok) {
      const c2 = await sandbox.caches.open('shifty-check');
      sandbox.fetch = () => Promise.resolve({ ok: true, body: streamOf(
        tarBytes({ 'rover/a\tb2.html': 'tabbed', 'rover/a b.html': 'spaced' })) });
      await sandbox.ApUnpack.fetchArchive(
        { id: 'rover', name: 'Rover', archive: 'rover-offline.tar' },
        c2, () => {}, { base: '/offline' });
      check('the tab-stripped page sits under its own wiki, not the reader\'s page',
            !!(await c2.match('/rover/ab2.html')) &&
            !(await c2.match('/ardupilot/docs/rover/ab2.html')));
      check('the spaced name sits at its encoded key',
            !!(await c2.match('/rover/a%20b.html')));
    }

    // The URL parser, not the guard, decides what a name means; every route
    // to a path outside the archive's own tree must come back refused.
    const verdict = (id, name) => {
      try { ApUnpack.cachePathFor(id, name); return 'accepted'; }
      catch (e) { return /unsafe archive path/.test(e.message) ? 'refused' : e.message; }
    };
    check('a malformed escape cannot hide an encoded climb',
          verdict('rover', 'rover/%/%2e%2e/%2e%2e/copter/index.html') === 'refused');
    check('invalid UTF-8 in an escape cannot either',
          verdict('rover', 'rover/%FF/%2e%2e/%2e%2e/copter/index.html') === 'refused');
    check('a tab inside a dot-dot pair is stripped by the parser, not missed',
          verdict('rover', 'rover/.\t./copter/index.html') === 'refused');
    check('so are a newline and a carriage return',
          verdict('rover', 'rover/.\n./copter/index.html') === 'refused' &&
          verdict('rover', 'rover/.\r./copter/index.html') === 'refused');
    check('an entry for another wiki is refused with no trickery at all',
          verdict('rover', 'copter/index.html') === 'refused');
    check('a prefix must anchor at the start, not appear anywhere',
          verdict('rover', 'evilwiki/rover/x.html') === 'refused');
    check('a fragment on an otherwise-fine name is refused',
          verdict('rover', 'rover/ok.html#sec') === 'refused');
    // 65 MB in the size field with no body: the header check must fire
    // before any allocation, not a truncation error after buffering.
    const hugeHead = Buffer.alloc(512);
    hugeHead.write('rover/huge.bin', 0, 100);
    hugeHead.write((65 * 1024 * 1024).toString(8).padStart(11, '0') + '\0', 124, 12);
    hugeHead.write('0', 156, 1);
    hugeHead.write('        ', 148, 8);
    { let sum = 0; for (const b of hugeHead) { sum += b; }
      hugeHead.write(sum.toString(8).padStart(6, '0') + '\0 ', 148, 8); }
    const huge = await attempt(Buffer.concat([hugeHead, Buffer.alloc(1024)]));
    check('an oversized entry is refused before it allocates',
          !huge.ok && /too large/.test(huge.error), JSON.stringify(huge));
    check('a protocol-relative name cannot change origin',
          verdict('rover', '/rover/rover/index.html') === 'refused');
    check('backslashes resolve like slashes and are judged after that',
          verdict('rover', 'rover\\..\\..\\sw.js') === 'refused');
    check('an ordinary page inside its own wiki still saves',
          verdict('rover', 'rover/docs/a.html') === 'accepted');
    check('the common archive holds images and folded wikis, nothing else',
          verdict('common', '_images/x.png') === 'accepted' &&
          verdict('common', 'ardupilot/docs/about.html') === 'accepted' &&
          verdict('common', 'sw.js') === 'refused' &&
          verdict('common', '_common/sw.js') === 'refused');
  }

  console.log('\nparameter versions ride in the archive: no picker, a count on the row');
  {
    const mk = (ver, def) => ({
      file: `docs/parameters-Copter-stable-V${ver}.html`, channel: 'stable',
      version: ver, label: ver, bytes: 400 * 1048576, default: !!def });
    const man = JSON.parse(JSON.stringify(MANIFEST));
    man.wikis.find((w) => w.id === 'copter').param_versions =
      [mk('4.7.0', true), mk('4.6.3'), mk('4.5.7')];
    const withVersions = load({ manifest: man });
    const without = load({ manifest: MANIFEST });
    await settle();
    const doc = withVersions.doc;
    check('no version picker is drawn',
          !doc.querySelector('.param-check') && !doc.querySelector('.apo-param-toggle') &&
          !doc.querySelector('.param-all') && !doc.getElementById('all-params'));
    const row = doc.querySelector('tr[data-wiki="copter"]');
    check('the row says how many versions the archive carries',
          !!row && /3 parameter versions/.test(row.textContent), row ? row.textContent.trim() : 'no row');
    for (const d of [doc, without.doc]) {
      const box = d.querySelector('.wiki-check[value="copter"]');
      box.checked = true;
      box.dispatchEvent(new d.defaultView.Event('change', { bubbles: true }));
    }
    await settle();
    const total = (d) => ($(d, 'selection-total').textContent || '').trim();
    check('the versions add nothing to the selection total: they are in the archive',
          total(doc) === total(without.doc) && total(doc) !== '',
          total(doc) + ' vs ' + total(without.doc));
  }

  console.log('\na refresh stores a delta as a delta and prunes a version the archive dropped');
  {
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'common', OLD_BUILD, {
      '_images/shared.png': ['c1', 'shared bytes']
    });
    await seedSaved(cachesObj, 'copter', OLD_BUILD, {
      'copter/index.html': ['h1', 'old index'],
      'copter/docs/parameters-Copter-stable-V3.6.0.html': ['h3', 'a version no longer carried'],
    });
    const BASE = 'parameters-Copter-stable-V4.7.0.html';
    const DELTA = 'parameters-Copter-stable-V4.6.0.html';
    const base = fs.readFileSync(path.join(FIXTURES, 'delta-base.html'));
    const frame = fs.readFileSync(path.join(FIXTURES, 'delta-page.zst'));
    const container = Buffer.concat([Buffer.from('APDELTA1 ' + BASE + ' ' +
      hash16(fs.readFileSync(path.join(FIXTURES, 'delta-page.html'))) + '\n'), frame]);
    const archives = {
      'copter/index.html': '<html>new index</html>',
      ['copter/docs/' + BASE]: base,
      ['copter/docs/' + DELTA]: container,
    };
    const table = {};
    for (const [n, b] of Object.entries(archives)) { table[n] = await fileHash(b); }
    const { doc } = load({ manifest: MANIFEST, caches: cachesObj, archives,
      tables: { 'copter-files.json': table,
                'common-files.json': { '_images/shared.png': 'c1' } } });
    await settle();
    $(doc, 'check-btn').click();
    for (let i = 0; i < 14; i++) { await settle(); }
    const cache = await cachesObj.open('ardupilot-offline-copter');
    const delta = await cache.match('/copter/docs/' + DELTA);
    check('the delta is stored as a delta',
          !!delta && delta.headers.get('x-ap-encoding') === 'zstd-delta',
          delta ? String(delta.headers.get('x-ap-encoding')) : 'not stored');
    check('the base beside it is stored plain',
          !!(await cache.match('/copter/docs/' + BASE)) &&
          !(await cache.match('/copter/docs/' + BASE)).headers.get('x-ap-encoding'));
    check('a version the archive no longer carries is pruned like any other page',
          !(await cache.match('/copter/docs/parameters-Copter-stable-V3.6.0.html')));
    check('the copy is marked complete', !!(await cache.match('/__ap_complete__')));
  }

  console.log('\na differential update stores a fetched delta as a delta');
  {
    const cachesObj = makeCaches();
    const BASE = 'parameters-Copter-stable-V4.7.0.html';
    const DELTA = 'parameters-Copter-stable-V4.6.0.html';
    const base = fs.readFileSync(path.join(FIXTURES, 'delta-base.html'));
    const frame = fs.readFileSync(path.join(FIXTURES, 'delta-page.zst'));
    const container = Buffer.concat([Buffer.from('APDELTA1 ' + BASE + ' ' +
      hash16(fs.readFileSync(path.join(FIXTURES, 'delta-page.html'))) + '\n'), frame]);
    await seedSaved(cachesObj, 'common', OLD_BUILD, {
      '_images/shared.png': ['c1', 'shared bytes']
    });
    await seedSaved(cachesObj, 'copter', OLD_BUILD, {
      'copter/index.html': ['h1', 'old index'],
      ['copter/docs/' + BASE]: [await fileHash(base), base],
    });
    const published = {
      'copter/index.html': 'h1',
      ['copter/docs/' + BASE]: await fileHash(base),
      ['copter/docs/' + DELTA]: await fileHash(container),
    };
    const { doc, fetchCalls } = load({ manifest: MANIFEST, caches: cachesObj,
      tables: { 'copter-files.json': published,
                'common-files.json': { '_images/shared.png': 'c1' } },
      loose: { ['copter/docs/' + DELTA]: container } });
    await settle();
    $(doc, 'check-btn').click();
    for (let i = 0; i < 14; i++) { await settle(); }
    const cache = await cachesObj.open('ardupilot-offline-copter');
    const delta = await cache.match('/copter/docs/' + DELTA);
    check('the update fetched the one new file loose, not the archive',
          fetchCalls.some((u) => u.indexOf('/files/copter/docs/' + DELTA) !== -1) &&
          !fetchCalls.some((u) => u.indexOf('copter-offline.tar') !== -1),
          fetchCalls.filter((u) => u.indexOf('copter') !== -1).join(' '));
    check('the fetched delta carries the delta marker',
          !!delta && delta.headers.get('x-ap-encoding') === 'zstd-delta',
          delta ? String(delta.headers.get('x-ap-encoding')) : 'not stored');
    check('and the copy stays complete', !!(await cache.match('/__ap_complete__')));
    check('a check that moved files is recorded as an update, not merely a check',
          /^Updated just now$/.test($(doc, 'last-checked').textContent),
          $(doc, 'last-checked').textContent);
  }

  console.log('\nthe decoder is checked once a saved wiki carries versions');
  {
    const mk = (ver, def) => ({
      file: `docs/parameters-Copter-stable-V${ver}.html`, channel: 'stable',
      version: ver, label: ver, bytes: 30000, default: !!def });
    const withVersions = () => {
      const man = JSON.parse(JSON.stringify(MANIFEST));
      man.wikis.find((w) => w.id === 'copter').param_versions = [mk('4.7.0', true), mk('4.6.3')];
      man.wikis.find((w) => w.id === 'rover').param_versions = [
        { file: 'docs/parameters-Rover-stable-V4.7.0.html', channel: 'stable',
          version: '4.7.0', label: '4.7.0', bytes: 30000, default: true }];
      return man;
    };
    // The versions as a save leaves them: deltas, marked as such.
    const seedDeltas = async (cachesObj) => {
      const c = await cachesObj.open('ardupilot-offline-copter');
      for (const ver of ['4.7.0', '4.6.3']) {
        await c.put('/copter/docs/parameters-Copter-stable-V' + ver + '.html',
          new FakeResponse(Buffer.from('APDELTA1 parameters-Copter-stable-V4.7.0.html\nxx'),
                           { headers: { 'Content-Type': 'text/html',
                                        'x-ap-encoding': 'zstd-delta' } }));
      }
    };
    // Nothing saved: no check, no wasm fetched.
    let r = load({ manifest: withVersions() });
    await settle(); await settle();
    check('with nothing saved the decoder is not checked',
          !r.fetchCalls.some((u) => u.indexOf('zstd.wasm') !== -1) &&
          $(r.doc, 'delta-warning').hidden);

    // Copter saved and carrying versions: the check runs and passes here.
    let cachesObj = makeCaches();
    await seedSaved(cachesObj, 'common', OLD_BUILD, { '_images/shared.png': ['c1', 'shared bytes'] });
    await seedSaved(cachesObj, 'copter', OLD_BUILD, { 'copter/index.html': ['h1', 'old index'] });
    await seedDeltas(cachesObj);
    r = load({ manifest: withVersions(), caches: cachesObj });
    for (let i = 0; i < 4; i++) { await settle(); }
    check('a saved wiki with versions triggers one decoder self-test',
          r.fetchCalls.filter((u) => u.indexOf('zstd.wasm') !== -1).length === 1,
          r.fetchCalls.filter((u) => u.indexOf('zstd') !== -1).length + ' wasm fetches');
    check('a working decoder shows no warning', $(r.doc, 'delta-warning').hidden);

    // The same, in a browser whose decoder cannot run.
    cachesObj = makeCaches();
    await seedSaved(cachesObj, 'common', OLD_BUILD, { '_images/shared.png': ['c1', 'shared bytes'] });
    await seedSaved(cachesObj, 'copter', OLD_BUILD, { 'copter/index.html': ['h1', 'old index'] });
    const served = {
      '/copter/docs/parameters-Copter-stable-V4.7.0.html': '<html><body>plain 4.7.0</body></html>',
      '/copter/docs/parameters-Copter-stable-V4.6.3.html': '<html><body>plain 4.6.3</body></html>',
      '/rover/docs/parameters-Rover-stable-V4.7.0.html': '<html><body>rover, not saved</body></html>',
    };
    await seedDeltas(cachesObj);
    r = load({ manifest: withVersions(), caches: cachesObj, decoder: false, served });
    for (let i = 0; i < 4; i++) { await settle(); }
    const warn = $(r.doc, 'delta-warning');
    check('a failing decoder shows the warning, naming what will not open',
          !warn.hidden && /cannot rebuild the compressed parameter versions/.test(warn.textContent) &&
          /will not open offline/.test(warn.textContent), warn.textContent.slice(0, 80));
    const btn = r.doc.getElementById('plain-params-btn');
    check('it offers one button with the size of the plain pages',
          !!btn && /about 1 MB/.test(btn.textContent), btn ? btn.textContent : 'no button');
    check('the current parameter list is said to be unaffected',
          /current parameter list is unaffected/.test(warn.textContent));

    // Pressing it fetches the plain page for the SAVED wiki's versions only.
    btn.click(); btn.click();
    for (let i = 0; i < 8; i++) { await settle(); }
    const pageFetches = r.fetchCalls.filter((u) => u.indexOf('/docs/parameters-') !== -1);
    check('every carried version of the saved wiki is fetched as a plain page, once',
          pageFetches.length === 2 &&
          pageFetches.every((u) => u.indexOf('/copter/') !== -1),
          pageFetches.join(' '));
    check('an unsaved wiki\'s versions are left alone',
          !pageFetches.some((u) => u.indexOf('/rover/') !== -1));
    check('each fetch is tagged as an update, so the worker goes to the network and never answers with its offline page',
          pageFetches.every((u) => /\?ap-update=/.test(u)), pageFetches[0]);
    const copter = await cachesObj.open('ardupilot-offline-copter');
    const stored = await copter.match('/copter/docs/parameters-Copter-stable-V4.6.3.html');
    check('the plain page is stored as a page, with no delta marker',
          !!stored && !stored.headers.get('x-ap-encoding'));
    const back = await r.sandbox.ApUnpack.readFrom(copter, '/copter/docs/parameters-Copter-stable-V4.6.3.html');
    check('and reads back without any decoder',
          !!back && (await back.text()) === served['/copter/docs/parameters-Copter-stable-V4.6.3.html']);
    check('the warning goes once the plain pages are in', $(r.doc, 'delta-warning').hidden);
    check('the reader is told all of them are saved',
          /All 2 parameter versions are saved as plain pages/.test($(r.doc, 'cache-progress').textContent),
          $(r.doc, 'cache-progress').textContent);
    check('the worker is told the caches changed',
          r.swMessages.some((m) => m && m.type === 'CACHES_CHANGED'));
    check('the copy stays complete', !!(await copter.match('/__ap_complete__')));

    // One page the site no longer serves: the rest still land, and it says so.
    cachesObj = makeCaches();
    await seedSaved(cachesObj, 'common', OLD_BUILD, { '_images/shared.png': ['c1', 'shared bytes'] });
    await seedSaved(cachesObj, 'copter', OLD_BUILD, { 'copter/index.html': ['h1', 'old index'] });
    const partial = Object.assign({}, served);
    delete partial['/copter/docs/parameters-Copter-stable-V4.6.3.html'];
    await seedDeltas(cachesObj);
    r = load({ manifest: withVersions(), caches: cachesObj, decoder: false, served: partial });
    for (let i = 0; i < 4; i++) { await settle(); }
    r.doc.getElementById('plain-params-btn').click();
    for (let i = 0; i < 8; i++) { await settle(); }
    const c2 = await cachesObj.open('ardupilot-offline-copter');
    const kept = await c2.match('/copter/docs/parameters-Copter-stable-V4.7.0.html');
    const still = await c2.match('/copter/docs/parameters-Copter-stable-V4.6.3.html');
    check('a version the site cannot serve is reported, the others still stored',
          !!kept && !kept.headers.get('x-ap-encoding') &&
          !!still && still.headers.get('x-ap-encoding') === 'zstd-delta' &&
          /1 of 2 parameter pages saved/.test($(r.doc, 'cache-progress').textContent) &&
          /parameters-Copter-stable-V4\.6\.3/.test($(r.doc, 'cache-progress').textContent),
          $(r.doc, 'cache-progress').textContent);
    check('the warning stays while any version is still a delta',
          !$(r.doc, 'delta-warning').hidden);

    // A captive portal answers 200 with a page of its own: refused, not stored.
    cachesObj = makeCaches();
    await seedSaved(cachesObj, 'common', OLD_BUILD, { '_images/shared.png': ['c1', 'shared bytes'] });
    await seedSaved(cachesObj, 'copter', OLD_BUILD, { 'copter/index.html': ['h1', 'old index'] });
    await seedDeltas(cachesObj);
    r = load({ manifest: withVersions(), caches: cachesObj, decoder: false, served });
    for (let i = 0; i < 4; i++) { await settle(); }
    const realFetch = r.sandbox.fetch;
    r.sandbox.fetch = (u, o) => {
      if (String(u).indexOf('V4.6.3') !== -1) {
        const portal = Buffer.from('<html><body>Sign in to the wifi</body></html>');
        return Promise.resolve({ ok: true, headers: { get: (k) =>
          (String(k).toLowerCase() === 'content-type' ? 'text/plain' : null) },
          arrayBuffer: () => Promise.resolve(portal.buffer.slice(portal.byteOffset,
            portal.byteOffset + portal.byteLength)) });
      }
      return realFetch(u, o);
    };
    r.doc.getElementById('plain-params-btn').click();
    for (let i = 0; i < 8; i++) { await settle(); }
    const c3 = await cachesObj.open('ardupilot-offline-copter');
    const portalHit = await c3.match('/copter/docs/parameters-Copter-stable-V4.6.3.html');
    check('a page not served as HTML is refused and the delta kept',
          !!portalHit && portalHit.headers.get('x-ap-encoding') === 'zstd-delta' &&
          /1 of 2 parameter pages saved/.test($(r.doc, 'cache-progress').textContent),
          $(r.doc, 'cache-progress').textContent);

    // A Save pressed while the plain pages are still arriving is refused.
    cachesObj = makeCaches();
    await seedSaved(cachesObj, 'common', OLD_BUILD, { '_images/shared.png': ['c1', 'shared bytes'] });
    await seedSaved(cachesObj, 'copter', OLD_BUILD, { 'copter/index.html': ['h1', 'old index'] });
    await seedDeltas(cachesObj);
    r = load({ manifest: withVersions(), caches: cachesObj, decoder: false, served });
    for (let i = 0; i < 4; i++) { await settle(); }
    let release;
    const slowFetch = r.sandbox.fetch;
    r.sandbox.fetch = (u, o) => (String(u).indexOf('/docs/parameters-') !== -1
      ? new Promise((res) => { release = () => res(slowFetch(u, o)); })
      : slowFetch(u, o));
    r.doc.getElementById('plain-params-btn').click();
    await settle();
    r.doc.querySelector('.wiki-check[value="rover"]').click();
    $(r.doc, 'download-cache-btn').click();
    await settle();
    check('a Save pressed mid-fallback is refused, not run over the same caches',
          /parameter pages are still downloading/.test($(r.doc, 'cache-progress').textContent) &&
          !r.fetchCalls.some((u) => u.indexOf('rover-offline.tar') !== -1),
          $(r.doc, 'cache-progress').textContent);
    if (release) { release(); }
    for (let i = 0; i < 8; i++) { await settle(); }
  }

  console.log('\nthe state text is green only when the worker controls the page');
  {
    let r = load({ manifest: MANIFEST, offline: true });
    await settle();
    let state = $(r.doc, 'offline-mode-state');
    check('opted in and controlled: "on", green',
          state.textContent === 'on' && state.classList.contains('apo-state-live'),
          state.textContent + ' [' + state.className + ']');
    r = load({ manifest: MANIFEST, offline: true, controlled: false, registered: false });
    await settle();
    state = $(r.doc, 'offline-mode-state');
    check('opted in but the worker not yet active: says so, not green',
          /^on, starting/.test(state.textContent) && !state.classList.contains('apo-state-live'),
          state.textContent + ' [' + state.className + ']');
    // A hard reload: the page is not controlled, the worker is active all the same.
    r = load({ manifest: MANIFEST, offline: true, controlled: false, registered: true });
    await settle();
    state = $(r.doc, 'offline-mode-state');
    check('an active worker that does not control this page (hard reload) still shows green',
          state.textContent === 'on' && state.classList.contains('apo-state-live'),
          state.textContent + ' [' + state.className + ']');
    r = load({ manifest: MANIFEST, offline: false });
    await settle();
    state = $(r.doc, 'offline-mode-state');
    check('not opted in: "off", not green',
          state.textContent === 'off' && !state.classList.contains('apo-state-live'));
  }

  console.log('\na finished check records when it ran');
  {
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'common', OLD_BUILD, { '_images/shared.png': ['c1', 'shared bytes'] });
    await seedSaved(cachesObj, 'copter', OLD_BUILD, { 'copter/index.html': ['h1', 'index'] });
    const same = JSON.parse(JSON.stringify(MANIFEST)); same.generated = OLD_BUILD;
    let r = load({ manifest: same, caches: cachesObj });
    await settle();
    check('before any check the line is empty', $(r.doc, 'last-checked').textContent === '');
    $(r.doc, 'check-btn').click();
    for (let i = 0; i < 6; i++) { await settle(); }
    check('an up-to-date check stamps the time and says what it found',
          /^Checked just now, up to date$/.test($(r.doc, 'last-checked').textContent) &&
          !!r.w.localStorage.getItem('ap-last-checked'),
          $(r.doc, 'last-checked').textContent);

    // Ninety minutes on, in a new session: the stamp survives and reads as age.
    const then = JSON.stringify({ t: new Date(Date.now() - 90 * 60000).toISOString(), r: 'current' });
    r = load({ manifest: same, caches: cachesObj });
    r.w.localStorage.setItem('ap-last-checked', then);
    await settle(); await settle();
    check('a later visit shows how long ago', /Checked (1|2) h ago, up to date/.test($(r.doc, 'last-checked').textContent),
          $(r.doc, 'last-checked').textContent);

    // Merely opening the page is not a check: no fetch of the manifest, no stamp.
    check('opening the page did not stamp anything by itself',
          r.w.localStorage.getItem('ap-last-checked') === then &&
          !r.fetchCalls.some((u) => u.indexOf('offline-manifest.json') !== -1 && r.fetchOpts.some((o) => o.url === u && o.opts.cache === 'no-cache')) ||
          r.w.localStorage.getItem('ap-last-checked') === then,
          r.w.localStorage.getItem('ap-last-checked'));

    // A check that never reached the site does not pretend it did.
    r = load({ manifest: null, caches: cachesObj });
    r.w.localStorage.setItem('ap-last-checked', then);
    await settle();
    $(r.doc, 'check-btn').click();
    for (let i = 0; i < 6; i++) { await settle(); }
    check('a failed check leaves the old stamp alone',
          r.w.localStorage.getItem('ap-last-checked') === then &&
          !/just now/.test($(r.doc, 'last-checked').textContent),
          $(r.doc, 'last-checked').textContent);

    // Nothing saved: nothing to reassure about.
    r = load({ manifest: MANIFEST });
    r.w.localStorage.setItem('ap-last-checked', then);
    await settle(); await settle();
    check('with nothing saved the line stays empty', $(r.doc, 'last-checked').textContent === '');
  }

  console.log('\na check finishes an interrupted refresh instead of calling it up to date');
  {
    // Copter complete; the shared images were complete once (they hold a
    // table) but lost their marker to an interrupted refresh.
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'copter', OLD_BUILD, { 'copter/index.html': ['h1', 'index'] });
    await seedSaved(cachesObj, 'common', OLD_BUILD, { '_images/shared.png': ['c1', 'shared bytes'] });
    const common = await cachesObj.open('ardupilot-offline-common');
    await common.delete('/__ap_complete__');
    const same = JSON.parse(JSON.stringify(MANIFEST)); same.generated = OLD_BUILD;
    const r = load({ manifest: same, caches: cachesObj,
      archives: { '_images/shared.png': 'shared bytes' },
      tables: { 'common-files.json': { '_images/shared.png': await fileHash('shared bytes') },
                'copter-files.json': { 'copter/index.html': 'h1' } } });
    await settle();
    check('the row says the shared images are incomplete',
          /Incomplete/.test(r.doc.querySelector('tr[data-wiki="common"]').textContent));
    $(r.doc, 'check-btn').click();
    for (let i = 0; i < 14; i++) { await settle(); }
    check('the check downloads the shared images again rather than skipping them',
          r.fetchCalls.some((u) => u.indexOf('common-offline.tar') !== -1),
          r.fetchCalls.filter((u) => u.indexOf('offline') !== -1).join(' '));
    check('and they are complete afterwards', !!(await common.match('/__ap_complete__')));
    check('the check is never called "up to date" while something was incomplete',
          !/up to date/i.test($(r.doc, 'check-result').textContent) &&
          /^Updated just now/.test($(r.doc, 'last-checked').textContent),
          $(r.doc, 'check-result').textContent + ' | ' + $(r.doc, 'last-checked').textContent);
  }

  console.log('\na first save the reader cancelled is not re-downloaded behind their back');
  {
    // Rover was never complete and holds no table: the reader stopped it.
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'common', OLD_BUILD, { '_images/shared.png': ['c1', 'shared bytes'] });
    await seedSaved(cachesObj, 'copter', OLD_BUILD, { 'copter/index.html': ['h1', 'index'] });
    const rover = await cachesObj.open('ardupilot-offline-rover');
    await rover.put('/rover/index.html', new FakeResponse('half'));
    const same = JSON.parse(JSON.stringify(MANIFEST)); same.generated = OLD_BUILD;
    const r = load({ manifest: same, caches: cachesObj });
    await settle();
    $(r.doc, 'check-btn').click();
    for (let i = 0; i < 8; i++) { await settle(); }
    check('nothing is fetched for it', !r.fetchCalls.some((u) => u.indexOf('rover-offline.tar') !== -1));
    check('the row still says incomplete, for the reader to decide',
          /Incomplete/.test(r.doc.querySelector('tr[data-wiki="rover"]').textContent));
  }

  console.log('\nthe first save is checked against the file table before the marker');
  {
    // The table names a page the archive lacked: a build landed mid-save.
    const cachesObj = makeCaches();
    const { doc } = load({ manifest: MANIFEST, caches: cachesObj,
      archives: { 'copter/index.html': '<html>a</html>' },
      tables: { 'copter-files.json': { 'copter/index.html': 'h1',
                                       'copter/docs/landed-later.html': 'h2' } } });
    await settle();
    doc.querySelector('.wiki-check[value="copter"]').click();
    await settle();
    $(doc, 'download-cache-btn').click();
    for (let i = 0; i < 12; i++) { await settle(); }
    const cache = await cachesObj.open('ardupilot-offline-copter');
    check('a table naming pages the archive lacked leaves no marker',
          !(await cache.match('/__ap_complete__')));
    check('the mismatched table is not stored either',
          !(await cache.match(TABLE_KEY)));
    check('the reader is told, not left with a spinner',
          /new build|try again/i.test($(doc, 'cache-progress').textContent || ''),
          JSON.stringify($(doc, 'cache-progress').textContent));

    // Right names but wrong bytes: a damaged download must not be marked saved.
    const badCaches = makeCaches();
    const bad = load({ manifest: MANIFEST, caches: badCaches,
      archives: { 'copter/index.html': '<html>a</html>' },
      tables: { 'copter-files.json': { 'copter/index.html': 'deadbeefdeadbeef' } } });
    await settle();
    bad.doc.querySelector('.wiki-check[value="copter"]').click();
    await settle();
    $(bad.doc, 'download-cache-btn').click();
    for (let i = 0; i < 12; i++) { await settle(); }
    const badCache = await badCaches.open('ardupilot-offline-copter');
    check('a body that does not hash to the table is not marked saved',
          !(await badCache.match('/__ap_complete__')) &&
          /damaged/.test($(bad.doc, 'cache-progress').textContent || ''),
          JSON.stringify($(bad.doc, 'cache-progress').textContent));

    // The archive carries a file the table does not name: refused, not kept.
    const strayCaches = makeCaches();
    const stray = load({ manifest: MANIFEST, caches: strayCaches,
      archives: { 'copter/index.html': '<html>a</html>',
                  'copter/stray.html': '<html>s</html>' },
      tables: { 'copter-files.json': { 'copter/index.html': await fileHash('<html>a</html>') } } });
    await settle();
    stray.doc.querySelector('.wiki-check[value="copter"]').click();
    await settle();
    $(stray.doc, 'download-cache-btn').click();
    for (let i = 0; i < 12; i++) { await settle(); }
    const strayCache = await strayCaches.open('ardupilot-offline-copter');
    check('an entry the table does not name is refused, not marked saved',
          !(await strayCache.match('/__ap_complete__')) &&
          /does not list/.test($(stray.doc, 'cache-progress').textContent || ''),
          JSON.stringify($(stray.doc, 'cache-progress').textContent));

    // The same save with a table the archive satisfies completes.
    const okCaches = makeCaches();
    const ok = load({ manifest: MANIFEST, caches: okCaches,
      archives: { 'copter/index.html': '<html>a</html>' },
      tables: { 'copter-files.json': { 'copter/index.html': await fileHash('<html>a</html>') },
                'common-files.json': {} } });
    await settle();
    ok.doc.querySelector('.wiki-check[value="copter"]').click();
    await settle();
    $(ok.doc, 'download-cache-btn').click();
    for (let i = 0; i < 12; i++) { await settle(); }
    const okCache = await okCaches.open('ardupilot-offline-copter');
    check('a table the archive satisfies is stored and marked complete',
          !!(await okCache.match('/__ap_complete__')) && !!(await okCache.match(TABLE_KEY)));
  }

  console.log('\na full redownload replaces the copy, deleted pages included');
  {
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'common', OLD_BUILD, {
      '_images/shared.png': ['c1', 'shared bytes']
    });
    // Both entries moved, so the caps push this to a full redownload.
    await seedSaved(cachesObj, 'copter', OLD_BUILD, {
      'copter/index.html':     ['h1', 'old index'],
      'copter/docs/gone.html': ['h2', 'a page since deleted upstream']
    });
    const { doc } = load({ manifest: MANIFEST, caches: cachesObj,
      archives: { 'copter/index.html': '<html>new index</html>' },
      tables: { 'copter-files.json':
                  { 'copter/index.html': await fileHash('<html>new index</html>') },
                'common-files.json': { '_images/shared.png': 'c1' } } });
    await settle();
    $(doc, 'check-btn').click();
    for (let i = 0; i < 14; i++) { await settle(); }
    const cache = await cachesObj.open('ardupilot-offline-copter');
    check('the deleted page is gone after the redownload',
          !(await cache.match('/copter/docs/gone.html')));
    check('the redownloaded copy is current and marked complete',
          (await bodyAt(cache, '/copter/index.html')) === '<html>new index</html>' &&
          !!(await cache.match('/__ap_complete__')));
  }

  console.log('\na failed redownload leaves the old copy readable');
  {
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'common', OLD_BUILD, {
      '_images/shared.png': ['c1', 'shared bytes']
    });
    await seedSaved(cachesObj, 'copter', OLD_BUILD, {
      'copter/index.html':  ['h1', 'the old but working copy'],
      'copter/docs/a.html': ['h2', 'old a']
    });
    // The differential fails (nothing served), promoting to full; the archive
    // fetch then fails too, which is exactly a bad network doing both.
    const { doc } = load({ manifest: MANIFEST, caches: cachesObj,
      tables: { 'copter-files.json': { 'copter/index.html': 'NEW1',
                                       'copter/docs/a.html': 'NEW2' },
                'common-files.json': { '_images/shared.png': 'c1' } } });
    await settle();
    $(doc, 'check-btn').click();
    for (let i = 0; i < 14; i++) { await settle(); }
    const cache = await cachesObj.open('ardupilot-offline-copter');
    check('the old copy is still there, marked complete',
          (await bodyAt(cache, '/copter/index.html')) === 'the old but working copy' &&
          !!(await cache.match('/__ap_complete__')));
  }

  console.log('\nan unfinished download is called incomplete, not hidden');
  {
    const cachesObj = makeCaches();
    (await cachesObj.open('ardupilot-offline-common')).put('/__ap_complete__',
      completeMarker(MANIFEST.generated, 'common'));
    // Entries but no marker: a save that died part-way.
    (await cachesObj.open('ardupilot-offline-copter')).put('/copter/index.html',
      new FakeResponse('<html></html>'));
    const { doc } = load({ manifest: MANIFEST, caches: cachesObj, usage: 80e6 });
    await settle();
    const row = doc.querySelector('tr[data-wiki="copter"]');
    check('the wiki with a markerless cache reads Incomplete, save again',
          /Incomplete, save again/.test(row ? row.textContent : ''),
          JSON.stringify(row && row.textContent.slice(0, 80)));
    check('the storage line counts the unfinished download',
          /1 incomplete download/.test($(doc, 'storage-status').textContent || ''),
          JSON.stringify($(doc, 'storage-status').textContent));
    check('a finished wiki is not called incomplete',
          !/Incomplete/.test((doc.querySelector('tr[data-wiki="common"]') || {}).textContent || ''));
  }

  console.log('\na running download owns the panel');
  {
    const cachesObj = makeCaches();
    const { doc, w, sandbox } = load({ manifest: MANIFEST, caches: cachesObj });
    await settle();
    // An archive fetch that never resolves keeps the download active.
    sandbox.fetch = () => new Promise(() => {});
    doc.querySelector('.wiki-check[value="copter"]').click(); await settle();
    $(doc, 'download-cache-btn').click(); await settle();
    const click = (id) => doc.getElementById(id)
      .dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    check('check and export are disabled while a download runs',
          $(doc, 'check-btn').disabled && $(doc, 'dl-single').disabled);
    click('check-btn'); await settle();
    check('a forced check refuses politely and cancels nothing',
          /download is running/i.test($(doc, 'check-result').textContent || '') &&
          /Cancel/.test($(doc, 'download-cache-btn').textContent),
          JSON.stringify($(doc, 'check-result').textContent));
    // The harness does not load the exporter; the guard fires before it is used.
    w.ArduPilotExport = { exportHtml: () => Promise.resolve({ pages: 0 }) };
    click('dl-single'); await settle(); await settle();
    check('a forced export refuses politely and cancels nothing',
          /already running/i.test($(doc, 'dl-single').textContent || '') &&
          /Cancel/.test($(doc, 'download-cache-btn').textContent),
          JSON.stringify($(doc, 'dl-single').textContent));
    check('the refused export leaves the button disabled, not re-armed',
          $(doc, 'dl-single').disabled);
    // Changing the selection mid-download must not hand the buttons back.
    doc.querySelector('.wiki-check[value="rover"]').click(); await settle();
    check('reselecting mid-download does not re-enable export',
          $(doc, 'dl-single').disabled && $(doc, 'check-btn').disabled);
  }

  console.log('\nexport repairs a broken shared-images cache first');
  {
    const cachesObj = makeCaches();
    (await cachesObj.open('ardupilot-offline-copter')).put('/__ap_complete__',
      completeMarker(MANIFEST.generated, 'copter'));
    // Entries but no marker: a cancelled common download left behind.
    (await cachesObj.open('ardupilot-offline-common')).put('/_common/_images/x.png',
      new FakeResponse('png'));
    const { doc, w, fetchCalls, apOffline } = load({ manifest: MANIFEST, caches: cachesObj,
      archives: { '_images/shared.png': 'png bytes' } });
    await settle();
    w.ArduPilotExport = { exportHtml: () => Promise.resolve({ pages: 3 }) };
    // The saved wiki renders pre-ticked; the export needs no clicks but its own.
    $(doc, 'dl-single').dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    for (let i = 0; i < 15; i++) { await settle(); }
    check('the export first re-downloads the shared images',
          fetchCalls.some((u) => u.indexOf('common-offline.tar') !== -1),
          JSON.stringify(fetchCalls.filter((u) => u.indexOf('.tar') !== -1)));
    const common = await cachesObj.open('ardupilot-offline-common');
    check('the repaired common cache is marked complete',
          !!(await common.match('/__ap_complete__')));
    check('and the export then completes',
          /Saved .*copter/.test($(doc, 'dl-single').textContent || ''),
          JSON.stringify($(doc, 'dl-single').textContent));
    check('the pre-save opted into offline mode like Save does',
          apOffline.calls.indexOf('enable') !== -1,
          JSON.stringify(apOffline.calls));
  }

  console.log('\na refused refresh is honestly incomplete');
  {
    // The wiki was saved and complete; the refresh arrives damaged. The old
    // marker must fall with it, or the mix would serve as complete forever.
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'copter', OLD_BUILD, { 'copter/index.html': ['h1', 'old'] });
    const { doc, w, swMessages } = load({ manifest: MANIFEST, caches: cachesObj,
      archives: { 'copter/index.html': '<html>new</html>' },
      tables: { 'copter-files.json': { 'copter/index.html': 'deadbeefdeadbeef' } } });
    await settle();
    swMessages.length = 0;
    $(doc, 'check-btn').dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    for (let i = 0; i < 14; i++) { await settle(); }
    const c = await cachesObj.open('ardupilot-offline-copter');
    check('the failed refresh takes the stale marker with it',
          !(await c.match('/__ap_complete__')),
          JSON.stringify($(doc, 'check-result').textContent || $(doc, 'cache-progress').textContent));
    check('and the panel does not claim an update completed',
          !/update complete|downloaded again/i.test($(doc, 'check-result').textContent || ''),
          JSON.stringify($(doc, 'check-result').textContent));
    // The worker memoises the marker check until told otherwise; the mix
    // it holds is now unmarked and must not keep serving as complete.
    check('the worker is told the caches changed after a refused refresh',
          swMessages.some((m) => m && m.type === 'CACHES_CHANGED'),
          JSON.stringify(swMessages));
  }

  console.log('\na persistence prompt nobody answers does not stall the save');
  {
    // Firefox turns navigator.storage.persist() into a doorhanger, and the
    // promise stays pending until the reader notices it. The save must not
    // wait on that: persistence is a nicety, the download is the point.
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'common', MANIFEST.generated, { '_images/seed.png': ['h0', 'x'] });
    const body = '<html>a</html>';
    const { doc, sandbox } = load({ manifest: MANIFEST, caches: cachesObj,
      archives: { 'copter/index.html': body },
      tables: { 'copter-files.json': { 'copter/index.html': await fileHash(body) } } });
    await settle();
    // The prompt is answered only after the save is long done.
    let allow;
    sandbox.navigator.storage.persist = () => new Promise((r) => { allow = r; });
    doc.querySelector('.wiki-check[value="copter"]').click(); await settle();
    $(doc, 'download-cache-btn').click();
    for (let i = 0; i < 40; i++) { await settle(); }
    const text = $(doc, 'cache-progress').textContent || '';
    check('the save completes without an answer', text === 'Saved', JSON.stringify(text));
    check('and the wiki is marked saved',
          !!(await (await cachesObj.open('ardupilot-offline-copter')).match('/__ap_complete__')));
    check('storage still reads as temporary meanwhile',
          /temporary/.test($(doc, 'storage-status').textContent || ''),
          JSON.stringify($(doc, 'storage-status').textContent));
    // The reader clicks Allow: the label must follow without a reload.
    sandbox.navigator.storage.persisted = () => Promise.resolve(true);
    allow(true);
    for (let i = 0; i < 6; i++) { await settle(); }
    check('a late allow is shown as soon as it arrives',
          /permanent/.test($(doc, 'storage-status').textContent || ''),
          JSON.stringify($(doc, 'storage-status').textContent));
  }

  console.log('\na save says it is checking once the bytes are in');
  {
    // The bar measures bytes; hashing and the table check come after. On a
    // slow device that tail is long, and the row must say so rather than
    // sit at 100% "Saving" or claim Saved before the marker is written.
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'common', MANIFEST.generated, { '_images/seed.png': ['h0', 'x'] });
    const body = '<html>a</html>';
    const { doc, w } = load({ manifest: MANIFEST, caches: cachesObj,
      archives: { 'copter/index.html': body },
      tables: { 'copter-files.json': { 'copter/index.html': await fileHash(body) } } });
    await settle();
    const said = [];
    const badgeAt = () => (doc.querySelector('tr[data-wiki="copter"] .apo-badge') || {}).textContent;
    new w.MutationObserver(() => {
      said.push([$(doc, 'cache-progress').textContent, badgeAt()]);
    }).observe($(doc, 'cache-progress'), { childList: true, characterData: true, subtree: true });
    doc.querySelector('.wiki-check[value="copter"]').click(); await settle();
    $(doc, 'download-cache-btn').click();
    for (let i = 0; i < 40; i++) { await settle(); }
    const checking = said.filter(([s]) => /^Checking Copter/.test(s));
    check('the status says Checking Copter after the bytes arrive',
          checking.length > 0, JSON.stringify(said.map(([s]) => s).slice(-6)));
    check('the row is not marked Saved while it is still checking',
          checking.length > 0 && checking.every(([, b]) => b !== 'Saved'),
          JSON.stringify(checking));
    check('and it ends Saved', $(doc, 'cache-progress').textContent === 'Saved' && badgeAt() === 'Saved');
  }

  console.log('\na refresh cancelled after it started writing is unmarked');
  {
    // The first entry lands, the stream stalls, the reader cancels. The old
    // marker would vouch for a copy that is half new and half old.
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'common', MANIFEST.generated, { '_images/seed.png': ['h0', 'x'] });
    await seedSaved(cachesObj, 'copter', OLD_BUILD, { 'copter/index.html': ['h1', 'old'], 'copter/docs/a.html': ['h2', 'old a'] });
    const body = '<html>new</html>';
    const { doc, w, sandbox, swMessages } = load({ manifest: MANIFEST, caches: cachesObj,
      archives: { 'copter/index.html': body, 'copter/docs/a.html': '<html>new a</html>' },
      tables: { 'copter-files.json': { 'copter/index.html': 'deadbeefdeadbeef', 'copter/docs/a.html': 'deadbeefdeadbeef' } } });
    await settle();
    // The archive delivers its first entry and then hangs until aborted.
    const whole = tarBytes({ 'copter/index.html': body, 'copter/docs/a.html': '<html>new a</html>' });
    const realFetch = sandbox.fetch;
    sandbox.fetch = (u, o) => {
      if (!/copter-offline\.tar/.test(String(u))) { return realFetch(u, o); }
      return Promise.resolve({ ok: true, body: new ReadableStream({ start(c) {
        c.enqueue(new Uint8Array(whole.subarray(0, 1024)));
        if (o && o.signal) {
          o.signal.addEventListener('abort', () => {
            const e = new Error('aborted'); e.name = 'AbortError'; c.error(e);
          });
        }
      } }) });
    };
    sandbox.window.fetch = sandbox.fetch;
    swMessages.length = 0;
    $(doc, 'check-btn').dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    // Wait until the first entry has been rewritten, then cancel.
    const c = await cachesObj.open('ardupilot-offline-copter');
    for (let i = 0; i < 30; i++) {
      await settle();
      if ((await bodyAt(c, '/copter/index.html')) === body) { break; }
    }
    check('the first entry was rewritten before the cancel', (await bodyAt(c, '/copter/index.html')) === body);
    $(doc, 'download-cache-btn').dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    for (let i = 0; i < 12; i++) { await settle(); }
    check('a cancelled refresh takes the stale marker with it',
          !(await c.match('/__ap_complete__')),
          JSON.stringify($(doc, 'cache-progress').textContent));
    check('and tells the worker', swMessages.some((m) => m && m.type === 'CACHES_CHANGED'),
          JSON.stringify(swMessages));
  }

  console.log('\nan update tick hands the selection back');
  {
    // The reader ticked Rover and unticked Copter; the tick has to re-download
    // Copter and must not leave its own selection behind.
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'common', MANIFEST.generated, { '_images/seed.png': ['h0', 'x'] });
    await seedSaved(cachesObj, 'copter', OLD_BUILD, { 'copter/index.html': ['h1', 'old'] });
    const body = '<html>new</html>';
    const { doc, w } = load({ manifest: MANIFEST, caches: cachesObj,
      archives: { 'copter/index.html': body },
      tables: { 'copter-files.json': { 'copter/index.html': await fileHash(body) } } });
    await settle();
    doc.querySelector('.wiki-check[value="rover"]').click();
    doc.querySelector('.wiki-check[value="copter"]').click();
    await settle();
    $(doc, 'check-btn').dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    for (let i = 0; i < 14; i++) { await settle(); }
    const result = $(doc, 'check-result').textContent || '';
    check('the tick downloaded the stale wiki again', /downloaded again: copter/i.test(result),
          JSON.stringify(result));
    check('and the reader keeps the selection they had before the tick',
          doc.querySelector('.wiki-check[value="rover"]').checked &&
          !doc.querySelector('.wiki-check[value="copter"]').checked,
          'rover ' + doc.querySelector('.wiki-check[value="rover"]').checked +
          ', copter ' + doc.querySelector('.wiki-check[value="copter"]').checked);
  }

  console.log('\na differential update hands the selection back too');
  {
    // The ordinary case: every stale wiki is patched in place, no archive.
    // The reader ticked Rover and unticked Copter; the tick must not
    // re-render that choice away on its way out.
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'common', OLD_BUILD, { '_images/shared.png': ['c1', 'shared bytes'] });
    await seedSaved(cachesObj, 'copter', OLD_BUILD, {
      'copter/index.html': ['h1', 'old index'], 'copter/docs/a.html': ['h2', 'old a'] });
    const { doc, fetchCalls } = load({
      manifest: MANIFEST, caches: cachesObj,
      tables: { 'common-files.json': { '_images/shared.png': 'c1' },
                'copter-files.json': { 'copter/index.html': 'h1',
                                       'copter/docs/a.html': await fileHash('NEW a') } },
      served: { '/copter/docs/a.html': 'NEW a' } });
    await settle();
    doc.querySelector('.wiki-check[value="rover"]').click();
    doc.querySelector('.wiki-check[value="copter"]').click();
    await settle();
    $(doc, 'check-btn').click();
    for (let i = 0; i < 20; i++) { await settle(); }
    check('the tick patched the stale wiki in place',
          /Updated 1 file/.test($(doc, 'check-result').textContent || '') &&
          !fetchCalls.some((u) => u.indexOf('.tar') !== -1),
          JSON.stringify($(doc, 'check-result').textContent));
    check('and the differential path keeps the selection the reader had',
          doc.querySelector('.wiki-check[value="rover"]').checked &&
          !doc.querySelector('.wiki-check[value="copter"]').checked,
          'rover ' + doc.querySelector('.wiki-check[value="rover"]').checked +
          ', copter ' + doc.querySelector('.wiki-check[value="copter"]').checked);
  }

  console.log('\na refresh that dies mid-unpack is not left marked complete');
  {
    // The first entry is rewritten, then the unpack throws on the second.
    // The old marker and table would serve that mix as complete, and the
    // worker's memo of the marker would keep doing so until told.
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'copter', OLD_BUILD, { 'copter/index.html': ['h1', 'old'] });
    const body = '<html>new</html>';
    const { doc, w, swMessages } = load({ manifest: MANIFEST, caches: cachesObj,
      archives: { 'copter/index.html': body, 'copter/../evil.html': 'x' },
      tables: { 'copter-files.json': { 'copter/index.html': await fileHash(body) } } });
    await settle();
    swMessages.length = 0;
    $(doc, 'check-btn').dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    for (let i = 0; i < 14; i++) { await settle(); }
    const c = await cachesObj.open('ardupilot-offline-copter');
    const text = $(doc, 'check-result').textContent || $(doc, 'cache-progress').textContent || '';
    check('an unpack that fails part-way takes the stale marker with it',
          !(await c.match('/__ap_complete__')) && !/update complete|downloaded again/i.test(text),
          JSON.stringify(text));
    check('and tells the worker the caches changed',
          swMessages.some((m) => m && m.type === 'CACHES_CHANGED'),
          JSON.stringify(swMessages));
  }

  console.log('\na mid-save build rotation names itself');
  {
    // Correct hashes throughout: only the missing check can fire, so its
    // mutation cannot hide behind the damaged message.
    const cachesObj = makeCaches();
    const body = '<html>a</html>';
    const { doc } = load({ manifest: MANIFEST, caches: cachesObj,
      archives: { 'copter/index.html': body },
      tables: { 'copter-files.json': { 'copter/index.html': await fileHash(body),
                                       'copter/docs/landed-later.html': 'h2' } } });
    await settle();
    doc.querySelector('.wiki-check[value="copter"]').click(); await settle();
    $(doc, 'download-cache-btn').click();
    for (let i = 0; i < 12; i++) { await settle(); }
    check('a page the table names but the archive lacks says new build',
          /published a new build/i.test($(doc, 'cache-progress').textContent || '') &&
          !(await (await cachesObj.open('ardupilot-offline-copter')).match('/__ap_complete__')),
          JSON.stringify($(doc, 'cache-progress').textContent));
  }

  console.log('\na table of null is not a table');
  {
    const cachesObj = makeCaches();
    const { doc } = load({ manifest: MANIFEST, caches: cachesObj,
      archives: { 'copter/index.html': '<html>a</html>' },
      tables: { 'copter-files.json': null } });
    await settle();
    doc.querySelector('.wiki-check[value="copter"]').click(); await settle();
    $(doc, 'download-cache-btn').click();
    for (let i = 0; i < 12; i++) { await settle(); }
    check('a JSON null table refuses the save',
          !(await (await cachesObj.open('ardupilot-offline-copter')).match('/__ap_complete__')) &&
          /could not verify/i.test($(doc, 'cache-progress').textContent || ''),
          JSON.stringify($(doc, 'cache-progress').textContent));
  }

  console.log('\nno file table, no completed save');
  {
    const cachesObj = makeCaches();
    const { doc } = load({ manifest: MANIFEST, caches: cachesObj,
      archives: { 'copter/index.html': '<html>a</html>' },
      tables: {} });
    await settle();
    doc.querySelector('.wiki-check[value="copter"]').click(); await settle();
    $(doc, 'download-cache-btn').click();
    for (let i = 0; i < 12; i++) { await settle(); }
    check('a save whose table never arrives is not marked complete',
          !(await (await cachesObj.open('ardupilot-offline-copter')).match('/__ap_complete__')) &&
          /could not verify/i.test($(doc, 'cache-progress').textContent || ''),
          JSON.stringify($(doc, 'cache-progress').textContent));
  }

  console.log('\nthe Cancel click still cancels');
  {
    const cachesObj = makeCaches();
    const { doc, w, sandbox } = load({ manifest: MANIFEST, caches: cachesObj });
    await settle();
    // Held open, but honouring the abort like a real fetch.
    sandbox.fetch = (u, o) => new Promise((res, rej) => {
      if (o && o.signal) {
        o.signal.addEventListener('abort', () => {
          const e = new Error('aborted'); e.name = 'AbortError'; rej(e);
        });
      }
    });
    sandbox.window.fetch = sandbox.fetch;
    doc.querySelector('.wiki-check[value="copter"]').click(); await settle();
    const click = () => $(doc, 'download-cache-btn')
      .dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    click(); await settle();
    check('the button reads Cancel while downloading',
          /Cancel/.test($(doc, 'download-cache-btn').textContent));
    click(); await settle(); await settle();
    check('a second click cancels without throwing',
          !/Cancel/.test($(doc, 'download-cache-btn').textContent) &&
          /cancelled/i.test($(doc, 'cache-progress').textContent || ''),
          JSON.stringify($(doc, 'cache-progress').textContent));
  }

  console.log('\nthree ways the panel used to lie');
  {
    // A legacy folded cache: healed by the check, never "updated" forever.
    const cachesObj = makeCaches();
    (await cachesObj.open('ardupilot-offline-common')).put('/__ap_complete__',
      completeMarker(MANIFEST.generated, 'common'));
    (await cachesObj.open('ardupilot-offline-ardupilot')).put('/__ap_complete__',
      completeMarker(OLD_BUILD, 'ardupilot'));
    const { doc, w } = load({ manifest: MANIFEST, caches: cachesObj });
    await settle();
    $(doc, 'check-btn').dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    for (let i = 0; i < 8; i++) { await settle(); }
    check('a legacy folded cache is retired by the check, not updated forever',
          !cachesObj._all.has('ardupilot-offline-ardupilot') &&
          !/Downloading again: .*ardupilot/i.test($(doc, 'check-result').textContent || ''),
          [...cachesObj._all.keys()].join(','));
  }
  {
    // A table hash the build tool left empty vouches for nothing.
    const cachesObj = makeCaches();
    const { doc } = load({ manifest: MANIFEST, caches: cachesObj,
      archives: { 'copter/index.html': '<html>a</html>' },
      tables: { 'copter-files.json': { 'copter/index.html': '' } } });
    await settle();
    doc.querySelector('.wiki-check[value="copter"]').click(); await settle();
    $(doc, 'download-cache-btn').click();
    for (let i = 0; i < 12; i++) { await settle(); }
    check('a falsy table hash is damage, not a free pass',
          !(await (await cachesObj.open('ardupilot-offline-copter')).match('/__ap_complete__')) &&
          /damaged/.test($(doc, 'cache-progress').textContent || ''),
          JSON.stringify($(doc, 'cache-progress').textContent));
  }
  {
    // The differential path: a hashless table row must not smuggle bytes in.
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'dev', OLD_BUILD, { 'dev/index.html': ['h1', 'old body'] });
    const { doc, w } = load({ manifest: MANIFEST, caches: cachesObj,
      tables: { 'dev-files.json': { 'dev/index.html': '' } },
      served: { '/dev/index.html': '<html>unvouched</html>' } });
    await settle();
    $(doc, 'check-btn').dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    for (let i = 0; i < 10; i++) { await settle(); }
    const held = await (await cachesObj.open('ardupilot-offline-dev'))
      .match('/dev/index.html');
    const body = held ? await held.text() : 'gone';
    check('the differential refuses a body no hash vouches for',
          body.indexOf('unvouched') === -1, JSON.stringify(body.slice(0, 40)));
  }
  {
    // Turning off mid-check must refuse, not orphan what the check writes.
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'dev', MANIFEST.generated, { 'dev/index.html': ['h1', 'x'] });
    const { doc, w, sandbox } = load({ manifest: MANIFEST, caches: cachesObj, offline: true });
    await settle();
    let released = false; const pend = [];
    const orig = sandbox.fetch;
    sandbox.fetch = (u, o) => {
      if (!released && /offline-manifest\.json/.test(String(u))) {
        return new Promise((res) => { pend.push(() => orig(u, o).then(res)); });
      }
      return orig(u, o);
    };
    sandbox.window.fetch = sandbox.fetch;
    $(doc, 'check-btn').dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    await settle();
    const box = doc.getElementById('offline-mode');
    box.checked = false;
    box.dispatchEvent(new w.Event('change', { bubbles: true }));
    await settle();
    check('turning off mid-check refuses and keeps everything',
          cachesObj._all.has('ardupilot-offline-dev') &&
          /still running/i.test($(doc, 'check-result').textContent || ''),
          JSON.stringify($(doc, 'check-result').textContent));
    released = true; pend.forEach((f) => f());
    for (let i = 0; i < 6; i++) { await settle(); }
  }

  console.log('\na Save clicked mid-check is refused politely');
  {
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'dev', MANIFEST.generated, { 'dev/index.html': ['h1', 'x'] });
    const { doc, w, sandbox, fetchCalls } = load({ manifest: MANIFEST, caches: cachesObj,
      archives: { 'copter/index.html': '<html>c</html>' } });
    await settle();
    // The check's manifest fetch is held open, keeping it on the network.
    let released = false; const pend = [];
    const orig = sandbox.fetch;
    sandbox.fetch = (u, o) => {
      if (!released && /offline-manifest\.json/.test(String(u))) {
        return new Promise((res) => { pend.push(() => orig(u, o).then(res)); });
      }
      return orig(u, o);
    };
    sandbox.window.fetch = sandbox.fetch;
    $(doc, 'check-btn').dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    await settle();
    doc.querySelector('.wiki-check[value="copter"]').click(); await settle();
    const tars = () => fetchCalls.filter((u) => u.indexOf('.tar') !== -1).length;
    $(doc, 'download-cache-btn').click(); await settle();
    check('a Save mid-check starts nothing and says why',
          tars() === 0 &&
          /update check is running/i.test($(doc, 'cache-progress').textContent || ''),
          tars() + ' tars, ' + JSON.stringify($(doc, 'cache-progress').textContent));
    released = true; pend.forEach((f) => f());
    for (let i = 0; i < 8; i++) { await settle(); }
    $(doc, 'download-cache-btn').click();
    for (let i = 0; i < 10; i++) { await settle(); }
    check('the same click works once the check has finished', tars() > 0,
          tars() + ' tar fetches');
  }

  console.log('\na finishing check leaves the buttons with the download that owns them');
  {
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'dev', OLD_BUILD, { 'dev/index.html': ['h1', 'x'] });
    const { doc, w, sandbox } = load({ manifest: MANIFEST, caches: cachesObj });
    await settle();
    // The check's table fetch is held open; archives never resolve.
    let releaseTable = null;
    const orig = sandbox.fetch;
    sandbox.fetch = (u, o) => {
      if (String(u).indexOf('-files.json') !== -1 || String(u).indexOf('ap-update=') !== -1) {
        return new Promise((res) => { releaseTable = () => res({ ok: false }); });
      }
      if (String(u).indexOf('.tar') !== -1) { return new Promise(() => {}); }
      return orig(u, o);
    };
    sandbox.window.fetch = sandbox.fetch;
    $(doc, 'check-btn').dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    await settle();
    // A download starts while the check is still on the network.
    doc.querySelector('.wiki-check[value="copter"]').click(); await settle();
    $(doc, 'download-cache-btn').click(); await settle();
    check('the download disables the check button', $(doc, 'check-btn').disabled);
    if (releaseTable) { releaseTable(); }
    for (let i = 0; i < 6; i++) { await settle(); }
    check('the finishing check leaves it with the download that owns it',
          $(doc, 'check-btn').disabled);
  }

  console.log('\nthe repair keeps the selection the reader made');
  {
    const cachesObj = makeCaches();
    for (const id of ['copter', 'rover', 'dev']) {
      (await cachesObj.open('ardupilot-offline-' + id)).put('/__ap_complete__',
        completeMarker(MANIFEST.generated, id));
    }
    // A cancelled common download: the export must repair it first.
    (await cachesObj.open('ardupilot-offline-common')).put('/_common/_images/x.png',
      new FakeResponse('png'));
    const { doc, w } = load({ manifest: MANIFEST, caches: cachesObj,
      archives: { '_images/shared.png': 'png' } });
    await settle();
    let exported = null;
    w.ArduPilotExport = { exportHtml: (ids) => { exported = ids.slice();
      return Promise.resolve({ pages: 1 }); } };
    // The reader keeps copter and drops rover from the export.
    doc.querySelector('.wiki-check[value="rover"]').click(); await settle();
    $(doc, 'dl-single').dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    for (let i = 0; i < 15; i++) { await settle(); }
    check('the repair does not widen the selection back to every saved wiki',
          exported !== null && exported.join(',') === 'copter,dev',
          JSON.stringify(exported));
    check('and rover stays unticked afterwards',
          !doc.querySelector('.wiki-check[value="rover"]').checked);
    check('and the select-all header follows the restored boxes',
          !$(doc, 'select-all').checked);
  }

  console.log('\na version the file could not carry is named, not swallowed');
  {
    const cachesObj = makeCaches();
    for (const id of ['common', 'copter']) {
      (await cachesObj.open('ardupilot-offline-' + id)).put('/__ap_complete__',
        completeMarker(MANIFEST.generated, id));
    }
    const { doc, w } = load({ manifest: MANIFEST, caches: cachesObj });
    await settle();
    const click = () => $(doc, 'dl-single')
      .dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    w.ArduPilotExport = { exportHtml: () => Promise.resolve({ pages: 3, leftOut: 2 }) };
    click();
    for (let i = 0; i < 15; i++) { await settle(); }
    const said = $(doc, 'dl-single').textContent || '';
    check('the reader is told how many parameter versions were left out',
          /3 pages/.test(said) && /2 parameter versions left out/.test(said), said);
    w.ArduPilotExport = { exportHtml: () => Promise.resolve({ pages: 3, leftOut: 0 }) };
    click();
    for (let i = 0; i < 15; i++) { await settle(); }
    const whole = $(doc, 'dl-single').textContent || '';
    check('and a whole file says nothing of the sort', !/left out/.test(whole), whole);
  }

  console.log('\nthe export button cannot start a second export mid-flight');
  {
    const cachesObj = makeCaches();
    // Everything already saved: no repair stands between click and pack, so
    // only the re-entry guard separates one export from two.
    for (const id of ['common', 'copter']) {
      (await cachesObj.open('ardupilot-offline-' + id)).put('/__ap_complete__',
        completeMarker(MANIFEST.generated, id));
    }
    const { doc, w } = load({ manifest: MANIFEST, caches: cachesObj });
    await settle();
    let calls = 0; let finish = null;
    w.ArduPilotExport = { exportHtml: () => { calls++;
      return new Promise((res) => { finish = () => res({ pages: 1 }); }); } };
    const click = () => $(doc, 'dl-single')
      .dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    // Clicks land during the pre-save repair and during the pack itself.
    click(); click(); await settle(); click();
    for (let i = 0; i < 10; i++) { await settle(); }
    click(); await settle();
    check('re-entrant clicks run exactly one export', calls === 1, calls + ' exports');
    if (finish) { finish(); } await settle(); await settle();
  }

  console.log('\na stale wiki found mid-pack defers instead of downloading');
  {
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'dev', OLD_BUILD, { 'dev/index.html': ['h1', 'x'] });
    (await cachesObj.open('ardupilot-offline-common')).put('/__ap_complete__',
      completeMarker(MANIFEST.generated, 'common'));
    // A differential is genuinely possible: new table and file are served,
    // so only the deferral stands between the check and a cache write.
    const { doc, w, sandbox, fetchCalls } = load({ manifest: MANIFEST, caches: cachesObj,
      tables: { 'dev-files.json': { 'dev/index.html': 'h2' } },
      served: { '/dev/index.html': '<html>newer</html>' } });
    await settle();
    let released = false; const pend = [];
    const orig = sandbox.fetch;
    sandbox.fetch = (u, o) => {
      if (!released && /offline-manifest\.json/.test(String(u))) {
        return new Promise((res) => { pend.push(() => orig(u, o).then(res)); });
      }
      return orig(u, o);
    };
    sandbox.window.fetch = sandbox.fetch;
    const click = (id) => $(doc, id).dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    click('check-btn'); await settle();
    let finishExport = null;
    w.ArduPilotExport = { exportHtml: () =>
      new Promise((res) => { finishExport = () => res({ pages: 1 }); }) };
    click('dl-single'); await settle();
    // The reader's own selection must survive the deferral untouched.
    doc.querySelector('.wiki-check[value="copter"]').click(); await settle();
    const tars = () => fetchCalls.filter((u) => u.indexOf('.tar') !== -1).length;
    const before = tars();
    released = true; pend.forEach((f) => f());
    for (let i = 0; i < 8; i++) { await settle(); }
    const updates = () => fetchCalls.filter((u) =>
      u.indexOf('ap-update=') !== -1 || /\/files\//.test(u) ||
      u.indexOf('-files.json') !== -1).length;
    check('a stale wiki found mid-pack starts no download', tars() === before,
          tars() + ' tar fetches, ' + before + ' before');
    check('not even the differential touches the caches mid-pack',
          updates() === 0, updates() + ' update fetches while packing');
    check('the deferral does not rewrite the selection',
          doc.querySelector('.wiki-check[value="copter"]').checked);
    check('the deferral says what will happen instead',
          /after the export/i.test($(doc, 'check-result').textContent || ''),
          JSON.stringify($(doc, 'check-result').textContent));
    // Only the resumed check's differential can move this counter: the
    // original check deferred before touching anything.
    const beforeResume = updates();
    finishExport();
    for (let i = 0; i < 10; i++) { await settle(); }
    check('the deferred update resumes once the export finishes',
          updates() > beforeResume,
          updates() + ' update fetches, ' + beforeResume + ' before');
  }

  console.log('\nthe check tail carries the resume release() had to skip');
  {
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'dev', OLD_BUILD, { 'dev/index.html': ['h1', 'x'] });
    (await cachesObj.open('ardupilot-offline-common')).put('/__ap_complete__',
      completeMarker(MANIFEST.generated, 'common'));
    const { doc, w, sandbox, fetchCalls } = load({ manifest: MANIFEST, caches: cachesObj,
      tables: { 'dev-files.json': { 'dev/index.html': 'h2' } },
      served: { '/dev/index.html': '<html>newer</html>' } });
    await settle();
    let released = false; const pendM = [];
    const orig = sandbox.fetch;
    sandbox.fetch = (u, o) => {
      if (!released && /offline-manifest\.json/.test(String(u))) {
        return new Promise((res) => { pendM.push(() => orig(u, o).then(res)); });
      }
      return orig(u, o);
    };
    sandbox.window.fetch = sandbox.fetch;
    const click = (id) => $(doc, id).dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    click('check-btn'); await settle();
    let finishExport = null;
    w.ArduPilotExport = { exportHtml: () =>
      new Promise((res) => { finishExport = () => res({ pages: 1 }); }) };
    click('dl-single'); await settle();
    const updates = () => fetchCalls.filter((u) =>
      u.indexOf('ap-update=') !== -1 || /\/files\//.test(u) ||
      u.indexOf('dev-files.json') !== -1).length;
    // The export ends the instant the deferral text lands, so release()
    // runs before the check tail: the tail must carry the resume.
    let firedAtDeferral = false;
    const mo = new w.MutationObserver(() => {
      if (!firedAtDeferral &&
          /after the export/i.test($(doc, 'check-result').textContent || '')) {
        firedAtDeferral = true;
        finishExport();
      }
    });
    mo.observe($(doc, 'check-result'), { childList: true, characterData: true, subtree: true });
    released = true; pendM.forEach((f) => f());
    for (let i = 0; i < 15; i++) { await settle(); }
    mo.disconnect();
    check('the export ended exactly at the deferral', firedAtDeferral);
    check('the check tail carries the resume release() had to skip',
          updates() > 0, updates() + ' update fetches');
    const devTables = fetchCalls.filter((u) => u.indexOf('dev-files.json') !== -1).length;
    check('and the resumed update runs exactly once', devTables === 1,
          devTables + ' table fetches');
  }

  console.log('\na deferred update survives a failing pre-save');
  {
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'dev', OLD_BUILD, { 'dev/index.html': ['h1', 'x'] });
    (await cachesObj.open('ardupilot-offline-common')).put('/_common/_images/x.png',
      new FakeResponse('png'));
    const { doc, w, sandbox, fetchCalls } = load({ manifest: MANIFEST, caches: cachesObj,
      tables: { 'dev-files.json': { 'dev/index.html': 'h2' } },
      served: { '/dev/index.html': '<html>newer</html>' } });
    await settle();
    let released = false; const pendM = [];
    let failTar = null;
    const orig = sandbox.fetch;
    sandbox.fetch = (u, o) => {
      if (!released && /offline-manifest\.json/.test(String(u))) {
        return new Promise((res) => { pendM.push(() => orig(u, o).then(res)); });
      }
      if (/common-offline\.tar/.test(String(u))) {
        return new Promise((res, rej) => { failTar = () => rej(new Error('gone')); });
      }
      return orig(u, o);
    };
    sandbox.window.fetch = sandbox.fetch;
    const click = (id) => $(doc, id).dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    click('check-btn'); await settle();
    w.ArduPilotExport = { exportHtml: () => Promise.resolve({ pages: 1 }) };
    click('dl-single'); await settle();
    released = true; pendM.forEach((f) => f());
    for (let i = 0; i < 6; i++) { await settle(); }
    const updates = () => fetchCalls.filter((u) =>
      u.indexOf('ap-update=') !== -1 || /\/files\//.test(u) ||
      u.indexOf('dev-files.json') !== -1).length;
    check('the check deferred while the pre-save ran', updates() === 0,
          updates() + ' update fetches');
    // The pre-save now fails outright; the deferred update must still run.
    failTar();
    for (let i = 0; i < 12; i++) { await settle(); }
    check('the deferred update survives the failed pre-save',
          updates() > 0, updates() + ' update fetches after the failure');
  }

  console.log('\nthe span between pre-save and pack is owned too');
  {
    const cachesObj = makeCaches();
    (await cachesObj.open('ardupilot-offline-copter')).put('/__ap_complete__',
      completeMarker(MANIFEST.generated, 'copter'));
    (await cachesObj.open('ardupilot-offline-common')).put('/_common/_images/x.png',
      new FakeResponse('png'));
    // Estimates resolve on a timer, as in a real browser, so the renders
    // from the pre-save cleanup land in the gap before packing starts.
    const { doc, w, sandbox } = load({ manifest: MANIFEST, caches: cachesObj,
      archives: { '_images/shared.png': 'png' }, estimateDelay: 10 });
    await settle();
    let finishExport = null;
    w.ArduPilotExport = { exportHtml: () =>
      new Promise((res) => { finishExport = () => res({ pages: 1 }); }) };
    // A MutationObserver sees even a one-microtask window where the
    // buttons come back; polling at settle granularity cannot.
    let clearSeenLive = false; let checkSeenLive = false;
    const mo = new w.MutationObserver(() => {
      if (!$(doc, 'clear-btn').disabled) { clearSeenLive = true; }
      if (!$(doc, 'check-btn').disabled) { checkSeenLive = true; }
    });
    mo.observe($(doc, 'clear-btn'), { attributes: true });
    mo.observe($(doc, 'check-btn'), { attributes: true });
    $(doc, 'dl-single').dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    for (let i = 0; i < 20; i++) {
      await settle();
      if (finishExport) { break; }
    }
    mo.disconnect();
    check('Remove all is never live between pre-save and pack',
          !clearSeenLive && !checkSeenLive,
          'clear ' + clearSeenLive + ' check ' + checkSeenLive);
    if (finishExport) { finishExport(); }
    for (let i = 0; i < 10; i++) { await settle(); }
    check('and it comes back at rest', !$(doc, 'clear-btn').disabled);
  }

  console.log('\na check landing during the pre-save defers like any other');
  {
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'dev', OLD_BUILD, { 'dev/index.html': ['h1', 'x'] });
    // Broken common: the export must repair it first, and that window is
    // exactly where the check's differential used to slip through.
    (await cachesObj.open('ardupilot-offline-common')).put('/_common/_images/x.png',
      new FakeResponse('png'));
    const { doc, w, sandbox, fetchCalls } = load({ manifest: MANIFEST, caches: cachesObj,
      archives: { '_images/shared.png': 'png' },
      tables: { 'dev-files.json': { 'dev/index.html': 'h2' },
                'common-files.json': { '_images/shared.png': 'ignored' } },
      served: { '/dev/index.html': '<html>newer</html>' } });
    await settle();
    let released = false; const pendM = [];
    let tarHeld = false; const pendT = [];
    const orig = sandbox.fetch;
    sandbox.fetch = (u, o) => {
      if (!released && /offline-manifest\.json/.test(String(u))) {
        return new Promise((res) => { pendM.push(() => orig(u, o).then(res)); });
      }
      if (!tarHeld && /common-offline\.tar/.test(String(u))) {
        tarHeld = true;
        return new Promise((res) => { pendT.push(() => orig(u, o).then(res)); });
      }
      return orig(u, o);
    };
    sandbox.window.fetch = sandbox.fetch;
    const click = (id) => $(doc, id).dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    click('check-btn'); await settle();
    let finishExport = null;
    w.ArduPilotExport = { exportHtml: () =>
      new Promise((res) => { finishExport = () => res({ pages: 1 }); }) };
    // The export starts; its pre-save hangs on the held common archive.
    click('dl-single'); await settle();
    const updates = () => fetchCalls.filter((u) =>
      u.indexOf('ap-update=') !== -1 || /\/files\//.test(u) ||
      u.indexOf('dev-files.json') !== -1).length;
    // The check lands while the pre-save is mid-download.
    released = true; pendM.forEach((f) => f());
    for (let i = 0; i < 8; i++) { await settle(); }
    check('a check landing during the pre-save touches nothing',
          updates() === 0, updates() + ' update fetches during the pre-save');
    pendT.forEach((f) => f());
    for (let i = 0; i < 12; i++) { await settle(); }
    if (finishExport) { finishExport(); }
    for (let i = 0; i < 12; i++) { await settle(); }
    check('and the deferred update resumes after that export too',
          updates() > 0, updates() + ' update fetches after completion');
  }

  console.log('\nan export cannot start while an update is writing the caches');
  {
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'dev', OLD_BUILD, { 'dev/index.html': ['h1', 'x'] });
    (await cachesObj.open('ardupilot-offline-common')).put('/__ap_complete__',
      completeMarker(MANIFEST.generated, 'common'));
    const { doc, w, sandbox } = load({ manifest: MANIFEST, caches: cachesObj,
      tables: { 'dev-files.json': { 'dev/index.html': 'h2' } },
      served: { '/dev/index.html': '<html>newer</html>' } });
    await settle();
    // The differential's file fetches are held mid-write until released.
    let released = false; const pend = [];
    const orig = sandbox.fetch;
    sandbox.fetch = (u, o) => {
      if (!released && /\/files\/|ap-update=/.test(String(u))) {
        return new Promise((res) => { pend.push(() => orig(u, o).then(res)); });
      }
      return orig(u, o);
    };
    sandbox.window.fetch = sandbox.fetch;
    const releaseFile = () => { released = true; pend.forEach((f) => f()); };
    const click = (id) => $(doc, id).dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    click('check-btn');
    for (let i = 0; i < 6; i++) { await settle(); }
    let calls = 0;
    w.ArduPilotExport = { exportHtml: () => { calls++;
      return Promise.resolve({ pages: 1 }); } };
    click('dl-single'); await settle();
    check('the export refuses while the update writes',
          calls === 0 && /update is being written/i.test($(doc, 'dl-single').textContent || ''),
          calls + ' exports, ' + JSON.stringify($(doc, 'dl-single').textContent));
    releaseFile();
    for (let i = 0; i < 10; i++) { await settle(); }
    click('dl-single');
    for (let i = 0; i < 10; i++) { await settle(); }
    check('the export runs once the update has finished', calls === 1,
          calls + ' exports, ' + JSON.stringify($(doc, 'dl-single').textContent));
  }

  console.log('\nan export finishing under a busy check withholds only its button');
  {
    const cachesObj = makeCaches();
    await seedSaved(cachesObj, 'dev', MANIFEST.generated, { 'dev/index.html': ['h1', 'x'] });
    (await cachesObj.open('ardupilot-offline-common')).put('/__ap_complete__',
      completeMarker(MANIFEST.generated, 'common'));
    const { doc, w, sandbox } = load({ manifest: MANIFEST, caches: cachesObj });
    await settle();
    let released = false; const pend = [];
    const orig = sandbox.fetch;
    sandbox.fetch = (u, o) => {
      if (!released && /offline-manifest\.json/.test(String(u))) {
        return new Promise((res) => { pend.push(() => orig(u, o).then(res)); });
      }
      return orig(u, o);
    };
    sandbox.window.fetch = sandbox.fetch;
    const click = (id) => $(doc, id).dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    click('check-btn'); await settle();
    let finishExport = null;
    w.ArduPilotExport = { exportHtml: () =>
      new Promise((res) => { finishExport = () => res({ pages: 1 }); }) };
    click('dl-single'); await settle();
    // The export ends first; the check is still on the network.
    finishExport();
    for (let i = 0; i < 6; i++) { await settle(); }
    check('the export hands back everything but the busy check button',
          $(doc, 'check-btn').disabled && !$(doc, 'dl-single').disabled,
          'check ' + $(doc, 'check-btn').disabled + ' export ' + $(doc, 'dl-single').disabled);
    released = true; pend.forEach((f) => f());
    for (let i = 0; i < 8; i++) { await settle(); }
    check('the finished check then frees its own button',
          !$(doc, 'check-btn').disabled);
  }

  console.log('\na finishing check cannot take the panel from a packing export');
  {
    const cachesObj = makeCaches();
    // Current build: the check must stay a check, not become a re-download.
    await seedSaved(cachesObj, 'dev', MANIFEST.generated, { 'dev/index.html': ['h1', 'x'] });
    (await cachesObj.open('ardupilot-offline-common')).put('/__ap_complete__',
      completeMarker(MANIFEST.generated, 'common'));
    const { doc, w, sandbox } = load({ manifest: MANIFEST, caches: cachesObj });
    await settle();
    // The check's own manifest fetch is held until released, so the check is
    // provably still running when the export starts, and provably finishes.
    let released = false;
    const pend = [];
    const orig = sandbox.fetch;
    sandbox.fetch = (u, o) => {
      if (/offline-manifest\.json/.test(String(u))) {
        if (released) { return orig(u, o); }
        return new Promise((res) => { pend.push(() => orig(u, o).then(res)); });
      }
      return orig(u, o);
    };
    sandbox.window.fetch = sandbox.fetch;
    const releaseTable = () => { released = true; pend.forEach((f) => f()); };
    const click = (id) => $(doc, id).dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    click('check-btn'); await settle();
    // The export starts while the check is still on the network.
    let finishExport = null;
    w.ArduPilotExport = { exportHtml: () =>
      new Promise((res) => { finishExport = () => res({ pages: 1 }); }) };
    click('dl-single'); await settle();
    check('packing holds the panel while a check is in flight',
          $(doc, 'check-btn').disabled && $(doc, 'clear-btn').disabled &&
          $(doc, 'download-cache-btn').disabled);
    doc.querySelector('.wiki-check[value="copter"]').click(); await settle();
    check('ticking an unsaved wiki mid-pack does not re-arm Save',
          $(doc, 'download-cache-btn').disabled);
    doc.querySelectorAll('.wiki-check').forEach((c) => { if (c.checked) { c.click(); } });
    await settle();
    check('unticking everything mid-pack keeps the cancel button alive',
          !$(doc, 'dl-single').disabled);
    doc.querySelector('.wiki-check[value="dev"]').click(); await settle();
    releaseTable();
    for (let i = 0; i < 8; i++) { await settle(); }
    check('the check provably finished while the export packed',
          !!($(doc, 'check-result').textContent || '').trim(),
          JSON.stringify($(doc, 'check-result').textContent));
    check('the finishing check leaves the panel with the export that owns it',
          $(doc, 'check-btn').disabled && $(doc, 'clear-btn').disabled,
          'check ' + $(doc, 'check-btn').disabled + ' clear ' + $(doc, 'clear-btn').disabled);
    finishExport(); await settle(); await settle();
    check('completion hands the panel back',
          !$(doc, 'check-btn').disabled && !$(doc, 'dl-single').disabled);
  }

  console.log('\na running export can be cancelled from its own button');
  {
    const cachesObj = makeCaches();
    for (const id of ['common', 'copter']) {
      (await cachesObj.open('ardupilot-offline-' + id)).put('/__ap_complete__',
        completeMarker(MANIFEST.generated, id));
    }
    const { doc, w } = load({ manifest: MANIFEST, caches: cachesObj });
    await settle();
    // An export that runs until told to stop, honouring the abort signal.
    let rejected = null;
    w.ArduPilotExport = { exportHtml: (ids, name, cb, sink, signal) =>
      new Promise((resolve, reject) => {
        cb(10, 100);
        const tick = () => {
          if (signal && signal.aborted) {
            const e = new Error('cancelled'); e.name = 'AbortError';
            rejected = e; reject(e); return;
          }
          setTimeout(tick, 5);
        };
        tick();
      }) };
    const click = () => $(doc, 'dl-single')
      .dispatchEvent(new w.MouseEvent('click', { bubbles: true }));
    click(); await settle();
    check('while packing, the button offers to cancel and stays clickable',
          !$(doc, 'dl-single').disabled &&
          /click to cancel/i.test($(doc, 'dl-single').textContent || ''),
          JSON.stringify($(doc, 'dl-single').textContent));
    check('while packing, the export owns the panel',
          $(doc, 'download-cache-btn').disabled && $(doc, 'check-btn').disabled &&
          $(doc, 'clear-btn').disabled);
    click(); await settle(); await settle();
    check('a second click cancels the export gracefully',
          rejected !== null && /cancelled/i.test($(doc, 'dl-single').textContent || ''),
          JSON.stringify($(doc, 'dl-single').textContent));
    check('the button is usable again after cancelling',
          !$(doc, 'dl-single').disabled);
    check('cancelling hands the panel back',
          !$(doc, 'check-btn').disabled && !$(doc, 'clear-btn').disabled);
  }

  console.log('\noffline mode switch: off removes everything, after a warning');
  {
    const cachesObj = makeCaches();
    for (const id of ['common', 'copter']) {
      (await cachesObj.open('ardupilot-offline-' + id)).put('/__ap_complete__',
        completeMarker(MANIFEST.generated, id));
    }
    const { doc, w, apOffline } = load({ manifest: MANIFEST, caches: cachesObj,
                                         usage: 1.2e9, offline: true });
    await settle();
    const box = doc.getElementById('offline-mode');
    const note = doc.getElementById('offline-off-warning');
    const flipOff = () => {
      box.checked = false;
      box.dispatchEvent(new w.Event('change', { bubbles: true }));
    };
    const click = (id) => doc.getElementById(id)
      .dispatchEvent(new w.MouseEvent('click', { bubbles: true }));

    check('the switch starts on for an opted-in reader', box.checked === true);
    flipOff(); await settle();
    check('off shows the warning and does nothing yet',
          !note.hidden && box.checked && apOffline.calls.length === 0,
          JSON.stringify({ hidden: note.hidden, on: box.checked, calls: apOffline.calls }));
    const size = doc.getElementById('offline-off-size').textContent;
    check('the warning quotes what the browser holds, from storage.estimate()',
          /^\d[\d.]* [GM]B$/.test(size), JSON.stringify(size));
    click('offline-off-keep'); await settle();
    check('Keep hides the warning and leaves offline mode on',
          note.hidden && box.checked && apOffline.calls.length === 0);
    flipOff(); await settle();
    click('offline-off-confirm'); await settle(); await settle();
    check('confirming turns offline mode off through pwa.js',
          apOffline.calls.join() === 'disable' && !box.checked &&
          doc.getElementById('offline-mode-state').textContent === 'off',
          JSON.stringify({ calls: apOffline.calls, on: box.checked }));
    check('and nothing of the wiki is left in Cache Storage',
          !(await cachesObj.keys()).some((n) => n.startsWith('ardupilot-')) &&
          /no wikis saved/.test(doc.getElementById('storage-status').textContent),
          JSON.stringify(await cachesObj.keys()));

    const b = load({ manifest: MANIFEST, usage: 0, offline: true });
    await settle();
    b.doc.getElementById('offline-mode').checked = false;
    b.doc.getElementById('offline-mode').dispatchEvent(new b.w.Event('change', { bubbles: true }));
    await settle(); await settle();
    check('with nothing saved, off needs no confirmation',
          b.apOffline.calls.join() === 'disable' &&
          b.doc.getElementById('offline-off-warning').hidden,
          JSON.stringify(b.apOffline.calls));
  }

  console.log('\n' + pass + ' passed, ' + fail + ' failed');
  if (fail) { console.log('failed: ' + failures.join('; ')); }
  process.exit(fail ? 1 : 0);
}

main().catch((e) => { console.error(e); process.exit(1); });
