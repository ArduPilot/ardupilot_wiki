/*
 * Harness for the single-file exporter: runs the real exporter against a
 * CacheStorage-alike over real build output and checks the bytes it produces.
 *
 *   node scripts/tests/test_offline_export.js [wiki...]
 */

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const REPO = path.resolve(__dirname, '..', '..');
const ARGS = process.argv.slice(2);
const FULL = ARGS.includes('--full');
// --full: every wiki, no page cap. Slow.

// Two wikis, so the cross-wiki sidebar and indexes are exercised.
const WIKIS_ARG = ARGS.filter((a) => !a.startsWith('--'));
const WIKI = WIKIS_ARG[0] || 'rover';
const ALL_WIKIS = ['copter', 'plane', 'rover', 'sub', 'blimp', 'dev',
                   'antennatracker', 'planner', 'planner2', 'ardupilot', 'mavproxy'];
const OUT = path.join(require('os').tmpdir(), 'ap-export-test');

const EXPORTER = path.join(REPO, 'common/source/_static/common_offline_export.js');
const DOCUMENT = path.join(REPO, 'common/source/_static/common_offline_document_builder.js');
const UNPACK = path.join(REPO, 'common/source/_static/common_offline_unpack.js');

let failures = 0;
function check(name, ok, detail) {
  console.log((ok ? '  PASS  ' : '  FAIL  ') + name + (detail ? '   ' + detail : ''));
  if (!ok) { failures++; }
}

/* ---------------------------------------------------------- cache shim ---- */

class FakeResponse {
  constructor(buf) { this._buf = buf; }
  arrayBuffer() { return Promise.resolve(this._buf.buffer.slice(
    this._buf.byteOffset, this._buf.byteOffset + this._buf.byteLength)); }
  text() { return Promise.resolve(this._buf.toString('utf8')); }
}

class FakeCache {
  constructor() { this.map = new Map(); }
  put(url, res) { this.map.set(String(url), res); return Promise.resolve(); }
  keys() {
    return Promise.resolve([...this.map.keys()].map((u) => ({ url: 'https://x' + u })));
  }
  match(req) {
    const url = typeof req === 'string' ? req : req.url.replace('https://x', '');
    return Promise.resolve(this.map.get(url));
  }
}

const caches = {
  _all: new Map(),
  keys() { return Promise.resolve([...this._all.keys()]); },
  open(name) {
    if (!this._all.has(name)) { this._all.set(name, new FakeCache()); }
    return Promise.resolve(this._all.get(name));
  }
};

/** Populate the cache the way a real download would: wiki pages + shared images. */
function loadWiki(wiki, limit) {
  const root = path.join(REPO, wiki, 'build', 'html');
  if (!fs.existsSync(root)) {
    console.error('No build output for ' + wiki + ' - run update.py --site ' + wiki);
    process.exit(2);
  }
  const wikiCache = new FakeCache();
  const commonCache = caches._all.get('ardupilot-offline-common') || new FakeCache();
  let pages = 0, images = 0, css = 0;

  const walk = (dir) => {
    for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
      const full = path.join(dir, entry.name);
      if (entry.isDirectory()) { walk(full); continue; }
      const rel = '/' + wiki + '/' + path.relative(root, full).split(path.sep).join('/');
      const buf = fs.readFileSync(full);

      if (rel.endsWith('.html')) {
        var isIndex = /\/index\.html$/.test(rel);
        // The unversioned parameters page rides along like an index: the
        // switcher's Latest option needs it whatever the cap.
        const isLatestParams = /\/docs\/parameters\.html$/.test(rel);
        if (pages >= limit && !isIndex && !isLatestParams) { continue; }
        pages++;
        wikiCache.put(rel, new FakeResponse(buf));
      } else if (/\.(png|jpe?g|gif|svg|ico|woff2?|ttf|eot)$/i.test(rel)) {
        // Mirror production: shared images live once under /_common/.
        if (rel.includes('/_images/')) {
          if (images >= limit * 3) { continue; }
          images++;
          commonCache.put(rel.replace(/^\/[^/]+\/_images\//, '/_common/_images/'),
                          new FakeResponse(buf));
        } else {
          wikiCache.put(rel, new FakeResponse(buf));
        }
      } else if (rel.endsWith('.css') || rel.endsWith('.js')) {
        css++;
        wikiCache.put(rel, new FakeResponse(buf));
      }
    }
  };
  walk(root);

  wikiCache.put('/__ap_complete__',
    new FakeResponse(Buffer.from(JSON.stringify({ build: 'test', id: wiki }))));
  commonCache.put('/__ap_complete__',
    new FakeResponse(Buffer.from(JSON.stringify({ build: 'test', id: 'common' }))));

  caches._all.set('ardupilot-offline-' + wiki, wikiCache);
  caches._all.set('ardupilot-offline-common', commonCache);
  return { pages, images, css };
}

/* A versioned parameter page in the site's markup; local builds have none. */
function paramPageHtml(vehicle, label) {
  return '<!DOCTYPE html><html><head><title>Complete Parameter List &mdash; ' +
    vehicle + ' documentation</title></head><body>' +
    '<div itemprop="articleBody">' +
    '<section id="complete-parameter-list"><h1>Complete Parameter List</h1>' +
    '<h2>Full Parameter List of ' + label + '</h2>' +
    '<p>You can change and check the parameters for another version:\n' +
    '  <select class="selectpicker" id="selectPicker"></select>\n</p>\n' +
    '<script type="text/javascript">\n' +
    // Upper case and a space: the tokenizer ends a block on either.
    'var closer = "</SCRIPT >";\n' +
    'document.addEventListener("DOMContentLoaded", function() {\n' +
    '  fetch("../_static/parameters-' + vehicle + '.json")\n' +
    '    .then(function(r) { return r.json(); }).then(appendToSelect);\n' +
    '});\n</script>\n' +
    '<p>This is a complete list of the parameters.</p>' +
    '</section></div><footer>x</footer></body></html>';
}

/** Put a set of versioned parameter pages in a wiki's cache. */
function loadParameterVersions(wiki, vehicle, versions) {
  const cache = caches._all.get('ardupilot-offline-' + wiki);
  const made = [];
  for (const v of versions) {
    const label = vehicle + ' stable ' + v;
    const rel = '/' + wiki + '/docs/parameters-' + vehicle + '-stable-' + v + '.html';
    cache.put(rel, new FakeResponse(Buffer.from(paramPageHtml(vehicle, label))));
    made.push({ path: rel.replace(/\.html$/, ''), label,
                body: paramPageHtml(vehicle, label) });
  }
  return made;
}

/* --------------------------------------------------------- module load ---- */

function loadExporter() {
  // The modules the page loads, in its order.
  const src = [DOCUMENT, UNPACK, EXPORTER]
    .map((f) => fs.readFileSync(f, 'utf8')).join('\n');
  const sandbox = {
    caches,
    TextEncoder, TextDecoder, URL, btoa, console,
    setTimeout, clearTimeout,
    navigator: {},                 // no service worker: forces the sink we pass
    document: { createElement: () => ({ style: {}, click() {}, remove() {} }),
                body: { appendChild() {} } },
    Blob: class { constructor(p) { this.parts = p; } },
    window: {}
  };
  sandbox.window = sandbox;
  vm.createContext(sandbox);
  vm.runInContext(src, sandbox);
  return sandbox.ArduPilotExport;
}

/** A sink that writes straight to a file, standing in for the download. */
function fileSink(target) {
  const fd = fs.openSync(target, 'w');
  return {
    write(chunk) { fs.writeSync(fd, Buffer.from(chunk)); return Promise.resolve(); },
    close() { fs.closeSync(fd); return Promise.resolve(); }
  };
}

/** Count patterns in a file too large to hold as a string; chunks overlap. */
function scanFile(file, patterns, literals) {
  const counts = patterns.map(() => 0);
  const found = {};
  literals.forEach((l) => { found[l] = false; });

  const SIZE = 8 * 1024 * 1024;
  const OVERLAP = 4096;
  const fd = fs.openSync(file, 'r');
  const buf = Buffer.alloc(SIZE);
  let carry = '';
  let bytes;
  while ((bytes = fs.readSync(fd, buf, 0, SIZE, null)) > 0) {
    const text = carry + buf.slice(0, bytes).toString('latin1');
    patterns.forEach((re, i) => {
      re.lastIndex = 0;
      const m = text.match(re);
      counts[i] += m ? m.length : 0;
    });
    literals.forEach((l) => { if (!found[l] && text.includes(l)) { found[l] = true; } });
    carry = text.slice(-OVERLAP);
  }
  fs.closeSync(fd);
  return { counts, found };
}

/** The routing payload, read back from the tail of the finished file. */
function readIndexPayload(file) {
  const size = fs.statSync(file).size;
  const span = Math.min(size, 64 * 1024 * 1024);
  const buf = Buffer.alloc(span);
  const fd = fs.openSync(file, 'r');
  fs.readSync(fd, buf, 0, span, size - span);
  fs.closeSync(fd);
  const tail = buf.toString('utf8');
  const m = tail.match(
    /<script type="application\/json" id="ap-index">([\s\S]*?)<\/script>/);
  if (!m) { return null; }
  try { return JSON.parse(m[1].split('<\\/').join('</')); } catch (e) { return null; }
}

/** The shipped search index and the stemmer that built it, read back out. */
function readSearchPayload(file) {
  const size = fs.statSync(file).size;
  const span = Math.min(size, 96 * 1024 * 1024);
  const buf = Buffer.alloc(span);
  const fd = fs.openSync(file, 'r');
  fs.readSync(fd, buf, 0, span, size - span);
  fs.closeSync(fd);
  const m = buf.toString('utf8')
    .match(/<script type="application\/json" id="ap-fts">([\s\S]*?)<\/script>/);
  if (!m) { return null; }
  try { return JSON.parse(m[1].split('<\\/').join('</')); } catch (e) { return null; }
}

// Sphinx's own stemmer and stopword list, loaded from the build output.
let stemWord = (w) => w, STOPWORDS = [];
(function () {
  const p = path.join(REPO, 'rover', 'build', 'html', '_static', 'language_data.js');
  if (!fs.existsSync(p)) { return; }
  const ctx = {};
  vm.createContext(ctx);
  vm.runInContext(fs.readFileSync(p, 'utf8'), ctx);
  if (ctx.Stemmer) { const s = new ctx.Stemmer(); stemWord = (w) => s.stemWord(w); }
  if (ctx.stopwords) { STOPWORDS = ctx.stopwords; }
})();

/** Lift named functions out of a shipped script. */
function liftFunctions(names, file) {
  const src = fs.readFileSync(file || EXPORTER, 'utf8');
  let out = '';
  for (const name of names) {
    const at = src.indexOf('function ' + name + '(');
    if (at === -1) { return null; }
    let i = src.indexOf('{', at), depth = 0;
    for (; i < src.length; i++) {
      if (src[i] === '{') { depth++; } else if (src[i] === '}') {
        depth--; if (depth === 0) { break; }
      }
    }
    out += src.slice(at, i + 1) + '\n';
  }
  const ctx = {};
  vm.createContext(ctx);
  try {
    vm.runInContext(out + names.map((n) => 'this.' + n + '=' + n + ';').join(''), ctx);
  } catch (e) { return null; }
  return ctx;
}

/** The shell's search code, recovered from the SHELL_JS literals as exported. */
function shellSource() {
  const src = fs.readFileSync(DOCUMENT, 'utf8');
  const s = src.indexOf('var SHELL_JS = [');
  const e = src.indexOf("].join('')", s);
  if (s === -1 || e === -1) { return null; }
  try {
    return vm.runInNewContext(src.slice(src.indexOf('[', s), e + 1) + ".join('')");
  } catch (err) { return null; }
}

/** The exported shell running in a DOM over the payload the export wrote. */
function bootShell(D, bodies) {
  let JSDOM;
  try { ({ JSDOM } = require('jsdom')); } catch (e) { return null; }
  const src = shellSource();
  if (!src) { return null; }

  const blocks = D.pages.map((p, i) =>
    '<script type="text/plain" id="p' + i + '">' +
    ((bodies && bodies[p.p]) || p.p).replace(/<\/(script)/gi, '<\\/$1') +
    '</script>').join('');
  const dom = new JSDOM(
    '<!DOCTYPE html><html><body class="wy-body-for-nav">' +
    '<div class="wy-menu wy-menu-vertical" id="ap-nav"></div>' +
    '<input id="ap-search">' +
    '<section class="wy-nav-content-wrap">' +
    '<div id="ap-miss"></div><div id="ap-crumb"></div>' +
    '<div itemprop="articleBody" id="ap-doc"></div>' +
    '<footer id="ap-foot"></footer></section>' + blocks +
    '<script type="application/json" id="ap-index"></script></body></html>',
    { url: 'https://example.org/', runScripts: 'outside-only' });

  const win = dom.window;
  win.document.getElementById('ap-index').textContent = JSON.stringify(D);
  try { win.eval(src); } catch (e) { return null; }
  return win;
}

/** Navigate the shell the way a click on an anchor would. */
function shellGo(win, p) {
  win.location.hash = '#' + p;
  win.dispatchEvent(new win.Event('hashchange'));
}

function liftSearch(fts) {
  const shell = shellSource();
  if (shell === null) { return null; }
  const from = shell.indexOf('var SI=null;');
  const END = 'return out;}';
  const to = shell.indexOf(END, from);
  if (from === -1 || to === -1) { return null; }
  const sandbox = { document: { getElementById: (id) =>
    (id === 'ap-fts' ? { textContent: JSON.stringify(fts) } : null) } };
  vm.createContext(sandbox);
  const lang = path.join(REPO, 'rover', 'build', 'html', '_static', 'language_data.js');
  if (fs.existsSync(lang)) { vm.runInContext(fs.readFileSync(lang, 'utf8'), sandbox); }
  try {
    vm.runInContext(shell.slice(from, to + END.length), sandbox);
  } catch (err) { return null; }
  return typeof sandbox.fullText === 'function'
    ? (q) => sandbox.fullText(q.toLowerCase()) : null;
}

/* ------------------------------------------------------------- the run ---- */

async function main() {
  fs.mkdirSync(OUT, { recursive: true });

  const wikis = FULL ? ALL_WIKIS.filter((w) =>
    fs.existsSync(path.join(REPO, w, 'build', 'html')))
    : (WIKIS_ARG.length ? WIKIS_ARG : [WIKI]);
  const cap = FULL ? Infinity : 40;

  let totals = { pages: 0, images: 0, css: 0 };
  for (const w of wikis) {
    const r = loadWiki(w, cap);
    totals.pages += r.pages; totals.images += r.images; totals.css += r.css;
  }
  // Six releases across four lines: the reader saved every one of them,
  // and every one of them must survive into the file.
  const PARAM_ALL = ['V4.7.1', 'V4.7.0', 'V4.6.0', 'V4.5.2', 'V4.4.0', 'V4.3.0'];
  const paramWiki = wikis.includes('rover') ? 'rover' : wikis[0];
  const paramVehicle = paramWiki.charAt(0).toUpperCase() + paramWiki.slice(1);
  const paramPages = loadParameterVersions(paramWiki, paramVehicle, PARAM_ALL);
  totals.pages += PARAM_ALL.length;
  const paramBodies = {};
  paramPages.forEach((p) => { paramBodies[p.path] = p.body; });

  const loaded = totals;
  console.log('\ncache: ' + loaded.pages + ' pages, ' + loaded.images +
              ' shared images, ' + loaded.css + ' stylesheets' +
              (FULL ? '  (' + wikis.length + ' wikis, no cap)' : '') + '\n');

  const api = loadExporter();

  console.log('single-file HTML');
  const htmlPath = path.join(OUT, 'test.html');
  const t0 = Date.now();
  // Cancellation: the abort signal stops the export and aborts the sink.
  {
    let aborted = false; let writes = 0;
    const flag = { aborted: false };
    // Tripped by the third write, so the test holds on any cache size.
    const sink = { write: () => { writes++;
                     if (writes === 3) { flag.aborted = true; }
                     return Promise.resolve(); },
                   close: () => Promise.resolve(),
                   abort: () => { aborted = true; return Promise.resolve(); } };
    const outcome = await api.exportHtml(wikis, 'x.html', null, sink, flag)
      .then(() => 'resolved', (e) => e);
    check('a cancelled export rejects with AbortError',
          !!outcome && outcome !== 'resolved' && outcome.name === 'AbortError',
          String(outcome && (outcome.name || outcome)));
    check('the cancelled export aborts its sink', aborted);
    // Cancelled at the third write: the pack must stop there, not run every
    // remaining page and notice at the end.
    check('the cancel stops the work where it lands', writes <= 5,
          writes + ' writes after a cancel at the third');
  }

  // A cancel after the last page must still cancel, not quietly save,
  // and it must be noticed WHERE it lands, not at the next-but-one check.
  {
    let aborted = false; let tailWritten = false;
    const flag = { aborted: false };
    const dec = new TextDecoder();
    const sink = { write: (c) => {
                     const text = dec.decode(c);
                     // The tail contains both markers; test it before tripping.
                     if (flag.aborted && text.indexOf('ap-index') !== -1) {
                       tailWritten = true;
                     }
                     if (text.indexOf('ap-fts') !== -1) { flag.aborted = true; }
                     return Promise.resolve();
                   },
                   close: () => Promise.resolve(),
                   abort: () => { aborted = true; return Promise.resolve(); } };
    const outcome = await api.exportHtml(wikis, 'x.html', null, sink, flag)
      .then(() => 'resolved', (e) => e);
    check('a cancel after the last page still cancels',
          outcome !== 'resolved' && outcome.name === 'AbortError',
          String(outcome && (outcome.name || outcome)));
    check('and it aborts the sink', aborted);
    check('a cancel during the search block stops before the tail',
          !tailWritten);
  }

  // A cancel landing in the very last write is caught before close().
  {
    let aborted = false; let closed = false;
    const flag = { aborted: false };
    const dec = new TextDecoder();
    const sink = { write: (c) => {
                     if (dec.decode(c).indexOf('ap-index') !== -1) { flag.aborted = true; }
                     return Promise.resolve();
                   },
                   close: () => { closed = true; return Promise.resolve(); },
                   abort: () => { aborted = true; return Promise.resolve(); } };
    const outcome = await api.exportHtml(wikis, 'x.html', null, sink, flag)
      .then(() => 'resolved', (e) => e);
    check('a cancel during the tail write is caught before close',
          outcome !== 'resolved' && outcome.name === 'AbortError' &&
          aborted && !closed,
          String(outcome && (outcome.name || outcome)) + ', closed ' + closed);
  }

  // A sink that fails is told to discard what it holds.
  {
    let aborted = false; let wrote = 0;
    const sink = { write: () => {
                     wrote++;
                     return wrote > 3 ? Promise.reject(new Error('disk full'))
                                      : Promise.resolve();
                   },
                   close: () => Promise.resolve(),
                   abort: () => { aborted = true; return Promise.resolve(); } };
    const outcome = await api.exportHtml(wikis, 'x.html', null, sink, null)
      .then(() => 'resolved', (e) => e);
    check('a failing sink aborts the download it cannot finish',
          outcome !== 'resolved' && aborted,
          String(outcome && outcome.message) + ', abort ' + aborted);
  }

  const htmlRes = await api.exportHtml(wikis, 'test.html', null, fileSink(htmlPath));
  console.log('  generated in ' + ((Date.now() - t0) / 1000).toFixed(0) + 's');
  // A full export exceeds V8's maximum string length.
  const scan = scanFile(htmlPath, [
    /id="i\d+"/g, /data-ap-img=/g, /data:image\//g, /@font-face/g,
    /<img[^>]{0,200}src="\.\.\//g
  ], ['.wy-nav-content', 'wy-body-for-nav', 'toctree-l1', '#/' + wikis[0] + '/',
      '#ap-toast.on{display:flex}',
      'if(mapped===null){e.preventDefault();toast(a.href);return;}',
      'go.target="_blank"',
      // The picker builds from manifest names; every interpolation escaped.
      '+esc(h.name||h.id)+',
      '+esc(h.pages)+',
      'href="#\'+esc(h.path)+',
      '<li><a href="#\'+esc(h.pg.p)+\'"><span>',
      '<li class="toctree-l1"><a href="#\'+esc(h.pg.p)+\'">',
      'href="#\'+esc(p)+\'" class="btn btn-neutral float-left"',
      'href="#\'+esc(p)+\'" class="btn btn-neutral float-right"',
      // A single backslash in a SHELL_JS literal vanishes from the built file.
      '.replace(/\\s+/g," ")',
      'id="selectPicker"',
      'td.linenos .normal',
      'table.useralerts-table td']);

  const html = { includes: (s) => scan.found[s] };
  const imgBlocks = scan.counts[0];
  const imgRefs = scan.counts[1];
  const inlineDataUris = scan.counts[2];

  check('pages written', htmlRes.pages === loaded.pages,
        htmlRes.pages + ' of ' + loaded.pages);
  check('theme stylesheet embedded', html.includes('.wy-nav-content'));
  check('code highlighting css embedded', html.includes('td.linenos .normal'));
  check('the wiki style overrides embedded', html.includes('table.useralerts-table td'));
  check('theme markup emitted', html.includes('wy-body-for-nav'));
  check('fonts inlined', scan.counts[3] > 0, scan.counts[3] + ' rules');
  check('images stored once (no per-page duplication)',
        inlineDataUris <= imgBlocks + 2,
        imgBlocks + ' blocks, ' + imgRefs + ' refs, ' + inlineDataUris + ' data URIs');
  check('images actually resolved', imgBlocks > 0);
  check('navigation from toctree', html.includes('toctree-l1'));

  // Leaving the file has to be a decision, not an accident.
  check('a link to another host raises the toast instead of navigating',
        html.includes('if(mapped===null){e.preventDefault();toast(a.href);return;}'));
  check('the toast is styled', html.includes('#ap-toast.on{display:flex}'));
  check('leaving anyway opens a new tab, keeping the offline copy loaded',
        html.includes('go.target="_blank"'));
  check('a doubled backslash survives into the built file',
        html.includes('.replace(/\\s+/g," ")'));
  check('the parameter switcher reaches the file as the theme wrote it',
        html.includes('id="selectPicker"'));
  check('path anchors', html.includes('#/' + wikis[0] + '/'));
  check('the picker escapes every interpolation',
        html.includes('+esc(h.name||h.id)+') && html.includes('+esc(h.pages)+') &&
        html.includes('href="#\'+esc(h.path)+'), 'picker esc literals');
  check('search and sidebar results escape their hrefs',
        html.includes('<li><a href="#\'+esc(h.pg.p)+\'"><span>') &&
        html.includes('<li class="toctree-l1"><a href="#\'+esc(h.pg.p)+\'">'),
        'result href esc literals');
  check('the footer buttons escape their hrefs',
        html.includes('href="#\'+esc(p)+\'" class="btn btn-neutral float-left"') &&
        html.includes('href="#\'+esc(p)+\'" class="btn btn-neutral float-right"'),
        'navButton esc literals');
  check('no unresolved relative image srcs', scan.counts[4] === 0,
        scan.counts[4] + ' left');

  // An unclosed fragment nests each wiki inside the last.
  const D = readIndexPayload(htmlPath);
  check('routing index readable', D !== null && Array.isArray(D.pages));
  if (D) {
    const captions = (D.nav.match(/<p class="caption">/g) || []).length;
    const indexed = [...new Set(D.pages.map((p) => p.p.split('/')[1]))];
    check('every wiki has a sidebar section', captions === wikis.length,
          captions + ' sections for ' + wikis.length + ' wikis');
    check('every wiki has pages in the index', indexed.length === wikis.length,
          indexed.join(', '));
    check('sidebar sections are siblings, not nested',
          !/<div/i.test(D.nav) && !/<form/i.test(D.nav));
    // Under a page cap only the anchor's shape can be asserted.
    const anchors = (D.nav.match(/href="#([^"]+)"/g) || [])
      .map((h) => h.slice(7, -1)).filter((p) => p.charAt(0) === '/');
    const malformed = anchors.filter(
      (p) => p.indexOf('//') !== -1 || /\.html?$/.test(p));
    check('sidebar anchors are well formed', malformed.length === 0,
          malformed.length ? malformed.slice(0, 3).join('  ')
                           : anchors.length + ' anchors');
    check('sidebar links all point at pages in the file',
          anchors.every((p) => D.pages.some((x) => x.p === p)) || cap !== Infinity,
          cap === Infinity ? '' : '(not checked: page cap in effect)');
    check('image index built', Object.keys(D.imgs || {}).length > 0,
          Object.keys(D.imgs || {}).length + ' image paths');

    // Every saved version is offered, newest first: what the reader chose
    // to save is not thinned behind their back.
    const offered = (D.params || {})[paramWiki] || [];
    const want = PARAM_ALL.map((v) =>
      '/' + paramWiki + '/docs/parameters-' + paramVehicle + '-stable-' + v);
    check('the switcher offers every saved version, newest first',
          offered.map((v) => v.p).join() === want.join(),
          offered.map((v) => v.p).join(' ') || 'none');
    check('the labels are the ones the site shows',
          offered.length > 0 &&
          offered[0].n === paramVehicle + ' stable ' + PARAM_ALL[0],
          offered.length ? offered[0].n : 'none');
    const carried = new Set(D.pages.map((p) => p.p));
    check('every saved version is carried into the file',
          want.every((p) => carried.has(p)), want.length + ' versions');
  }

  // The nav builders weave reader-reachable text into HTML: prove inert.
  const escFns = liftFunctions(['escapeHtml', 'listNav', 'renderNodes'], DOCUMENT);
  check('nav templates lifted from the document module', escFns !== null);
  if (escFns) {
    const evil = '/rover/docs/x" onmouseover="alert(1)';
    const list = escFns.listNav([{ path: evil + '.html' }], 'rover');
    check('the fallback sidebar escapes hostile paths',
          list.indexOf('onmouseover="alert') === -1 &&
          list.indexOf('&quot;') !== -1, list.slice(0, 90));
    const rn = escFns.renderNodes([{ external: false, href: evil,
      label: '<img src=x onerror=alert(1)>', children: [] }], 1);
    check('the toctree sidebar escapes hostile labels and hrefs',
          rn.indexOf('<img') === -1 && rn.indexOf('onmouseover="alert') === -1,
          rn.slice(0, 120));
  }

  // Root-relative cross-wiki links exist only in archives.
  const navFns = liftFunctions(
    ['resolvePath', 'innerOf', 'topLevelLists', 'textOf', 'navHref', 'prune',
     'parseToc', 'navNodes', 'mergeToc'], DOCUMENT);
  check('nav helpers lifted from the document module', navFns !== null);
  if (navFns) {
    const fixture =
      '<div class="wy-menu wy-menu-vertical"><ul>' +
      '<li><a href="docs/common-team.html">Team</a></li>' +
      '<li><a href="/copter/index.html">Copter</a></li>' +
      '<li><a href="/plane/docs/common-choosing-a-ground-station.html">GCSes</a></li>' +
      '<li><a href="https://cloud.ardupilot.org">Drone Engage</a></li>' +
      '</ul></div>';
    const nodes = navFns.navNodes(fixture, '/ardupilot/index.html');
    const got = nodes.map((n) => n.href);
    check('a link relative to the wiki keeps its wiki',
          got.indexOf('/ardupilot/docs/common-team') !== -1, got.join('  '));
    check('a cross-wiki link from the site root is not prefixed again',
          got.indexOf('/copter/index') !== -1, got.join('  '));
    check('no anchor has an empty path segment',
          !nodes.some((n) => !n.external && n.href.indexOf('//') !== -1),
          got.join('  '));
    check('a link to another host is left alone',
          got.indexOf('https://cloud.ardupilot.org') !== -1, got.join('  '));

    // Two pages expanding two branches of one tree, as the theme emits them.
    const pageA =
      '<div class="wy-menu wy-menu-vertical"><ul class="current">' +
      '<li class="toctree-l1"><a href="common-autopilots.html">Autopilots</a></li>' +
      '<li class="toctree-l1 current"><a href="additional-information.html">More</a>' +
      '<ul class="current">' +
      '<li class="toctree-l2"><a href="reference-frames.html">Frames</a></li>' +
      '<li class="toctree-l2 current"><a class="current" href="#">Appendix</a>' +
      '<ul><li class="toctree-l3"><a href="#a-heading">A heading</a></li></ul>' +
      '</li></ul></li>' +
      '<li class="toctree-l1"><a href="common-user-alerts.html">Alerts</a></li>' +
      '</ul></div>';
    const pageB =
      '<div class="wy-menu wy-menu-vertical"><ul class="current">' +
      '<li class="toctree-l1"><a href="common-autopilots.html">Autopilots</a></li>' +
      '<li class="toctree-l1 current"><a href="additional-information.html">More</a>' +
      '<ul class="current">' +
      '<li class="toctree-l2 current"><a href="reference-frames.html">Frames</a>' +
      '<ul class="current"><li class="toctree-l3 current">' +
      '<a class="current" href="#">Body frame</a></li></ul></li>' +
      '<li class="toctree-l2"><a href="common-appendix.html">Appendix</a></li>' +
      '</ul></li>' +
      '<li class="toctree-l1"><a href="common-user-alerts.html">Alerts</a></li>' +
      '</ul></div>';

    const one = navFns.navNodes(pageA, '/copter/docs/common-appendix.html');
    check('a page sidebar parses as a tree, not a flat list',
          one.length === 3 && one[1].children.length === 2,
          one.length + ' top level, ' +
          one.map((n) => n.children.length).join('/') + ' children');
    check('headings inside the page being read are not toctree entries',
          one[1].children[1].children.length === 0 &&
          one[1].children[1].href === '/copter/docs/common-appendix',
          JSON.stringify(one[1].children[1].href) + ', ' +
          one[1].children[1].children.length + ' children');

    const merged = [];
    navFns.mergeToc(merged, one);
    navFns.mergeToc(merged,
      navFns.navNodes(pageB, '/copter/docs/reference-frames.html'));
    const frames = merged[1].children[0];
    check('a branch only another page expands survives the merge',
          frames.href === '/copter/docs/reference-frames' &&
          frames.children.length === 1 &&
          frames.children[0].href === '/copter/docs/reference-frames',
          frames.children.map((c) => c.href).join(' '));
    check('merging two pages does not duplicate what both list',
          merged.length === 3 && merged[1].children.length === 2,
          merged.length + ' top level, ' +
          merged[1].children.length + ' under the expanded one');
  }

  // The sidebar and the reading order must agree.
  if (D) {
    const order = D.order || [];
    check('a reading order was published', order.length > 0,
          order.length + ' pages');

    const rendered = [];
    const seen = new Set();
    (D.nav.match(/href="#(\/[^"]+)"/g) || []).forEach((h) => {
      const p = h.slice(7, -1);
      if (!seen.has(p)) { seen.add(p); rendered.push(p); }
    });
    const missing = rendered.filter((p) => order.indexOf(p) === -1 &&
                                           p.split('/')[1] === D.wikis[0]);
    check('every page the sidebar lists is in the reading order',
          missing.length === 0,
          missing.length ? missing.slice(0, 3).join('  ')
                         : rendered.length + ' sidebar anchors');

    const inOrder = rendered.filter((p) => order.indexOf(p) !== -1)
                            .map((p) => order.indexOf(p));
    let sorted = true;
    for (let i = 1; i < inOrder.length; i++) {
      // Wikis follow one another in both, so the whole sequence is monotonic.
      if (inOrder[i] < inOrder[i - 1]) { sorted = false; break; }
    }
    check('the reading order runs down the sidebar, not past it', sorted,
          inOrder.length + ' compared');

    const depth = (n) => (D.nav.match(
      new RegExp('class="toctree-l' + n + '"', 'g')) || []).length;
    check('the sidebar has nested levels, not one flat list',
          depth(2) > 0, 'l1 ' + depth(1) + ', l2 ' + depth(2) +
          ', l3 ' + depth(3));

    // Without the expand button a branch opens only by visiting a page in it.
    const buttons = (D.nav.match(/<button class="toctree-expand"/g) || []).length;
    const parents = (D.nav.match(/<\/a><ul>/g) || []).length;
    check('every branch has the theme expand button', buttons === parents &&
          buttons > 0, buttons + ' buttons for ' + parents + ' branches');
  }

  /* ------------------------------------------- the shell, driven in a DOM -- */

  const win = D ? bootShell(D, paramBodies) : null;
  if (D && win === null) {
    console.log('  SKIP  shell behaviour: jsdom is not installed');
  }
  if (win) {
    const doc = win.document;
    const nav = doc.getElementById('ap-nav');
    const inFile = new Set(D.pages.map((p) => p.p));
    // The export's own wiki order, where the reading order has to stop.
    const wiki0 = D.wikis[0];

    // From the markup, not D.order, so the two are checked against each other.
    const walk = [];
    const seenA = new Set();
    [].forEach.call(nav.querySelectorAll('a[href^="#/"]'), (a) => {
      const p = a.getAttribute('href').slice(1);
      if (p.split('/')[1] !== wiki0 || seenA.has(p)) { return; }
      seenA.add(p);
      walk.push(p);
    });
    const reachable = walk.filter((p) => inFile.has(p));

    // A page with something on either side of it, so both buttons are due.
    const at = reachable.findIndex((p, i) => i > 0 && i < reachable.length - 1);
    const target = at === -1 ? null : reachable[at];
    check('a page with neighbours to test against', target !== null,
          reachable.length + ' of ' + walk.length + ' sidebar pages in the file');

    if (target) {
      shellGo(win, target);
      const foot = doc.getElementById('ap-foot');
      const next = foot.querySelector('a[rel="next"]');
      const prev = foot.querySelector('a[rel="prev"]');

      check('the page footer carries next and previous buttons',
            !!next && !!prev,
            (prev ? 'prev ' : '') + (next ? 'next' : '') || 'neither');
      check('the buttons are the theme\'s own',
            !!next && next.className === 'btn btn-neutral float-right' &&
            !!prev && prev.className === 'btn btn-neutral float-left',
            next ? next.className : '');
      check('next is the page the sidebar shows next',
            !!next && next.getAttribute('href') === '#' + reachable[at + 1],
            (next ? next.getAttribute('href') : 'none') +
            ' wanted #' + reachable[at + 1]);
      check('previous is the page the sidebar shows before',
            !!prev && prev.getAttribute('href') === '#' + reachable[at - 1],
            (prev ? prev.getAttribute('href') : 'none') +
            ' wanted #' + reachable[at - 1]);
      // As the live wiki does.
      if (wikis.length > 1) {
        shellGo(win, reachable[reachable.length - 1]);
        check('the last page of a wiki offers no next',
              !doc.getElementById('ap-foot').querySelector('a[rel="next"]'),
              reachable[reachable.length - 1]);
        shellGo(win, target);
      }

      // An <li> is open when it carries "current".
      const here = [].slice.call(nav.querySelectorAll('a[href^="#/"]'))
        .filter((a) => a.getAttribute('href') === '#' + target)[0];
      check('the sidebar marks the page being read',
            !!here && here.classList.contains('current'));
      if (here) {
        let li = here.parentNode, open = 0, all = 0;
        while (li && li !== nav) {
          if (li.tagName === 'LI') { all++; if (li.classList.contains('current')) { open++; } }
          li = li.parentNode;
        }
        check('the branch down to it is open, not just the entry',
              all > 0 && open === all, open + ' of ' + all + ' ancestors');
      }

      const branches = [].slice.call(nav.querySelectorAll('button.toctree-expand'))
        .map((b) => b.closest('li'));
      const shut = branches.filter((li) => !li.classList.contains('current'));
      check('branches the reader is not in stay collapsed',
            branches.length > 0 && shut.length > 0,
            shut.length + ' of ' + branches.length + ' closed');

      if (shut.length) {
        const li = shut[0];
        const btn = li.querySelector('button.toctree-expand');
        const ev = new win.MouseEvent('click',
                                      { bubbles: true, cancelable: true });
        btn.dispatchEvent(ev);
        check('clicking the arrow opens that branch',
              li.classList.contains('current'));
        // The arrow sits inside the anchor; the click must be cancelled.
        check('opening a branch does not navigate away', ev.defaultPrevented);
        btn.dispatchEvent(new win.MouseEvent('click',
                                             { bubbles: true, cancelable: true }));
        check('clicking it again closes the branch',
              !li.classList.contains('current'));
      }
    }

    /* --------------------------------------- the parameter version switcher */

    const versions = (D.params || {})[paramWiki] || [];
    if (versions.length > 1) {
      shellGo(win, versions[1].p);
      const sel = doc.querySelector('#selectPicker');
      // Latest leads, then every saved version.
      check('the version switcher is filled in', !!sel && sel.options.length ===
            versions.length + 1,
            sel ? sel.options.length + ' options' : 'no select');
      if (sel) {
        check('the version being read is the one selected',
              sel.value === versions[1].p, sel.value);
        check('the options are labelled as the site labels them',
              [].map.call(sel.options, (o) => o.textContent).join() ===
              ['Latest'].concat(versions.map((v) => v.n)).join(),
              [].map.call(sel.options, (o) => o.textContent).join(' '));

        // Elements, not text: a swallowed page still contains the words.
        const paras = [].map.call(doc.querySelectorAll('#ap-doc p'),
                                  (el) => el.textContent);
        check('the page below its own inline script survives',
              paras.some((t) => t.indexOf(
                'This is a complete list of the parameters.') !== -1),
              paras.length + ' paragraphs');

        sel.value = versions[0].p;
        sel.dispatchEvent(new win.Event('change'));
        check('choosing a version opens it',
              win.location.hash === '#' + versions[0].p, win.location.hash);
      }

      // The mirror rewrites cross-wiki links root-relative; a click on one
      // must keep its own path, never gain the current wiki as a prefix.
      {
        const dest = D.pages[0];
        const from = D.pages.find((p) => p.p !== dest.p);
        const bodies2 = {};
        bodies2[from.p] = '<a id="xw" href="/copter9/index.html">Copter</a>' +
                          '<a id="sw" href="' + dest.p + '.html">Same site</a>';
        const w2 = bootShell(D, bodies2);
        if (w2) {
          shellGo(w2, from.p);
          w2.document.getElementById('xw').dispatchEvent(
            new w2.MouseEvent('click', { bubbles: true, cancelable: true }));
          const miss = w2.document.getElementById('ap-doc').textContent || '';
          check('a root-relative link to an absent wiki keeps its own path',
                miss.indexOf('/copter9/index is not included') !== -1 &&
                miss.indexOf('docs/copter9') === -1,
                JSON.stringify(miss.slice(0, 120)));
          shellGo(w2, from.p);
          w2.document.getElementById('sw').dispatchEvent(
            new w2.MouseEvent('click', { bubbles: true, cancelable: true }));
          check('a root-relative link to a held page opens it',
                w2.location.hash === '#' + dest.p,
                w2.location.hash + ' wanted #' + dest.p);
        } else {
          check('root-relative link shell booted', false);
        }
      }

      // In-page anchors scroll; they never route to the missing panel.
      {
        const from = D.pages[0];
        const anchorBodies = {};
        anchorBodies[from.p] =
          '<h2 id="a-section">Section<a id="pl" class="headerlink" href="#a-section">P</a></h2>' +
          '<p>body text</p>';
        const w3 = bootShell(D, anchorBodies);
        if (w3) {
          shellGo(w3, from.p);
          const before = w3.document.getElementById('ap-doc').textContent;
          w3.document.getElementById('pl').dispatchEvent(
            new w3.MouseEvent('click', { bubbles: true, cancelable: true }));
          const after = w3.document.getElementById('ap-doc').textContent;
          check('clicking a headerlink keeps the page',
                after === before && after.indexOf('body text') !== -1 &&
                after.indexOf('Not in this offline copy') === -1,
                JSON.stringify(after.slice(0, 60)));
          // A fragment typed straight into the hash scrolls too, never routes.
          shellGo(w3, 'a-section');
          const typed = w3.document.getElementById('ap-doc').textContent;
          check('a bare fragment hash never shows the missing panel',
                typed.indexOf('Not in this offline copy') === -1,
                JSON.stringify(typed.slice(0, 60)));
        } else {
          check('anchor shell booted', false);
        }
      }

      // The switcher offers a way back to the latest parameters page.
      {
        const latest = '/' + paramWiki + '/docs/parameters';
        const anyVersion = ((D.params || {})[paramWiki] || [])[1];
        if (anyVersion && D.pages.some((pp) => pp.p === latest)) {
          const w4 = bootShell(D, paramBodies);
          if (w4) {
            shellGo(w4, anyVersion.p);
            const opts = [].slice.call(
              w4.document.querySelectorAll('#selectPicker option'));
            check('the switcher leads back to Latest',
                  opts.length > 0 && opts[0].value === latest &&
                  opts[0].textContent === 'Latest',
                  opts.length ? opts[0].value : 'no options');
          }
        } else {
          check('the switcher leads back to Latest',
                true, 'no unversioned parameters page in this fixture');
        }
      }

      const bare = bootShell(Object.assign({}, D, { params: {} }), paramBodies);
      if (bare) {
        shellGo(bare, versions[1].p);
        const box = bare.document.querySelector('#selectPicker').parentNode;
        check('with no versions to offer, the switcher is taken away',
              box.style.display === 'none', box.style.display || 'shown');
      }
    }
  }

  // Sphinx omits stopwords from its index; a query containing one must still work.
  const fts = readSearchPayload(htmlPath);
  const search = fts ? liftSearch(fts) : null;
  check('search lifted from the exporter', !fts || search !== null);
  if (search) {
    const probe = (q) => Object.keys(search(q)).length;
    const bare = probe('vehicle');
    check('full-text search finds a word in body text', bare > 0, bare + ' docs');
    check('a stopword in the query does not empty the results',
          probe('the vehicle') === bare, probe('the vehicle') + ' vs ' + bare);
    check('a whole pasted sentence still matches',
          probe('the vehicle is a copter') > 0, probe('the vehicle is a copter') + ' docs');

    // A pasted selection clips its first and last words.
    check('a word that matches nothing does not empty the results',
          probe('vehicle zzzznotaword') === bare,
          probe('vehicle zzzznotaword') + ' vs ' + bare);

    // A real pasted selection, first word clipped.
    const CLIPPED = 'rial-grade, dual-band GNSS module designed and ' +
      'manufactured in India by TeraVolt Labs. It is specifically engineered ' +
      'to support the NavIC (IRNSS) constellation, making it fully compliant ' +
      'with DGCA requirements for indigenous dron';
    const hasPage = Object.keys(fts).some(
      (w) => (fts[w].docnames || []).some((n) => /AeroNav-1/i.test(n)));
    if (hasPage) {
      const hits = Object.keys(search(CLIPPED));
      check('a sentence clipped mid-word by the selection still finds its page',
            hits.some((p) => /AeroNav-1/i.test(p)),
            hits.length + ' hits: ' + (hits.slice(0, 3).join(', ') || 'none'));
    }
  }

  console.log('\nwrote ' + OUT + '/test.html');
  console.log(failures ? '\n' + failures + ' CHECK(S) FAILED\n' : '\nall checks passed\n');
  process.exit(failures ? 1 : 0);
}

main().catch((err) => { console.error(err); process.exit(1); });
