/*
 * [copywiki destination="copter,plane,rover,sub,blimp,antennatracker,dev,planner,planner2,ardupilot,mavproxy"]
 *
 * Downloads one wiki archive and unpacks it into Cache Storage. The archive is
 * a tar served as a gzip content coding, so the browser decompresses it; the
 * tar is walked as a stream and each entry is stored under the URL the site
 * serves it at, text entries gzipped. Historical parameter pages arrive as
 * zstd deltas against the base page beside them and are stored that way;
 * readFrom rebuilds one with the decoder in zstd-delta.js. Exposes
 * window.ApUnpack.
 */
(function (global) {
  'use strict';

  var MIME = {
    html: 'text/html; charset=utf-8', js: 'text/javascript', css: 'text/css',
    json: 'application/json', png: 'image/png', jpg: 'image/jpeg',
    jpeg: 'image/jpeg', gif: 'image/gif', svg: 'image/svg+xml',
    webp: 'image/webp', bmp: 'image/bmp', ico: 'image/x-icon', woff: 'font/woff',
    woff2: 'font/woff2', ttf: 'font/ttf', inv: 'application/octet-stream'
  };

  function mimeFor(name) {
    var ext = name.split('.').pop().toLowerCase();
    return MIME[ext] || 'application/octet-stream';
  }

  function textField(bytes, offset, length) {
    var out = '';
    for (var i = offset; i < offset + length; i++) {
      if (bytes[i] === 0) { break; }
      out += String.fromCharCode(bytes[i]);
    }
    return out;
  }

  // Minimal tar reader: 512-byte headers, data padded to 512. `pathFor` maps
  // an entry name to the exact cache key; storing anything else would let a
  // name the guard approved land somewhere it did not.
  // No page or image comes close; a hostile archive must not balloon memory.
  var MAX_ENTRY = 64 * 1024 * 1024;

  function untarToCache(stream, cache, pathFor, onEntry) {
    var reader = stream.getReader();
    var chunks = [];
    var buffered = 0;
    var done = false;

    function pull() {
      return reader.read().then(function (r) {
        if (r.done) { done = true; return; }
        chunks.push(r.value);
        buffered += r.value.length;
      });
    }

    function need(n) {
      if (buffered >= n || done) { return Promise.resolve(buffered >= n); }
      return pull().then(function () { return need(n); });
    }

    // One allocation per read, never a rolling copy of the whole tail.
    function take(n) {
      var out = new Uint8Array(n);
      var off = 0;
      while (off < n) {
        var head = chunks[0];
        var want = n - off;
        if (head.length <= want) {
          out.set(head, off);
          off += head.length;
          chunks.shift();
        } else {
          out.set(head.subarray(0, want), off);
          chunks[0] = head.subarray(want);
          off = n;
        }
      }
      buffered -= n;
      return out;
    }

    // A PAX or GNU long-name header names the NEXT entry.
    var override = null;
    // Set by the zero blocks that close a tar; a stream ending without them was cut short.
    var sawEnd = false;

    // "path=<value>" from a PAX extended header body.
    function paxPath(body) {
      var text = '';
      for (var i = 0; i < body.length; i++) { text += String.fromCharCode(body[i]); }
      var m = text.match(/\d+ path=([^\n]*)\n/);
      return m ? m[1] : null;
    }

    function step() {
      return need(512).then(function (ok) {
        if (!ok) {
          if (buffered || !sawEnd) { throw new Error('archive truncated'); }
          return;
        }
        var header = take(512);
        var name = textField(header, 0, 100);
        if (!name) { sawEnd = true; return step(); }   // zero block: end of archive

        // ustar prefix field; a PAX/GNU override wins over both.
        var pfx = textField(header, 345, 155);
        if (pfx) { name = pfx + '/' + name; }

        var size = parseInt(textField(header, 124, 12).trim(), 8) || 0;
        if (size > MAX_ENTRY) {
          throw new Error('archive entry too large: ' + name);
        }
        var type = String.fromCharCode(header[156] || 48);
        var padded = Math.ceil(size / 512) * 512;

        return need(padded).then(function (haveBody) {
          if (!haveBody) { throw new Error('archive truncated in ' + name); }
          var body = take(padded).slice(0, size);

          // Names the NEXT entry: capture it and read on.
          if (type === 'x' || type === 'g') {
            var p = paxPath(body);
            if (p) { override = p; }
            return step();
          }
          if (type === 'L') {
            override = textField(body, 0, body.length);
            return step();
          }

          // '0' and NUL are regular files; skip directories and other metadata.
          if (type !== '0' && type !== '\0') { override = null; return step(); }

          var entryName = override || name;
          override = null;
          var path = pathFor(entryName);
          return storeEntry(cache, path, entryName, body).then(function () {
            if (onEntry) {
              // Awaited, so a hashing onEntry finishes before the next read.
              return Promise.resolve(onEntry(path, entryName, body)).then(step);
            }
            return step();
          });
        });
      });
    }

    return step();
  }


  // Text is stored gzipped (455 MB -> 57 MB); AP_ENCODED marks those entries.
  var AP_ENCODED = 'x-ap-encoding';
  var COMPRESSIBLE = /\.(html?|js|mjs|css|json|svg|xml|txt|inv|map)$/i;

  function canCompress() {
    return typeof CompressionStream === 'function';
  }

  function gzip(bytes) {
    var stream = new Response(bytes).body
      .pipeThrough(new CompressionStream('gzip'));
    return new Response(stream).arrayBuffer();
  }

  // A delta entry opens with this, the base page's filename, the content
  // hash of the page it rebuilds and a newline, then a zstd frame whose
  // dictionary is that base, stored beside it. The hash is checked after
  // rebuilding, so neither decoder can hand back a wrong page unnoticed.
  var DELTA_MAGIC = 'APDELTA1 ';
  var AP_DELTA = 'zstd-delta';
  var WASM_URL = '/js/zstd.wasm';

  /** { base, hash, frame } for a delta entry's bytes, or null for anything else. */
  function deltaHeader(bytes) {
    for (var i = 0; i < DELTA_MAGIC.length; i++) {
      if (bytes[i] !== DELTA_MAGIC.charCodeAt(i)) { return null; }
    }
    var end = bytes.indexOf(10, DELTA_MAGIC.length);
    if (end === -1 || end > DELTA_MAGIC.length + 220) { return null; }
    var fields = textField(bytes, DELTA_MAGIC.length, end - DELTA_MAGIC.length).split(' ');
    var base = fields[0], hash = fields[1];
    // A bare filename: the base is the page next door, never anywhere else.
    if (!base || /[\/\\]/.test(base) || base === '.' || base === '..') { return null; }
    if (fields.length > 2) { return null; }
    // The hash is what proves a rebuilt page; a delta without one is still a
    // delta (never served as a page) but can only be refused.
    var ok = /^[0-9a-f]{16}$/.test(hash || '');
    return { base: base, hash: ok ? hash : null, frame: bytes.subarray(end + 1) };
  }

  // Exactly as the build computes it: sha256, first eight bytes, hex.
  function contentHash(bytes) {
    return crypto.subtle.digest('SHA-256', bytes).then(function (d) {
      var v = new Uint8Array(d);
      var out = '';
      for (var i = 0; i < 8; i++) { out += (v[i] < 16 ? '0' : '') + v[i].toString(16); }
      return out;
    });
  }

  /** Write one entry, gzipped when that helps; any failure stores plain bytes. */
  function storeEntry(cache, path, entryName, body) {
    var type = mimeFor(entryName);
    var plain = function () {
      return cache.put(new Request(path),
        new Response(body, { headers: { 'Content-Type': type } }));
    };
    if (deltaHeader(body)) {
      var marked = { 'Content-Type': type };
      marked[AP_ENCODED] = AP_DELTA;
      return cache.put(new Request(path), new Response(body, { headers: marked }));
    }
    if (!canCompress() || !COMPRESSIBLE.test(entryName) || body.length < 1024) {
      return plain();
    }
    return gzip(body).then(function (packed) {
      if (!packed || packed.byteLength >= body.length) { return plain(); }
      var headers = { 'Content-Type': type };
      headers[AP_ENCODED] = 'gzip';
      return cache.put(new Request(path), new Response(packed, { headers: headers }));
    }).catch(plain);
  }


  // Every reader of these caches comes through here: raw gzip is silent mojibake.
  function inflate(response) {
    if (!response || !response.headers ||
        response.headers.get(AP_ENCODED) !== 'gzip') {
      return response;
    }
    if (typeof DecompressionStream !== 'function') { return undefined; }
    var headers = new Headers(response.headers);
    headers.delete(AP_ENCODED);
    return new Response(
      response.body.pipeThrough(new DecompressionStream('gzip')),
      { status: 200, statusText: 'OK', headers: headers }
    );
  }

  // Kept in step with FOLD_INTO_COMMON (build) and FOLDED_INTO_COMMON
  // (sw.js, page): the wikis whose pages the common archive carries.
  var FOLDED_INTO_COMMON = ['ardupilot'];

  // The trees an archive may write into. Anything else in a common archive
  // is refused; a wiki archive may only fill its own tree.
  function allowedPrefixes(id) {
    if (id !== 'common') { return ['/' + id + '/']; }
    return ['/_common/_images/'].concat(FOLDED_INTO_COMMON.map(function (w) {
      return '/' + w + '/';
    }));
  }

  // Where an entry is stored; shared with the differential update. Names come
  // off the network, so the URL the cache will really use, with encoded dots
  // decoded, tabs stripped and backslashes turned into slashes, must stay
  // inside the archive's own tree. The parser decides that, not this code.
  function cachePathFor(id, name) {
    var path = (id === 'common' && name.indexOf('_images/') === 0)
      ? '/_common/' + name : '/' + name;
    var url = new URL(path, 'https://a');
    var inTree = url.origin === 'https://a' && !url.search && !url.hash &&
      allowedPrefixes(id).some(function (prefix) {
        return url.pathname.indexOf(prefix) === 0;
      });
    if (!inTree) { throw new Error('unsafe archive path ' + name); }
    return url.pathname;
  }

  // The decoder, initialised once; a failed load is forgotten so the next
  // read can try again.
  var zstdReady = null;

  function fetchWasm() {
    return fetch(WASM_URL).then(function (r) {
      if (!r.ok) { throw new Error('could not fetch ' + WASM_URL + ' (' + r.status + ')'); }
      return r.arrayBuffer();
    });
  }

  // The wasm when it can be had, the JavaScript decoder when it cannot.
  function decoder(loadWasm) {
    if (!zstdReady) {
      zstdReady = Promise.resolve().then(function () {
        var zstd = global.ApZstd;
        if (!zstd) { throw new Error('zstd-delta.js is not loaded'); }
        return (loadWasm || fetchWasm)().then(function (buf) {
          return zstd.init(buf);
        }, function (err) {
          console.warn('[offline] no wasm for the delta decoder, using JavaScript:', err && err.message);
          return zstd.init(null);
        }).then(function () { return zstd; });
      });
      zstdReady.catch(function () { zstdReady = null; });
    }
    return zstdReady;
  }

  /** The stored response as its page: inflated, or rebuilt from its delta
   * and the base page in the same cache. opts.loadWasm supplies the decoder's
   * bytes when a plain fetch would not do. */
  // A delta is self-describing, so a versioned parameter page is read and
  // sniffed for the magic even without the marker: an entry stored by older
  // code, or by anything that put the bytes in plain, still rebuilds.
  var PARAM_VERSION_PATH = /\/docs\/parameters-[^/]+\.html$/;

  function restore(response, cache, path, opts) {
    if (!response || !response.headers) { return Promise.resolve(response); }
    var marked = response.headers.get(AP_ENCODED) === AP_DELTA;
    if (!marked && !PARAM_VERSION_PATH.test(path)) {
      return Promise.resolve(inflate(response));
    }
    var plain = marked ? response : inflate(response);
    if (!plain) { return Promise.resolve(undefined); }
    return plain.arrayBuffer().then(function (buf) {
      var bytes = new Uint8Array(buf);
      var head = deltaHeader(bytes);
      if (!head) {
        if (marked) { throw new Error('malformed delta entry ' + path); }
        // A real page after all.
        return new Response(bytes, {
          status: 200, statusText: 'OK',
          headers: { 'Content-Type': mimeFor(path) }
        });
      }
      if (!head.hash) { throw new Error('delta entry ' + path + ' carries no hash to check it against'); }
      var basePath = path.slice(0, path.lastIndexOf('/') + 1) + head.base;
      return Promise.all([
        readFrom(cache, basePath, opts),
        decoder(opts && opts.loadWasm)
      ]).then(function (r) {
        if (!r[0]) { throw new Error('base page ' + basePath + ' missing for ' + path); }
        return r[0].arrayBuffer().then(function (base) {
          var page = r[1].patch(head.frame, new Uint8Array(base));
          return contentHash(page).then(function (got) {
            if (got !== head.hash) {
              throw new Error('the rebuilt page for ' + path + ' does not match its hash');
            }
            return new Response(page, {
              status: 200, statusText: 'OK',
              headers: { 'Content-Type': mimeFor(path) }
            });
          });
        });
      });
    });
  }

  /** cache.match, but readable. */
  function readFrom(cache, path, opts) {
    return cache.match(path).then(function (hit) {
      return restore(hit, cache, path, opts);
    });
  }

  /** Fetch one archive and unpack it into `cache`. opts: base, build, signal. */
  function fetchArchive(entry, cache, onBytes, opts) {
    opts = opts || {};
    // Tagged with the build id so a CDN never serves the previous build.
    var url = opts.base + '/' + (entry.archive || entry.id + '-offline.tar.gz') +
              (opts.build ? '?v=' + encodeURIComponent(opts.build) : '');
    return fetch(url, { mode: 'cors', signal: opts.signal }).then(function (response) {
      if (!response.ok) {
        throw new Error('could not fetch ' + entry.name + ' (' + response.status + ')');
      }
      if (!response.body) {
        throw new Error('this browser cannot stream the download');
      }

      var counter = new TransformStream({
        transform: function (chunk, controller) {
          onBytes(chunk.byteLength);
          controller.enqueue(chunk);
        }
      });

      // Counted after the browser decompressed, so compare with raw_bytes.
      var stream = response.body.pipeThrough(counter);

      // Resolves with the entries, hashed when asked, so the caller can
      // check them off against the published table by name and by content.
      var names = [];
      return untarToCache(stream, cache, function (entryName) {
        return cachePathFor(entry.id, entryName);
      }, function (_path, entryName, body) {
        if (!opts.hash) { names.push({ name: entryName }); return undefined; }
        return opts.hash(body).then(function (digest) {
          names.push({ name: entryName, hash: digest });
        });
      }).then(function () { return names; });
    });
  }

  global.ApUnpack = {
    cachePathFor: cachePathFor,
    mimeFor: mimeFor,
    untarToCache: untarToCache,
    fetchArchive: fetchArchive,
    inflate: inflate,
    restore: restore,
    deltaHeader: deltaHeader,
    contentHash: contentHash,
    readFrom: readFrom,
    storeEntry: storeEntry
  };
})(typeof self !== 'undefined' ? self : this);
