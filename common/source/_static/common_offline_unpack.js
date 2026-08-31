/*
 * [copywiki destination="copter,plane,rover,sub,blimp,antennatracker,dev,planner,planner2,ardupilot,mavproxy"]
 *
 * Downloads one wiki archive and unpacks it into Cache Storage. The archive is
 * a tar served as a gzip content coding, so the browser decompresses it; the
 * tar is walked as a stream and each entry is stored under the URL the site
 * serves it at, text entries gzipped. Exposes window.ApUnpack.
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

  /** Write one entry, gzipped when that helps; any failure stores plain bytes. */
  function storeEntry(cache, path, entryName, body) {
    var type = mimeFor(entryName);
    var plain = function () {
      return cache.put(new Request(path),
        new Response(body, { headers: { 'Content-Type': type } }));
    };
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

  /** cache.match, but readable. */
  function readFrom(cache, path) {
    return cache.match(path).then(inflate);
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
    readFrom: readFrom,
    storeEntry: storeEntry
  };
})(typeof self !== 'undefined' ? self : this);
