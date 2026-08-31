/*
 * [copywiki destination="copter,plane,rover,sub,blimp,antennatracker,dev,planner,planner2,ardupilot,mavproxy"]
 *
 * Updates a saved wiki in place. The build publishes a table of path -> hash
 * per archive; the stored table is compared with the published one and only
 * the changed files are fetched, each verified against its hash before it is
 * written. Too large a difference returns null and the panel re-downloads the
 * archive. Exposes window.ApUpdate.
 */
(function (global) {
  'use strict';

  var TABLE_KEY = '/__ap_files__';

  // Paced so an update reads as a browser, not a crawler.
  var MIN_REQUEST_GAP_MS = 250;

  // Above these, re-download the archive instead. A template edit changes
  // every page but no images, 40% to 85% of a wiki's entries, and must stay
  // differential: about 20 MB of pages against a 74 MB archive for Copter.
  // Past either cap one archive request beats hundreds of file fetches.
  var MAX_DIFF_FILES = 400;
  var MAX_DIFF_FRACTION = 0.5;
  // Below this a proportion says nothing; only the absolute cap applies.
  var DIFF_FRACTION_MIN_FILES = 50;

  // Unpaced sequential fetches ran at about 75 a second from one browser.
  var UPDATE_RATE_PER_SEC = 15;
  var lastFetchAt = 0;

  function pace() {
    var gap = 1000 / UPDATE_RATE_PER_SEC;
    var wait = Math.max(0, lastFetchAt + gap - Date.now());
    lastFetchAt = Date.now() + wait;
    return wait ? new Promise(function (r) { setTimeout(r, wait); })
                : Promise.resolve();
  }

  // Exactly as the build computes it: sha256, first eight bytes, hex.
  function hashBytes(buf) {
    return crypto.subtle.digest('SHA-256', buf).then(function (d) {
      var v = new Uint8Array(d);
      var out = '';
      for (var i = 0; i < 8; i++) {
        out += (v[i] < 16 ? '0' : '') + v[i].toString(16);
      }
      return out;
    });
  }

  function tableUrl(entry, base, build) {
    return base + '/' + (entry.files || entry.id + '-files.json') +
           (build ? '?v=' + encodeURIComponent(build) : '');
  }

  function storeTable(cache, table) {
    return cache.put(TABLE_KEY, new Response(JSON.stringify(table), {
      headers: { 'Content-Type': 'application/json' }
    }));
  }

  function readTable(cache) {
    return cache.match(TABLE_KEY).then(function (r) {
      return r ? r.json().catch(function () { return null; }) : null;
    });
  }

  // The unpacker's rule, imported rather than restated.
  var cacheKeyFor = ApUnpack.cachePathFor;

  // Sources for a changed file, best first.
  function sourcesFor(id, name, cfg) {
    var out = [cfg.base + '/files/' + name];
    if (id !== 'common' || name.indexOf('_images/') !== 0) {
      out.push('/' + name);
      return out;
    }
    var ids = [cfg.here].concat((cfg.wikis || []).map(function (w) { return w.id; }));
    var seen = {};
    ids.forEach(function (w) {
      if (w && !seen[w]) { seen[w] = 1; out.push('/' + w + '/' + name); }
    });
    return out;
  }

  function fetchInto(cache, id, name, expected, cfg) {
    // Tagged so the worker goes to the network, not the cache being refreshed.
    var tag = 'ap-update=' + encodeURIComponent(cfg.build || '1');
    var urls = sourcesFor(id, name, cfg).map(function (u) {
      return u + (u.indexOf('?') === -1 ? '?' : '&') + tag;
    });
    var key = cacheKeyFor(id, name);
    function attempt(i) {
      if (i >= urls.length) {
        throw new Error('could not fetch ' + name);
      }
      return pace().then(function () {
        return fetch(urls[i], {
          cache: 'no-cache',
          signal: cfg.getSignal ? cfg.getSignal() : undefined
        });
      }).then(function (r) {
        // Rate limited: stop rather than try the next source.
        if (r.status === 429 || r.status === 503) {
          var e = new Error('server is rate limiting updates');
          e.name = 'RateLimited';
          throw e;
        }
        if (!r.ok) { return attempt(i + 1); }
        return r.arrayBuffer().then(function (body) {
          // A body that does not hash to the table's value is refused, and
          // a table row with no hash cannot vouch for any body at all.
          if (!expected) {
            return attempt(i + 1);
          }
          return hashBytes(body).then(function (got) {
            if (got !== expected) { return attempt(i + 1); }
            return cache.put(new Request(key), new Response(body, {
              headers: { 'Content-Type': cfg.mimeFor(name) }
            }));
          });
        });
      }, function (err) {
        if (err && (err.name === 'AbortError' || err.name === 'RateLimited')) {
          throw err;
        }
        return attempt(i + 1);
      });
    }
    return attempt(0);
  }

  /** Apply the table difference; null means the caller re-fetches the archive. */
  function updateStored(entry, cfg, onProgress) {
    var cacheName = cfg.offlinePrefix + entry.id;
    return caches.open(cacheName).then(function (cache) {
      return Promise.all([
        readTable(cache),
        fetch(tableUrl(entry, cfg.base, cfg.build), { cache: 'no-cache' })
          .then(function (r) {
            if (!r.ok) { throw new Error('could not fetch the file list'); }
            return r.json();
          })
      ]).then(function (both) {
        var stored = both[0], published = both[1];
        if (!stored) { return null; }

        var changed = [], removed = [];
        Object.keys(published).forEach(function (k) {
          if (stored[k] !== published[k]) { changed.push(k); }
        });
        Object.keys(stored).forEach(function (k) {
          if (!Object.prototype.hasOwnProperty.call(published, k)) {
            removed.push(k);
          }
        });

        var total = Object.keys(published).length || 1;
        if (changed.length > MAX_DIFF_FILES ||
            (total >= DIFF_FRACTION_MIN_FILES &&
             changed.length / total > MAX_DIFF_FRACTION)) {
          return null;
        }

        // Paced from the start of each request.
        var done = 0;
        function next(i) {
          if (i >= changed.length) { return Promise.resolve(); }
          var startedAt = Date.now();
          return fetchInto(cache, entry.id, changed[i],
                           published[changed[i]], cfg).then(function () {
            done += 1;
            if (onProgress) { onProgress(done, changed.length); }
            if (i + 1 >= changed.length) { return next(i + 1); }
            var wait = MIN_REQUEST_GAP_MS - (Date.now() - startedAt);
            if (wait <= 0) { return next(i + 1); }
            return new Promise(function (go) { setTimeout(go, wait); })
              .then(function () { return next(i + 1); });
          });
        }

        return next(0).then(function () {
          return Promise.all(removed.map(function (k) {
            return cache.delete(new Request(cacheKeyFor(entry.id, k)));
          }));
        }).then(function () {
          // Written last: an interruption leaves the previous build's metadata
          // intact, and the stale table re-lists any files already fetched, so
          // the next successful update heals the mixed content.
          return storeTable(cache, published);
        }).then(function () {
          return cache.put(cfg.completeMarker, new Response(JSON.stringify({
            build: cfg.build, saved: Date.now(), id: entry.id
          }), { headers: { 'Content-Type': 'application/json' } }));
        }).then(function () {
          // `done`, not `changed.length`: completed writes, not planned ones.
          return { id: entry.id, changed: done, removed: removed.length };
        });
      });
    });
  }

  global.ApUpdate = {
    updateStored: updateStored,
    tableUrl: tableUrl,
    storeTable: storeTable,
    TABLE_KEY: TABLE_KEY,
    hashBytes: hashBytes
  };
})(typeof self !== 'undefined' ? self : this);
