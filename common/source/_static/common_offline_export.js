/*
 * [copywiki destination="copter,plane,rover,sub,blimp,antennatracker,dev,planner,planner2,ardupilot,mavproxy"]
 *
 * Writes the single-file offline export. Reads the saved wikis out of Cache
 * Storage, inlines images and stylesheets as data URIs, and streams the result
 * to disk through the service worker so a 900 MB file never sits in memory.
 * What the file contains is decided by common_offline_document_builder.js.
 */
(function (global) {
  'use strict';

  var OFFLINE_CACHE_PREFIX = 'ardupilot-offline-';
  var COMPLETE_MARKER = '/__ap_complete__';

  /* --------------------------------------------------------- cache reading */

  /** Every cached entry belonging to a stored wiki, as [path, Response]. */
  function storedEntries(wikiIds) {
    return caches.keys().then(function (names) {
      var wanted = names.filter(function (n) {
        if (n.indexOf(OFFLINE_CACHE_PREFIX) !== 0) { return false; }
        var id = n.slice(OFFLINE_CACHE_PREFIX.length);
        return wikiIds.indexOf(id) !== -1 || id === 'common';
      });
      return Promise.all(wanted.map(function (name) {
        return caches.open(name).then(function (cache) {
          // An aborted download must not export as if it were whole.
          return cache.match('/__ap_complete__').then(function (marker) {
            if (!marker) {
              var id = name.slice(OFFLINE_CACHE_PREFIX.length);
              throw new Error((id === 'common' ? 'The shared images are' : id + ' is') +
                              ' incomplete in this browser. Save again, then export.');
            }
            return cache.keys().then(function (reqs) {
              return { cache: cache, reqs: reqs };
            });
          });
        });
      }));
    });
  }

  /* ------------------------------------------------------ download plumbing */

  /** A download sink: service worker stream, else File System Access, else a Blob. */
  function openDownload(filename) {
    if (navigator.serviceWorker && navigator.serviceWorker.controller &&
        typeof TransformStream !== 'undefined') {
      var ts = new TransformStream();
      var writer = ts.writable.getWriter();
      var id = String(Date.now()) + Math.random().toString(36).slice(2);

      navigator.serviceWorker.controller.postMessage(
        { type: 'EXPORT_START', id: id, filename: filename, stream: ts.readable },
        [ts.readable]
      );

      // An iframe: navigating away would tear down the generating page.
      var frame = document.createElement('iframe');
      frame.hidden = true;
      frame.src = '/__export__/' + id;
      document.body.appendChild(frame);

      return Promise.resolve({
        write: function (chunk) { return writer.write(chunk); },
        close: function () {
          return writer.close().then(function () {
            setTimeout(function () { frame.remove(); }, 2000);
          });
        },
        // Erroring the stream cancels the browser-side download outright;
        // the frame goes only after the writer, mirroring close().
        abort: function () {
          return writer.abort(new Error('cancelled')).catch(function () {})
            .then(function () { frame.remove(); });
        }
      });
    }

    // Needs a user gesture, which the click that got us here provides.
    if (global.showSaveFilePicker) {
      return global.showSaveFilePicker({ suggestedName: filename })
        .then(function (handle) { return handle.createWritable(); })
        .then(function (w) {
          return {
            write: function (chunk) { return w.write(chunk); },
            close: function () { return w.close(); },
            abort: function () { return w.abort().catch(function () {}); }
          };
        });
    }

    var parts = [];
    return Promise.resolve({
      abort: function () { parts.length = 0; return Promise.resolve(); },
      write: function (chunk) { parts.push(chunk); return Promise.resolve(); },
      close: function () {
        var url = URL.createObjectURL(new Blob(parts));
        var a = document.createElement('a');
        a.href = url; a.download = filename;
        a.click();
        setTimeout(function () { URL.revokeObjectURL(url); }, 30000);
        return Promise.resolve();
      }
    });
  }

  /* --------------------------------------------------- exports: single HTML */

  var BINARY = /\.(png|jpe?g|gif|webp|svg|ico|bmp|woff2?|ttf)$/i;

  function mimeFor(path) {
    var ext = (path.split('.').pop() || '').toLowerCase();
    return ({ png: 'image/png', jpg: 'image/jpeg', jpeg: 'image/jpeg',
              gif: 'image/gif', webp: 'image/webp', svg: 'image/svg+xml',
              bmp: 'image/bmp',
              ico: 'image/x-icon', woff: 'font/woff', woff2: 'font/woff2',
              ttf: 'font/ttf' })[ext] || 'application/octet-stream';
  }

  // Looked up per call so the scripts may load in either order.
  function resolvePath(basePath, href) {
    return global.ArduPilotOfflineDocument.resolvePath(basePath, href);
  }

  function base64(bytes) {
    // Chunked: fromCharCode.apply blows the argument limit on large images.
    var out = '', CHUNK = 0x8000;
    for (var i = 0; i < bytes.length; i += CHUNK) {
      out += String.fromCharCode.apply(null, bytes.subarray(i, i + CHUNK));
    }
    return btoa(out);
  }

  /** Sphinx's search index, trimmed to what searching needs (11 MB -> 5 MB). */
  function readSearchIndex(entry) {
    return ApUnpack.readFrom(entry.cache, entry.path)
      .then(function (r) { return r.text(); })
      .then(function (src) {
        var open = src.indexOf('('), close = src.lastIndexOf(')');
        if (open === -1 || close <= open) { return null; }
        var d = JSON.parse(src.slice(open + 1, close));
        if (!d.terms || !d.docnames) { return null; }
        return { docnames: d.docnames, titles: d.titles || [],
                 terms: d.terms, titleterms: d.titleterms || {} };
      })
      .catch(function () { return null; });
  }

  /** Write one self-contained HTML file from the cached pages, page by page. */
  function exportHtml(wikiIds, filename, onProgress, sink, signal) {
    var enc = new TextEncoder();
    var DOC = global.ArduPilotOfflineDocument;
    if (!DOC) {
      throw new Error('common_offline_document_builder.js is not loaded.');
    }

    return storedEntries(wikiIds).then(function (groups) {
      var pages = [], assets = {}, styles = {};
      // Sphinx's stemmed index and its stemmer are both in the cache.
      var indexes = {}, stemmerSrc = null;

      groups.forEach(function (g) {
        g.reqs.forEach(function (req) {
          var path = new URL(req.url).pathname;
          if (path === COMPLETE_MARKER) { return; }
          var si = path.match(/^\/([^/]+)\/searchindex\.js$/);
          if (si) { indexes[si[1]] = { cache: g.cache, path: path }; return; }
          if (/\/_static\/language_data\.js$/.test(path)) {
            if (!stemmerSrc) { stemmerSrc = { cache: g.cache, path: path }; }
            return;
          }
          if (BINARY.test(path)) { assets[path] = g.cache; }
          else if (/\.css$/.test(path)) { styles[path] = g.cache; assets[path] = g.cache; }
          else if (/\.html?$/.test(path)) {
            pages.push({ path: path, cache: g.cache });
          }
        });
      });

      if (!pages.length) {
        throw new Error('Nothing is saved yet - download a wiki first.');
      }
      pages.sort(function (a, b) { return a.path < b.path ? -1 : 1; });

      // Decide which parameter-list versions to carry before anything is written.
      var params = DOC.parameterVersions(pages.map(function (p) {
        return p.path.replace(/\.html?$/, '');
      }));
      pages = pages.filter(function (p) {
        return !params.drop[p.path.replace(/\.html?$/, '')];
      });

      // From the pages themselves, so a wiki appears even without its index page.
      var wikis = [];
      pages.forEach(function (pg) {
        var w = pg.path.split('/')[1];
        if (w && wikis.indexOf(w) === -1) { wikis.push(w); }
      });
      wikis.sort();

      return buildThemeCss(styles, assets).then(function (themeCss) {
        return (sink ? Promise.resolve(sink) : openDownload(filename))
        .then(function (sink) {
          // Checked between pages: a cancel stops the work, not mid-write.
          var bail = function () {
            var e = new Error('Export cancelled.');
            e.name = 'AbortError';
            return e;
          };
          var done = 0, index = [];
          // Each image is emitted once and referenced by id.
          var imgIds = { __next: 0 };
          var imgPaths = {};
          // The navigation is the union of every page's expanded sidebar.
          var navState = DOC.newNav();
          var write = function (text) { return sink.write(enc.encode(text)); };

          return write(DOC.head(wikis, themeCss)).then(function () {
            var chain = Promise.resolve();
            pages.forEach(function (p, i) {
              chain = chain.then(function () {
                if (signal && signal.aborted) { throw bail(); }
                return ApUnpack.readFrom(p.cache, p.path)
                  .then(function (res) { return res.text(); })
                  .then(function (html) {
                    var title = (html.match(/<title>([^<]*)<\/title>/i) || [])[1] ||
                                p.path.replace(/^\//, '');
                    // Strip the theme's " <dash> Project documentation" suffix.
                    title = title.split('&mdash;')[0].split(' — ')[0].trim();
                    index.push({ t: title, p: p.path.replace(/\.html?$/, '') });

                    DOC.addNav(navState, html, p.path);

                    var m = html.match(/<div[^>]*itemprop="articleBody"[^>]*>([\s\S]*?)<\/div>\s*<footer/i);
                    return referenceImages(showAllTabs(m ? m[1] : html), assets, p.path,
                                           imgIds, imgPaths);
                  })
                  .then(function (r) {
                    done++;
                    if (onProgress && done % 10 === 0) { onProgress(done, pages.length); }
                    return write(DOC.pageBlock(i, r.html, r.fresh));
                  });
              });
            });
            return chain;
          }).then(function () {
            if (signal && signal.aborted) { throw bail(); }
            // One wiki opens directly; several show the list.
            var homes = DOC.wikiHomes(index, wikis);
            // Sidebar and reading order from one call, so they agree.
            var nav = DOC.buildNav(navState, wikis, pages);
            var payload = { pages: index, nav: nav.html, order: nav.order,
                            wikis: wikis, imgs: imgPaths, homes: homes,
                            params: params.byWiki,
                            home: homes.length === 1 ? homes[0].path : '' };

            var wantIndex = wikis.filter(function (w) { return indexes[w]; });
            return Promise.all(wantIndex.map(function (w) {
              return readSearchIndex(indexes[w]).then(function (d) {
                return d ? [w, d] : null;
              });
            })).then(function (loaded) {
              var byWiki = {};
              loaded.forEach(function (e) { if (e) { byWiki[e[0]] = e[1]; } });
              var stem = stemmerSrc
                ? ApUnpack.readFrom(stemmerSrc.cache, stemmerSrc.path)
                    .then(function (r) { return r.text(); })
                    .catch(function () { return ''; })
                : Promise.resolve('');
              return stem.then(function (stemSrc) {
                return write(DOC.searchBlock(byWiki, stemSrc));
              });
            }).then(function () {
              if (signal && signal.aborted) { throw bail(); }
              return write(DOC.tail(payload));
            });
          }).then(function () {
            if (signal && signal.aborted) { throw bail(); }
            return sink.close();
          })
            .then(function () { return { pages: done }; })
            .catch(function (err) {
              // Cancelled or failed, the reader must not be handed the file.
              if (sink.abort) {
                return Promise.resolve(sink.abort()).then(
                  function () { throw err; }, function () { throw err; });
              }
              throw err;
            });
        });
      });
    });
  }

  /** The theme's stylesheets with their fonts inlined. */
  // The page's whole cascade, in the order the site loads it.
  var THEME_CSS_ORDER = ['css/theme.css', 'css/badge_only.css', 'pygments.css',
                         'css/ardupilot.css', 'custom.css',
                         'common_theme_override.css'];

  function themeCssRank(path) {
    for (var i = 0; i < THEME_CSS_ORDER.length; i++) {
      if (path.indexOf('_static/' + THEME_CSS_ORDER[i]) !== -1) { return i; }
    }
    return -1;
  }

  function buildThemeCss(styles, assets) {
    var wanted = Object.keys(styles).filter(function (p) {
      return themeCssRank(p) !== -1;
    }).sort(function (a, b) { return themeCssRank(a) - themeCssRank(b); });
    if (!wanted.length) { return Promise.resolve(''); }

    // One wiki's copy is enough - they are identical across wikis.
    var seen = {};
    wanted = wanted.filter(function (p) {
      var base = p.replace(/^\/[^/]+\//, '');
      if (seen[base]) { return false; }
      seen[base] = 1;
      return true;
    });

    var chain = Promise.resolve('');
    wanted.forEach(function (path) {
      chain = chain.then(function (acc) {
        return ApUnpack.readFrom(styles[path], path)
          .then(function (r) { return r.text(); })
          .then(function (css) { return inlineCssUrls(css, path, assets); })
          .then(function (css) { return acc + '\n' + css; })
          .catch(function () { return acc; });
      });
    });
    return chain;
  }

  /** Replace url(...) in a stylesheet with data URIs from the cache. */
  function inlineCssUrls(css, cssPath, assets) {
    var refs = [];
    css.replace(/url\(\s*['"]?([^'")]+)['"]?\s*\)/g, function (all, ref) {
      if (!/^(data:|https?:|\/\/)/.test(ref) && refs.indexOf(ref) === -1) {
        refs.push(ref);
      }
      return all;
    });
    if (!refs.length) { return Promise.resolve(css); }

    var chain = Promise.resolve(css);
    refs.forEach(function (ref) {
      chain = chain.then(function (current) {
        var clean = ref.split('?')[0].split('#')[0];
        var resolved = clean.charAt(0) === '/' ? clean : resolvePath(cssPath, clean);
        if (!assets[resolved]) { return current; }
        return ApUnpack.readFrom(assets[resolved], resolved)
          .then(function (r) { return r.arrayBuffer(); })
          .then(function (buf) {
            var uri = 'data:' + mimeFor(resolved) + ';base64,' +
                      base64(new Uint8Array(buf));
            return current.split(ref).join(uri);
          })
          .catch(function () { return current; });
      });
    });
    return chain;
  }

  /** Point each <img> at a shared image block; returns the html and any new images. */
  // The export ships no page JavaScript, so sphinx_tabs' tab buttons do
  // nothing and its non-default panels stay hidden. The panels sit after
  // their tablist, so revealing them in place would run every label first
  // and every body after. Instead each panel is un-hidden and gets its own
  // label heading, paired by the shared name attribute, and the now-inert
  // tablist is dropped, so the export reads label-then-body per panel.
  function showAllTabs(html) {
    if (html.indexOf('sphinx-tabs-panel') === -1) { return html; }
    var labels = {};
    html.replace(
      /<button[^>]*class="[^"]*sphinx-tabs-tab[^"]*"[^>]*\bname="([^"]*)"[^>]*>([\s\S]*?)<\/button>/gi,
      function (all, name, text) { labels[name] = text.trim(); return all; });
    return html
      // Drop the tablist wrapper and its buttons; they cannot switch anything.
      .replace(/<div[^>]*role="tablist"[^>]*>[\s\S]*?<\/div>/gi, '')
      // Un-hide each panel, wherever the attribute sits in the tag.
      .replace(
        /(<div[^>]*class="[^"]*sphinx-tabs-panel[^"]*"[^>]*?)\s+hidden(="[^"]*")?/gi, '$1')
      // Prepend each panel's label, matched by the shared name attribute.
      .replace(
        /(<div[^>]*class="[^"]*sphinx-tabs-panel[^"]*"[^>]*\bname="([^"]*)"[^>]*>)/gi,
        function (all, open, name) {
          var label = labels[name];
          return open +
            (label ? '<p class="ap-tab-label"><strong>' + label + '</strong></p>' : '');
        });
  }

  function referenceImages(html, assets, pagePath, imgIds, imgPaths) {
    var srcs = [];
    html.replace(/<img[^>]+src="([^"]+)"/gi, function (all, src) {
      if (srcs.indexOf(src) === -1) { srcs.push(src); }
      return all;
    });
    if (!srcs.length) { return Promise.resolve({ html: html, fresh: [] }); }

    var fresh = [];
    var chain = Promise.resolve(html);

    srcs.forEach(function (src) {
      chain = chain.then(function (current) {
        if (/^(data:|https?:|\/\/)/.test(src)) { return current; }

        var clean = src.split('?')[0].split('#')[0];
        var resolved = clean.charAt(0) === '/' ? clean : resolvePath(pagePath, clean);
        var candidates = [
          resolved,
          resolved.replace(/^\/[^/]+\/_images\//, '/_common/_images/')
        ];

        var hit = null;
        candidates.forEach(function (c) { if (!hit && assets[c]) { hit = c; } });
        if (!hit) {
          // Drop the src so the alt text renders instead of a blank space.
          return current.split('src="' + src + '"')
                        .join('data-ap-missing="' + src + '"');
        }

        // Already emitted: point at it.
        if (imgIds[hit] !== undefined) {
          if (imgPaths) { imgPaths[resolved] = imgIds[hit]; }
          return current.split('src="' + src + '"')
                        .join('data-ap-img="' + imgIds[hit] + '"');
        }

        return ApUnpack.readFrom(assets[hit], hit)
          .then(function (res) { return res.arrayBuffer(); })
          .then(function (buf) {
            // The id is taken only once the bytes are in hand, so a failed
            // read here does not blank this image on every later page.
            var id = imgIds[hit] = imgIds.__next++;
            if (imgPaths) { imgPaths[resolved] = id; }
            fresh.push({
              id: id,
              uri: 'data:' + mimeFor(hit) + ';base64,' + base64(new Uint8Array(buf))
            });
            return current.split('src="' + src + '"')
                          .join('data-ap-img="' + id + '"');
          })
          .catch(function () {
            return current.split('src="' + src + '"')
                          .join('data-ap-missing="' + src + '"');
          });
      });
    });

    return chain.then(function (out) { return { html: out, fresh: fresh }; });
  }

  global.ArduPilotExport = {
    exportHtml: exportHtml,
    openDownload: openDownload
  };
})(window);
