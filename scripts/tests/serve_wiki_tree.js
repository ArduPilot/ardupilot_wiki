/*
 * Serve the locally built wiki as production lays it out, on one origin:
 *
 *   /offline/<f>        -> offline/<f>
 *   /<wiki>/<rest>      -> <wiki>/build/html/<rest>
 *   everything else     -> frontend/<path>
 *
 * Used by test_offline_browsers.js, which stops the server to take every
 * engine offline.
 */
'use strict';

const http = require('http');
const fs = require('fs');
const path = require('path');

const ROOT = path.resolve(__dirname, '..', '..');
const WIKIS = new Set(['copter', 'plane', 'rover', 'sub', 'blimp', 'dev',
  'antennatracker', 'planner', 'planner2', 'ardupilot', 'mavproxy']);

const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'application/javascript; charset=utf-8',
  '.mjs': 'application/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.jpeg': 'image/jpeg',
  '.gif': 'image/gif',
  '.webp': 'image/webp',
  '.svg': 'image/svg+xml',
  '.ico': 'image/x-icon',
  '.woff': 'font/woff',
  '.woff2': 'font/woff2',
  '.ttf': 'font/ttf',
  '.eot': 'application/vnd.ms-fontobject',
  '.txt': 'text/plain; charset=utf-8',
  '.xml': 'application/xml',
  '.gz': 'application/gzip',
  '.inv': 'application/octet-stream',
  '.pdf': 'application/pdf',
  '.zip': 'application/zip',
};

/** Map a request path onto a file in the source tree. */
function resolveFile(urlPath) {
  let rel = decodeURIComponent(urlPath.split('?')[0].split('#')[0]);
  if (rel.endsWith('/')) {
    rel += 'index.html';
  }
  rel = rel.replace(/^\/+/, '');

  // No traversal out of the tree, whatever the path claims to be.
  const parts = rel.split('/').filter((p) => p && p !== '.' && p !== '..');
  if (!parts.length) {
    return path.join(ROOT, 'frontend', 'index.html');
  }

  if (parts[0] === 'offline') {
    return path.join(ROOT, 'offline', ...parts.slice(1));
  }
  if (WIKIS.has(parts[0])) {
    return path.join(ROOT, parts[0], 'build', 'html', ...parts.slice(1));
  }
  return path.join(ROOT, 'frontend', ...parts);
}

// The header rules the feature depends on.
function extraHeaders(urlPath) {
  if (urlPath === '/sw.js') {
    return { 'Cache-Control': 'no-cache', 'Service-Worker-Allowed': '/' };
  }
  if (urlPath === '/manifest.json' || urlPath.startsWith('/offline/')) {
    return { 'Cache-Control': 'no-cache', 'Access-Control-Allow-Origin': '*' };
  }
  return {};
}

// Bumping this changes sw.js, so a test can cause a worker update.
let workerBuild = 0;
function bumpWorker() {
  workerBuild += 1;
  return workerBuild;
}

// The kill switch served as /sw.js, as an emergency deploy would.
let killWorker = false;
function serveKill(on) {
  killWorker = on;
}

function createServer() {
  return http.createServer((req, res) => {
    const urlPath = (req.url || '/').split('?')[0];
    const file = urlPath === '/sw.js' && killWorker
      ? path.join(ROOT, 'frontend', 'sw-kill.js') : resolveFile(req.url || '/');

    // As nginx gzip_static does.
    if (urlPath.endsWith('.tar') && fs.existsSync(file + '.gz')) {
      const gz = file + '.gz';
      res.writeHead(200, Object.assign({
        'Content-Type': 'application/octet-stream',
        'Content-Length': fs.statSync(gz).size,
        'Content-Encoding': 'gzip',
      }, extraHeaders(urlPath)));
      if (req.method === 'HEAD') { res.end(); return; }
      fs.createReadStream(gz).pipe(res);
      return;
    }

    if (urlPath === '/sw.js' && workerBuild) {
      fs.readFile(file, 'utf8', (err, body) => {
        if (err) {
          res.writeHead(404); res.end(); return;
        }
        const stamped = body + '\n// test build ' + workerBuild + '\n';
        res.writeHead(200, Object.assign({
          'Content-Type': TYPES['.js'],
          'Content-Length': Buffer.byteLength(stamped),
        }, extraHeaders(urlPath)));
        res.end(stamped);
      });
      return;
    }

    fs.stat(file, (err, stat) => {
      if (err || !stat.isFile()) {
        if (!err && stat.isDirectory()) {
          res.writeHead(301, { Location: urlPath + '/' });
          res.end();
          return;
        }
        res.writeHead(404, { 'Content-Type': 'text/html; charset=utf-8' });
        res.end('<h1>404</h1>');
        return;
      }
      const type = TYPES[path.extname(file).toLowerCase()] ||
        'application/octet-stream';
      res.writeHead(200, Object.assign({
        'Content-Type': type,
        'Content-Length': stat.size,
      }, extraHeaders(urlPath)));
      if (req.method === 'HEAD') {
        res.end();
        return;
      }
      fs.createReadStream(file).pipe(res);
    });
  });
}

/** Listen on an ephemeral port; resolves with { server, port, close }. */
function start(port) {
  return new Promise((resolve, reject) => {
    const server = createServer();
    // Or close() waits on the browser's keep-alive sockets.
    server.keepAliveTimeout = 1;
    const sockets = new Set();
    server.on('connection', (s) => {
      sockets.add(s);
      s.on('close', () => sockets.delete(s));
    });
    server.on('error', reject);
    server.listen(port || 0, '127.0.0.1', () => {
      resolve({
        server,
        port: server.address().port,
        close: () => new Promise((done) => {
          sockets.forEach((s) => s.destroy());
          server.close(() => done());
        }),
      });
    });
  });
}

module.exports = { start, createServer, resolveFile, bumpWorker, serveKill, WIKIS };

if (require.main === module) {
  const port = Number(process.argv[2] || 8000);
  start(port).then(({ port: p }) => {
    console.log(`serving the built wiki on http://localhost:${p}/`);
  });
}
