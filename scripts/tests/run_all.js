#!/usr/bin/env node
/**
 * Runs the test suites in parallel, each in its own process, and reports
 * one line per suite as it finishes, then the full output of any that
 * failed. The browser suites run one engine per process, after the export
 * suite they share an artefact with.
 *
 *   node scripts/tests/run_all.js                 the unit suites (npm test)
 *   node scripts/tests/run_all.js --only browsers  the browser engines
 *   node scripts/tests/run_all.js --all            everything
 *   --jobs N        parallel processes (default: CPUs, at most 6)
 *   --page-shards N the page suite split N ways (default 4)
 *   --browsers a,b  engines for the browser jobs (default chromium,firefox,webkit)
 */
'use strict';
const { spawn } = require('child_process');
const os = require('os');
const path = require('path');

const ROOT = path.join(__dirname, '..', '..');
const args = process.argv.slice(2);
const opt = (name, dflt) => {
  const i = args.indexOf('--' + name);
  return i === -1 ? dflt : args[i + 1];
};
// A count given without a number, or with something that is not one, falls
// back to the default: no flag can quietly reduce the run to nothing.
const count = (name, dflt) => {
  const n = Number(opt(name, dflt));
  return Number.isFinite(n) && n >= 1 ? Math.floor(n) : dflt;
};
const only = args.includes('--all') ? 'all' : opt('only', 'unit');
if (['unit', 'browsers', 'all'].indexOf(only) === -1) {
  console.error('run_all: --only takes unit, browsers or all, not "' + only + '"');
  process.exit(2);
}
const jobsMax = count('jobs', Math.min(os.cpus().length, 6));
const engines = opt('browsers', 'chromium,firefox,webkit').split(',');

const node = process.execPath;
const py = 'python3';
const T = (f) => path.join('scripts', 'tests', f);

// The page suite is the long one; its sections are independent, so it runs
// as shards, each a process.
const PAGE_SHARDS = count('page-shards', 4);
const pageShards = Array.from({ length: PAGE_SHARDS }, (_, i) => ({
  name: 'page ' + (i + 1) + '/' + PAGE_SHARDS,
  cmd: [node, T('test_offline_page.js'), '--shard', (i + 1) + '/' + PAGE_SHARDS],
}));

const unit = pageShards.concat([

  { name: 'worker',    cmd: [node, T('test_offline_worker.js'), 'rover'] },
  { name: 'export',    cmd: [node, T('test_offline_export.js'), 'rover'] },
  { name: 'video',     cmd: [node, T('test_pwa_video.js')] },
  { name: 'nav',       cmd: [node, T('test_pwa_nav.js')] },
  { name: 'decoder',   cmd: [node, T('test_zstd_delta.js')] },
  { name: 'archives',  cmd: [py, T('test_offline_archives.py')] },
  { name: 'build',     cmd: [py, '-m', 'unittest', 'scripts.tests.test_build_offline_artifacts'] },
  { name: 'update.py', cmd: [py, '-m', 'unittest', 'scripts.tests.test_update_build.TestImageCacheWiring'] },
  { name: 'youtube',   cmd: [py, T('test_lazy_youtube.py')] },
  { name: 'iframes',   cmd: [py, T('test_check_raw_iframes.py')] },
  { name: 'images',    cmd: [py, T('test_image_resize.py')] },
  { name: 'optimise',  cmd: [py, T('test_optimise_images.py')] },
]);
// One process per engine; each starts its own server. They read the export
// artefact the export suite writes, so they wait for it.
const browsers = engines.map((e) => ({
  name: 'browser:' + e, cmd: [node, T('test_offline_browsers.js'), '--browsers', e],
  after: 'export',
}));

const exportJob = unit.find((j) => j.name === 'export');
let jobs = only === 'browsers' ? [exportJob].concat(browsers)
         : only === 'all' ? unit.concat(browsers) : unit;

function run(job) {
  return new Promise((resolve) => {
    const started = Date.now();
    const child = spawn(job.cmd[0], job.cmd.slice(1), { cwd: ROOT, env: process.env });
    let out = '';
    child.stdout.on('data', (d) => { out += d; });
    child.stderr.on('data', (d) => { out += d; });
    child.on('close', (code) => {
      resolve({ job, code, out, ms: Date.now() - started });
    });
  });
}

function summary(out) {
  const lines = out.split('\n').map((l) => l.trim()).filter(Boolean);
  const tail = lines.slice(-6).reverse();
  return tail.find((l) => /passed|FAILED|CHECK\(S\) FAILED|^OK|^Ran /.test(l)) || lines[lines.length - 1] || '';
}

(async () => {
  const done = new Map();
  const results = [];
  const pending = jobs.slice();
  const running = new Set();
  console.log('running ' + jobs.length + ' suites, ' + jobsMax + ' at a time\n');
  await new Promise((finish) => {
    const pump = () => {
      while (running.size < jobsMax) {
        const i = pending.findIndex((j) => !j.after || done.has(j.after));
        if (i === -1) { break; }
        const job = pending.splice(i, 1)[0];
        const p = run(job).then((r) => {
          running.delete(p);
          done.set(job.name, r.code);
          results.push(r);
          const fails = (r.out.match(/^\s*FAIL /gm) || []).length;
          console.log((r.code === 0 ? '  ok   ' : '  FAIL ') + job.name.padEnd(18) +
                      (r.ms / 1000).toFixed(1).padStart(7) + 's  ' + summary(r.out) +
                      (fails && r.code !== 0 ? '  (' + fails + ' FAIL lines)' : ''));
          if (!pending.length && !running.size) { finish(); } else { pump(); }
        });
        running.add(p);
      }
      if (!pending.length && !running.size) { finish(); }
    };
    pump();
  });
  const failed = results.filter((r) => r.code !== 0);
  failed.forEach((r) => {
    console.log('\n---- ' + r.job.name + ' (exit ' + r.code + ') ----\n' + r.out.trim() + '\n');
  });
  const total = results.reduce((a, r) => a + r.ms, 0);
  const said = failed.length ? failed.length + ' SUITE(S) FAILED'
             : results.length ? 'all suites passed' : 'NO SUITES RAN';
  console.log('\n' + said + '  (' + (total / 1000).toFixed(0) + ' s of suite time)');
  process.exit(failed.length || !results.length ? 1 : 0);
})();
