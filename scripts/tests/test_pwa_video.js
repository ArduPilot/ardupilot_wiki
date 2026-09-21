#!/usr/bin/env node
/**
 * pwa.js turns a saved page's video card into a player when the video host
 * answers. The player must come from the host the live pages embed from,
 * www.youtube.com, so a saved page plays wherever the live page plays: some
 * networks filter youtube-nocookie.com while letting youtube.com through.
 *
 *   node scripts/tests/test_pwa_video.js
 */
'use strict';
const fs = require('fs');
const path = require('path');
const { JSDOM } = require('jsdom');

let failures = 0;
function check(name, ok, detail) {
  console.log((ok ? '  PASS  ' : '  FAIL  ') + name + (detail ? '   ' + detail : ''));
  if (!ok) { failures++; }
}

const PWA = fs.readFileSync(path.join(__dirname, '..', '..', 'frontend', 'js', 'pwa.js'), 'utf8');

/** A page holding one video card, with the probe answering as told. */
function pageWith(href, probeAnswers) {
  const dom = new JSDOM(
    '<!doctype html><html><body>' +
    '<a class="ap-video" href="' + href + '" data-ap-external="1">' +
    '<img src="/copter/_images/still.jpg" alt=""></a></body></html>',
    { url: 'https://example.test/copter/docs/page.html', runScripts: 'outside-only',
      pretendToBeVisual: true });
  const w = dom.window;
  w.matchMedia = () => ({ matches: false, addListener() {}, removeListener() {} });
  // A worker API that registers nothing; the upgrade does not need one.
  Object.defineProperty(w.navigator, 'serviceWorker', { configurable: true, value: {
    controller: null, addEventListener() {},
    register: () => Promise.resolve({ addEventListener() {} }),
    getRegistration: () => Promise.resolve(undefined) } });
  const fetched = [];
  w.fetch = (u) => {
    fetched.push(String(u));
    return probeAnswers ? Promise.resolve({ type: 'opaque', status: 0 })
                        : Promise.reject(new TypeError('Failed to fetch'));
  };
  w.eval(PWA);
  return { w, fetched };
}

const settle = (ms) => new Promise((r) => setTimeout(r, ms));

(async () => {
  console.log('\nthe saved-page player comes from the host the live pages use');
  {
    const { w, fetched } = pageWith('https://www.youtube.com/watch?v=tC0mF-N5z0Q', true);
    w.dispatchEvent(new w.Event('load'));
    await settle(600);
    const frame = w.document.querySelector('.ap-video-live iframe');
    check('a reachable host gets the card replaced by a player', !!frame);
    check('the player embeds from www.youtube.com like the live page',
          !!frame && /^https:\/\/www\.youtube\.com\/embed\/tC0mF-N5z0Q/.test(frame.src),
          frame ? frame.src : 'no iframe');
    check('the reachability probe asked the host the player will load from',
          fetched.some((u) => u.startsWith('https://www.youtube.com/')), fetched.join(', '));
  }
  {
    const { w } = pageWith('https://vimeo.com/123456', true);
    w.dispatchEvent(new w.Event('load'));
    await settle(600);
    const frame = w.document.querySelector('.ap-video-live iframe');
    check('a Vimeo card gets the Vimeo player',
          !!frame && frame.src.startsWith('https://player.vimeo.com/video/123456'),
          frame ? frame.src : 'no iframe');
  }
  {
    const { w } = pageWith('https://www.youtube.com/watch?v=tC0mF-N5z0Q', false);
    w.dispatchEvent(new w.Event('load'));
    await settle(600);
    check('an unreachable host leaves the card and its link in place',
          !!w.document.querySelector('a.ap-video') && !w.document.querySelector('iframe'));
  }
  console.log(failures ? '\n' + failures + ' CHECK(S) FAILED\n' : '\nall checks passed\n');
  process.exit(failures ? 1 : 0);
})().catch((err) => { console.error(err); process.exit(1); });
