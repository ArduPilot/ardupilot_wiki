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
function pageWith(href, probeAnswers, onLine) {
  const dom = new JSDOM(
    '<!doctype html><html><body>' +
    '<a class="ap-video" href="' + href + '" data-ap-external="1">' +
    '<img src="/copter/_images/still.jpg" alt="">' +
    '<span class="ap-video-label"><span style="opacity:.8">Video not loaded &middot; </span>' +
    'Watch on YouTube &#8599;</span></a></body></html>',
    { url: 'https://example.test/copter/docs/page.html', runScripts: 'outside-only',
      pretendToBeVisual: true });
  const w = dom.window;
  w.matchMedia = () => ({ matches: false, addListener() {}, removeListener() {} });
  if (onLine === false) {
    // Windows behind a VPN or virtual adapter: the OS says offline, the network works.
    Object.defineProperty(w.navigator, 'onLine', { configurable: true, value: false });
  }
  // A real window.open hands back the new tab; a blocked one hands back null.
  w.opened = [];
  w.blockPopups = false;
  w.open = (u) => {
    if (w.blockPopups) { w.opened.push({ url: String(u), blocked: true }); return null; }
    const tab = { url: String(u), closed: false, opener: w, close() { tab.closed = true; } };
    Object.defineProperty(tab, 'location', {
      get: () => tab.url, set: (v) => { tab.url = String(v); } });
    w.opened.push(tab);
    return tab;
  };
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
    const label = w.document.querySelector('.ap-video-label');
    check('the caption then says the video could not be loaded, and keeps the link wording',
          !!label && /could not be loaded/.test(label.textContent) &&
          /Watch on YouTube/.test(label.textContent), label ? label.textContent : 'no label');
  }
  {
    // A probe that never answers: the caption says it is connecting meanwhile.
    const { w } = pageWith('https://www.youtube.com/watch?v=tC0mF-N5z0Q', true);
    w.fetch = () => new Promise(() => {});
    w.dispatchEvent(new w.Event('load'));
    await settle(600);
    const label = w.document.querySelector('.ap-video-label');
    check('while the host is being probed the caption says so',
          !!label && /Connecting to YouTube/.test(label.textContent), label ? label.textContent : 'no label');
  }

  console.log('\nnavigator.onLine is a hint; the probe decides');
  {
    const { w, fetched } = pageWith('https://www.youtube.com/watch?v=tC0mF-N5z0Q', true, false);
    w.dispatchEvent(new w.Event('load'));
    await settle(600);
    check('a browser that says offline still probes the host',
          fetched.some((u) => u.startsWith('https://www.youtube.com/')), fetched.join(', '));
    check('and a host that answers gets the player anyway',
          !!w.document.querySelector('.ap-video-live iframe'));
  }
  {
    const { w } = pageWith('https://www.youtube.com/watch?v=tC0mF-N5z0Q', false, false);
    w.dispatchEvent(new w.Event('load'));
    await settle(600);
    const card = w.document.querySelector('a.ap-video');
    check('offline for real, the card stays', !!card && !w.document.querySelector('iframe'));
    const ev = new w.MouseEvent('click', { bubbles: true, cancelable: true });
    card.dispatchEvent(ev);
    await settle(300);
    const note = w.document.getElementById('ap-offline-note');
    check('a click offline is held with the note, not a browser error page',
          ev.defaultPrevented && !!note && !note.hidden && /needs a connection/.test(note.textContent));
    check('and the tab opened for it is closed again, never left blank',
          w.opened.length === 1 && w.opened[0].closed && w.opened[0].url === '',
          JSON.stringify(w.opened.map((t) => t.url + (t.closed ? ' (closed)' : ''))));
  }
  {
    const { w } = pageWith('https://www.youtube.com/watch?v=tC0mF-N5z0Q', true, false);
    // No load event: the card is not upgraded, the reader clicks the still.
    const card = w.document.querySelector('a.ap-video');
    const ev = new w.MouseEvent('click', { bubbles: true, cancelable: true });
    card.dispatchEvent(ev);
    // The tab has to exist before the probe answers: opened after it, the
    // browser has no user gesture to attribute it to and blocks it.
    check('the tab is opened in the click itself, not when the probe answers',
          w.opened.length === 1 && w.opened[0].url === '' && w.opened[0].opener === null,
          JSON.stringify(w.opened.map((t) => t.url)));
    await settle(300);
    check('a click while the OS says offline opens the link once the host answers',
          ev.defaultPrevented && w.opened.length === 1 &&
          /youtube\.com\/watch/.test(w.opened[0].url) && !w.opened[0].closed &&
          !w.document.getElementById('ap-offline-note'),
          JSON.stringify(w.opened.map((t) => t.url)));
  }
  {
    // A browser that blocks it anyway: the link still has to open.
    const { w } = pageWith('https://www.youtube.com/watch?v=tC0mF-N5z0Q', true, false);
    w.blockPopups = true;
    const card = w.document.querySelector('a.ap-video');
    card.dispatchEvent(new w.MouseEvent('click', { bubbles: true, cancelable: true }));
    await settle(300);
    check('with the tab blocked, the link is opened on the answer as before',
          w.opened.length === 2 && /youtube\.com\/watch/.test(w.opened[1].url),
          JSON.stringify(w.opened.map((t) => t.url)));
  }

  console.log(failures ? '\n' + failures + ' CHECK(S) FAILED\n' : '\nall checks passed\n');
  process.exit(failures ? 1 : 0);
})().catch((err) => { console.error(err); process.exit(1); });
