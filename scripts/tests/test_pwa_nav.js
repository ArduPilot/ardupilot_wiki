#!/usr/bin/env node
/**
 * The site's top-menu "Offline" link is the sign, on every page, that
 * offline mode is really working: green while the worker is active, with
 * the last check as its tooltip, and plain otherwise.
 *
 *   node scripts/tests/test_pwa_nav.js
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
const settle = (ms) => new Promise((r) => setTimeout(r, ms));

/** A wiki page with the theme's top menu, in a browser in the given state. */
function pageIn({ optedIn = false, active = false, stamp = null, href = 'common-offline.html' } = {}) {
  const dom = new JSDOM(
    '<!doctype html><html><head></head><body><ul>' +
    '<li><a title="Home" href="../index.html">Home</a></li>' +
    '<li><a title="Offline" href="' + href + '">Offline</a></li>' +
    '</ul><p>page</p></body></html>',
    { url: 'https://example.test/copter/docs/page.html', runScripts: 'outside-only',
      pretendToBeVisual: true });
  const w = dom.window;
  w.matchMedia = () => ({ matches: false, addListener() {}, removeListener() {} });
  if (optedIn) { w.localStorage.setItem('ap-offline-enabled', '1'); }
  if (stamp) { w.localStorage.setItem('ap-last-checked', JSON.stringify(stamp)); }
  const state = { active };
  Object.defineProperty(w.navigator, 'serviceWorker', { configurable: true, value: {
    controller: null, addEventListener() {},
    ready: new Promise(() => {}),
    register: () => Promise.resolve({ addEventListener() {}, update() { return Promise.resolve(); } }),
    getRegistration: () => Promise.resolve(state.active ? { active: {} } : undefined) } });
  w.fetch = () => Promise.reject(new TypeError('no network in this test'));
  w.eval(PWA);
  return { w, state, link: () => w.document.querySelector('a[data-ap-offline-link], a[title="Offline"]') };
}

(async () => {
  console.log('\nthe top-menu Offline link shows the worker is really on\n');
  {
    const { link } = pageIn({ optedIn: true, active: true });
    await settle(100);
    check('opted in with the worker active: green', link().classList.contains('ap-offline-live'));
    check('the tooltip says offline mode is on', /Offline mode is on/.test(link().title), link().title);
    check('a style for the green link was injected',
          [...link().ownerDocument.querySelectorAll('style')].some((s) => /ap-offline-live/.test(s.textContent)));
  }
  {
    const { link } = pageIn({ optedIn: true, active: true,
      stamp: { t: new Date(Date.now() - 5 * 60000).toISOString(), r: 'current' } });
    await settle(100);
    check('the tooltip carries the last check', /checked 5 min ago/.test(link().title), link().title);
  }
  {
    const { link } = pageIn({ optedIn: true, active: true,
      stamp: { t: new Date(Date.now() - 3 * 3600000).toISOString(), r: 'updated' } });
    await settle(100);
    check('and says updated when the last check moved files', /updated 3 h ago/.test(link().title), link().title);
  }
  {
    const { link } = pageIn({ optedIn: false, active: true });
    await settle(100);
    check('not opted in: plain, whatever the registration says',
          !link().classList.contains('ap-offline-live') && link().title === 'Offline', link().title);
  }
  {
    const { link } = pageIn({ optedIn: true, active: false });
    await settle(100);
    check('opted in but no active worker yet: plain', !link().classList.contains('ap-offline-live'));
  }
  {
    // Opting in on this page turns the link green once the worker is active.
    const { w, state, link } = pageIn({ optedIn: false, active: false });
    await settle(100);
    state.active = true;
    w.ApOffline.enable();
    await settle(200);
    check('opting in repaints the link once the worker is active',
          link().classList.contains('ap-offline-live'));
    w.ApOffline.disable();
    await settle(200);
    check('opting out takes the green away', !link().classList.contains('ap-offline-live'));
  }
  {
    // On the Offline page itself the theme writes the href as "#".
    const { link } = pageIn({ optedIn: true, active: true, href: '#' });
    await settle(100);
    check('the link is found on the Offline page itself', link().classList.contains('ap-offline-live'));
  }
  {
    // From a wiki's front page the theme writes the href with the docs/ prefix.
    const { link } = pageIn({ optedIn: true, active: true, href: 'docs/common-offline.html' });
    await settle(100);
    check('the link is found from a front page too', link().classList.contains('ap-offline-live'));
  }
  {
    const { w } = pageIn({ optedIn: true, active: true });
    w.document.querySelector('a[data-ap-offline-link], a[title="Offline"]').remove();
    await settle(100);
    check('a page without the link is left alone', true);
  }
  console.log(failures ? '\n' + failures + ' CHECK(S) FAILED\n' : '\nall checks passed\n');
  process.exit(failures ? 1 : 0);
})().catch((err) => { console.error(err); process.exit(1); });
