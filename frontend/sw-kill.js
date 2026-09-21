/*
 * Kill switch. Deployed in place of sw.js it opts every reader out on their
 * next visit, exactly as the switch on the offline page does: it deletes
 * every ardupilot-* cache, saved wikis included, tells each open page to
 * clear the opt-in flag so pwa.js does not register it again, and
 * unregisters itself. Nothing is left behind. Needs sw.js served no-cache.
 */

self.addEventListener('install', () => {
  self.skipWaiting();
});

self.addEventListener('activate', (event) => {
  event.waitUntil((async () => {
    const names = await caches.keys();
    await Promise.all(names
      .filter((name) => name.startsWith('ardupilot-') ||
                        name === 'ap-offline-off')
      .map((name) => caches.delete(name)));

    // The page that registered this worker is not controlled by it yet.
    const clients = await self.clients.matchAll({ type: 'window', includeUncontrolled: true });
    clients.forEach((client) => { client.postMessage({ type: 'OFFLINE_KILLED' }); });

    await self.registration.unregister();
  })());
});

// Pass everything straight through while the cleanup happens.
self.addEventListener('fetch', () => {});
