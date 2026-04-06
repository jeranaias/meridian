/* ============================================================
   offline-tiles.js — Cache-first Leaflet tile layer
   Custom L.TileLayer subclass that checks Cache API first,
   falls back to network. Provides downloadRegion() for
   pre-caching tiles for offline use.
   ============================================================ */

'use strict';

window.OfflineTiles = (function () {

    var CACHE_NAME = 'meridian-tiles-v1';

    // --- Custom Leaflet TileLayer ---
    var CachedTileLayer = L.TileLayer.extend({

        createTile: function (coords, done) {
            var tile = document.createElement('img');
            tile.alt = '';
            tile.setAttribute('role', 'presentation');

            var url = this.getTileUrl(coords);

            // Try cache first, fallback to network
            if ('caches' in window) {
                caches.open(CACHE_NAME).then(function (cache) {
                    return cache.match(url);
                }).then(function (response) {
                    if (response) {
                        return response.blob();
                    }
                    // Not cached — fetch from network and cache
                    return fetch(url).then(function (netResp) {
                        if (netResp.ok) {
                            var clone = netResp.clone();
                            caches.open(CACHE_NAME).then(function (cache) {
                                cache.put(url, clone);
                            });
                        }
                        return netResp.blob();
                    });
                }).then(function (blob) {
                    tile.src = URL.createObjectURL(blob);
                    done(null, tile);
                }).catch(function () {
                    // Last resort: direct load
                    tile.src = url;
                    tile.onload = function () { done(null, tile); };
                    tile.onerror = function () { done(new Error('Tile load failed'), tile); };
                });
            } else {
                // No Cache API — standard loading
                tile.src = url;
                tile.onload = function () { done(null, tile); };
                tile.onerror = function () { done(new Error('Tile load failed'), tile); };
            }

            return tile;
        }
    });

    // --- Factory ---
    function createLayer(url, options) {
        return new CachedTileLayer(url, options);
    }

    // --- Tile coordinate math ---
    function lon2tile(lon, zoom) {
        return Math.floor((lon + 180) / 360 * Math.pow(2, zoom));
    }

    function lat2tile(lat, zoom) {
        return Math.floor((1 - Math.log(Math.tan(lat * Math.PI / 180) +
            1 / Math.cos(lat * Math.PI / 180)) / Math.PI) / 2 * Math.pow(2, zoom));
    }

    function getTilesInBounds(bounds, zoom) {
        var ne = bounds.getNorthEast();
        var sw = bounds.getSouthWest();

        var xMin = lon2tile(sw.lng, zoom);
        var xMax = lon2tile(ne.lng, zoom);
        var yMin = lat2tile(ne.lat, zoom);
        var yMax = lat2tile(sw.lat, zoom);

        var tiles = [];
        for (var x = xMin; x <= xMax; x++) {
            for (var y = yMin; y <= yMax; y++) {
                tiles.push({ x: x, y: y, z: zoom });
            }
        }
        return tiles;
    }

    // --- Download Region ---
    // Pre-cache tiles for a given bounds and zoom range
    // progressCb(downloaded, total) called on progress
    function downloadRegion(tileUrlTemplate, bounds, minZoom, maxZoom, progressCb) {
        return new Promise(function (resolve, reject) {
            if (!('caches' in window)) {
                reject(new Error('Cache API not available'));
                return;
            }

            // Collect all tile URLs
            var allTiles = [];
            for (var z = minZoom; z <= maxZoom; z++) {
                var tiles = getTilesInBounds(bounds, z);
                tiles.forEach(function (t) {
                    var url = tileUrlTemplate
                        .replace('{z}', t.z)
                        .replace('{x}', t.x)
                        .replace('{y}', t.y)
                        .replace('{r}', '')
                        .replace('{s}', 'a'); // Pick a subdomain
                    allTiles.push(url);
                });
            }

            var total = allTiles.length;
            var downloaded = 0;
            var failed = 0;

            if (total === 0) {
                if (progressCb) progressCb(0, 0);
                resolve({ downloaded: 0, failed: 0 });
                return;
            }

            if (progressCb) progressCb(0, total);

            caches.open(CACHE_NAME).then(function (cache) {
                // Download in batches of 6
                var idx = 0;
                var BATCH = 6;

                function next() {
                    if (idx >= total) {
                        resolve({ downloaded: downloaded, failed: failed, total: total });
                        return;
                    }

                    var batch = allTiles.slice(idx, idx + BATCH);
                    idx += BATCH;

                    Promise.all(batch.map(function (url) {
                        return fetch(url).then(function (resp) {
                            if (resp.ok) {
                                downloaded++;
                                return cache.put(url, resp);
                            } else {
                                failed++;
                            }
                        }).catch(function () {
                            failed++;
                        }).then(function () {
                            if (progressCb) progressCb(downloaded + failed, total);
                        });
                    })).then(next);
                }

                next();
            }).catch(reject);
        });
    }

    // --- Cache size estimation ---
    function getCacheSize() {
        return new Promise(function (resolve) {
            if (!('caches' in window)) {
                resolve({ count: 0, bytes: 0 });
                return;
            }

            caches.open(CACHE_NAME).then(function (cache) {
                return cache.keys();
            }).then(function (keys) {
                var count = keys.length;
                // Estimate size: average tile ~15KB
                resolve({ count: count, bytes: count * 15000 });
            }).catch(function () {
                resolve({ count: 0, bytes: 0 });
            });
        });
    }

    // --- Clear cache ---
    function clearCache() {
        return new Promise(function (resolve) {
            if (!('caches' in window)) {
                resolve();
                return;
            }
            caches.delete(CACHE_NAME).then(function () {
                resolve();
            }).catch(function () {
                resolve();
            });
        });
    }

    // --- Format bytes ---
    function formatBytes(bytes) {
        if (bytes < 1024) return bytes + ' B';
        if (bytes < 1048576) return (bytes / 1024).toFixed(1) + ' KB';
        return (bytes / 1048576).toFixed(1) + ' MB';
    }

    return {
        createLayer: createLayer,
        downloadRegion: downloadRegion,
        getCacheSize: getCacheSize,
        clearCache: clearCache,
        formatBytes: formatBytes,
        CACHE_NAME: CACHE_NAME,
    };

})();
