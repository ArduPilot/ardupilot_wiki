"""Recompress the PNGs in the built output losslessly, after Sphinx.

Each PNG is re-encoded at maximum deflate effort, decoded again and kept only
if pixel-identical and smaller. Results are cached by content hash so later
builds only touch new images. update.py keeps its cache under --destdir, outside
the cleaned checkout.
"""

import hashlib
import io
import os
from pathlib import Path

CACHE_DIR = ".image-cache"
# Older caches can contain unoptimised originals from a run without Pillow.
CACHE_VERSION = b"png-cache-v2\0"


def _cache_path(cache, data):
    return cache / (hashlib.sha256(CACHE_VERSION + data).hexdigest()[:32] + ".png")


def shrink_png(data):
    """Return a smaller, pixel-identical PNG, or the original if there isn't one."""
    try:
        from PIL import Image
    except ImportError:
        return data

    try:
        with Image.open(io.BytesIO(data)) as im:
            im.load()
            mode, size, pixels = im.mode, im.size, im.tobytes()
            out = io.BytesIO()
            im.save(out, format="PNG", optimize=True, compress_level=9)
        shrunk = out.getvalue()

        if len(shrunk) >= len(data):
            return data

        # Prove the pixels survived rather than trusting the encoder.
        with Image.open(io.BytesIO(shrunk)) as check:
            check.load()
            if (check.mode, check.size) != (mode, size) or check.tobytes() != pixels:
                return data

        return shrunk
    except Exception:
        # Nothing about one image should stop a build.
        return data


def run(wikis, root=Path("."), cache_dir=None):
    """Recompress built PNGs, optionally caching outside root. Return counts/bytes."""
    try:
        import PIL.Image  # noqa: F401
    except ImportError:
        # Do not fill the cache with originals or scan the image corpus when
        # there is no encoder. A later Pillow installation must do real work.
        return 0, 0

    root = Path(root)
    cache = Path(cache_dir) if cache_dir is not None else root / CACHE_DIR
    try:
        cache.mkdir(parents=True, exist_ok=True)
    except OSError:
        cache = None

    changed = 0
    saved = 0
    for wiki in wikis:
        image_root = root / wiki / "build" / "html" / "_images"
        if not image_root.is_dir():
            continue
        for image in sorted(image_root.rglob("*.png")):
            try:
                data = image.read_bytes()
            except OSError:
                continue

            best = None
            cached = _cache_path(cache, data) if cache else None
            if cached and cached.is_file():
                try:
                    best = cached.read_bytes()
                except OSError:
                    pass
            elif cached and cached.with_suffix(".unchanged").is_file():
                best = data
            if best is None:
                best = shrink_png(data)
                if cached:
                    try:
                        if len(best) < len(data):
                            _write_atomic(cached, best)
                            # An incremental build may see the optimised bytes
                            # next time; do not encode them a second time.
                            unchanged = _cache_path(cache, best).with_suffix(".unchanged")
                        else:
                            # Remember a negative result without duplicating
                            # an image that cannot be made smaller.
                            unchanged = cached.with_suffix(".unchanged")
                        _write_atomic(unchanged, b"")
                    except OSError:
                        pass

            if len(best) < len(data):
                _write_atomic(image, best)
                changed += 1
                saved += len(data) - len(best)

    return changed, saved


def _write_atomic(path, data):
    """Write via a temp file and rename, so an interrupted build cannot truncate an image."""
    tmp = path.with_name(path.name + ".pngtmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)
