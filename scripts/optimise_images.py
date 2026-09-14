"""Recompress the PNGs in the built output losslessly, after Sphinx.

Each PNG is re-encoded at maximum deflate effort, decoded again and kept only
if pixel-identical and smaller. Results are cached by content hash so later
builds only touch new images. update.py keeps its cache under --destdir, outside
the cleaned checkout.
"""

import hashlib
import io
import os
import stat
import tempfile
from pathlib import Path

CACHE_DIR = ".image-cache"
# Older caches can contain unoptimised originals from a run without Pillow.
CACHE_VERSION = b"png-cache-v2\0"
# Empty markers from v2 may represent a failed encode. Keep its PNG results,
# but only trust negative results explicitly recorded after a successful encode.
UNCHANGED = b"png-encode-ok-v1\n"


def _cache_path(cache, data):
    return cache / (hashlib.sha256(CACHE_VERSION + data).hexdigest()[:32] + ".png")


def shrink_png(data):
    """Return a smaller PNG, the original if no smaller result, or None on failure."""
    try:
        from PIL import Image
    except ImportError:
        return None

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
                return None

        return shrunk
    except Exception:
        # Nothing about one image should stop a build.
        return None


def _valid_cached_png(data, original):
    """Check complete PNG structure and decoded pixels before publishing a hit."""
    from PIL import Image

    try:
        if not data.endswith(b"\x00\x00\x00\x00IEND\xaeB\x60\x82"):
            return False
        with Image.open(io.BytesIO(data)) as image:
            if image.format != "PNG":
                return False
            image.verify()  # Check chunk CRCs as well as decoding below.
        with Image.open(io.BytesIO(data)) as image, Image.open(io.BytesIO(original)) as source:
            image.load()
            source.load()
            return ((image.mode, image.size, image.tobytes()) ==
                    (source.mode, source.size, source.tobytes()) and
                    image.convert("RGBA").tobytes() == source.convert("RGBA").tobytes())
    except Exception:
        return False


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
                    candidate = cached.read_bytes()
                    if len(candidate) < len(data) and _valid_cached_png(candidate, data):
                        best = candidate
                except OSError:
                    pass
            elif cached and cached.with_suffix(".unchanged").is_file():
                try:
                    if cached.with_suffix(".unchanged").read_bytes() == UNCHANGED:
                        best = data
                except OSError:
                    pass
            if best is None:
                best = shrink_png(data)
                if best is None:
                    # Leave the source and cache alone so a later build retries.
                    continue
                if cached:
                    try:
                        if len(best) < len(data):
                            _write_atomic(cached, best)
                            # An incremental build may see the optimised bytes
                            # next time; do not encode them a second time.
                            unchanged = _cache_path(cache, best).with_suffix(".unchanged")
                        else:
                            # Remember a successful negative result without
                            # duplicating an image that cannot be made smaller.
                            # A bad PNG entry must not hide this marker on the
                            # next build and force the same encode again.
                            cached.unlink(missing_ok=True)
                            unchanged = cached.with_suffix(".unchanged")
                        _write_atomic(unchanged, UNCHANGED)
                    except OSError:
                        pass

            if len(best) < len(data):
                _write_atomic(image, best)
                changed += 1
                saved += len(data) - len(best)

    return changed, saved


def _write_atomic(path, data):
    """Publish a durable complete file without sharing a temporary name."""
    try:
        mode = stat.S_IMODE(path.stat().st_mode)
    except FileNotFoundError:
        mode = 0o644
    fd, name = tempfile.mkstemp(dir=path.parent, prefix="." + path.name + ".", suffix=".pngtmp")
    tmp = Path(name)
    try:
        with os.fdopen(fd, "wb") as output:
            # mkstemp defaults to 0600; built images must remain web-readable.
            os.chmod(tmp, mode)
            output.write(data)
            output.flush()
            os.fsync(output.fileno())
        os.replace(tmp, path)
        if os.name == "posix":
            directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
    finally:
        tmp.unlink(missing_ok=True)
