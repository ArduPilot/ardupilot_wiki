"""Recompress the PNGs in the built output losslessly, after Sphinx.

Each PNG is re-encoded at maximum deflate effort, decoded again and kept only
if pixel-identical and smaller. Results are cached by content hash in
.image-cache/ so later builds only touch new images.
"""

import hashlib
import io
import os
from pathlib import Path

CACHE_DIR = ".image-cache"


def _cache_path(root, data):
    return Path(root) / CACHE_DIR / (hashlib.sha256(data).hexdigest()[:32] + ".png")


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


def run(wikis, root=Path(".")):
    """Recompress every built PNG in place. Returns (images_changed, bytes_saved)."""
    root = Path(root)
    cache = root / CACHE_DIR
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
            cached = _cache_path(root, data) if cache else None
            if cached and cached.is_file():
                try:
                    best = cached.read_bytes()
                except OSError:
                    pass
            if best is None:
                best = shrink_png(data)
                if cached:
                    try:
                        _write_atomic(cached, best)
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
