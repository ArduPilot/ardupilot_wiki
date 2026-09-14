"""Tests for scripts/optimise_images.

    python3 scripts/tests/test_optimise_images.py
"""
import io
import os
import shutil
import subprocess
import sys
import tempfile
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import scripts.optimise_images as oi  # noqa: E402

failures = 0


def check(name, ok, detail=""):
    global failures
    print(("  PASS  " if ok else "  FAIL  ") + name + (f"   {detail}" if detail else ""))
    if not ok:
        failures += 1


def noisy_png(w=240, h=180):
    """A PNG written at low effort, as if straight out of a tool."""
    from PIL import Image
    im = Image.new("RGB", (w, h))
    px = im.load()
    for y in range(h):
        for x in range(w):
            px[x, y] = ((x * 7) % 256, (y * 5) % 256, ((x + y) * 3) % 256)
    out = io.BytesIO()
    im.save(out, "PNG", compress_level=1)
    return out.getvalue()


def pixels(data):
    from PIL import Image
    with Image.open(io.BytesIO(data)) as im:
        im.load()
        return im.mode, im.size, im.tobytes()


def check_shrinks_without_changing_pixels():
    original = noisy_png()
    shrunk = oi.shrink_png(original)
    check("a never-optimised PNG gets smaller", len(shrunk) < len(original),
          f"{len(original)} -> {len(shrunk)}")
    check("and decodes to the same pixels", pixels(shrunk) == pixels(original))
    # The "already optimal" branch, which 56% of the real corpus takes.
    check("an already-optimal PNG is returned unchanged", oi.shrink_png(shrunk) == shrunk)


def check_modes_survive():
    """Pillow can drop an alpha channel or flatten a palette on save."""
    from PIL import Image
    out = io.BytesIO()
    rgba = Image.new("RGBA", (64, 64), (255, 0, 0, 0))
    for i in range(64):
        rgba.putpixel((i, i), (0, 255, 0, 128))
    rgba.save(out, "PNG", compress_level=1)
    transparent = out.getvalue()
    check("an alpha channel is preserved",
          pixels(oi.shrink_png(transparent)) == pixels(transparent))

    out = io.BytesIO()
    Image.new("P", (64, 64)).save(out, "PNG", compress_level=1)
    check("a palette image keeps its mode", pixels(oi.shrink_png(out.getvalue()))[0] == "P")


def check_failed_encode_is_signalled():
    original = noisy_png()
    for name, data in [("garbage bytes", b"not an image at all"),
                       ("a truncated PNG", original[:40])]:
        check(f"{name} signals an encode failure", oi.shrink_png(data) is None)

    import builtins
    real_import = builtins.__import__

    def no_pillow(name, *a, **k):
        if name.startswith("PIL"):
            raise ImportError("no Pillow")
        return real_import(name, *a, **k)

    builtins.__import__ = no_pillow
    try:
        check("without Pillow encoding signals failure", oi.shrink_png(original) is None)
    finally:
        builtins.__import__ = real_import


def check_pass_over_a_built_tree():
    original = noisy_png()
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        images = root / "rover" / "build" / "html" / "_images"
        images.mkdir(parents=True)
        (images / "diagram.png").write_bytes(original)
        (images / "photo.jpg").write_bytes(b"\xff\xd8\xff\xe0 not really a jpeg")
        (images / "broken.png").write_bytes(b"\x89PNG\r\n\x1a\n truncated")
        jpg = (images / "photo.jpg").read_bytes()
        broken = (images / "broken.png").read_bytes()

        changed, saved = oi.run(["rover"], root)
        check("the built PNG was rewritten smaller", changed == 1 and saved > 0,
              f"{changed} changed, {saved} bytes saved")
        check("its pixels are unchanged",
              pixels((images / "diagram.png").read_bytes()) == pixels(original))
        check("a non-PNG is untouched", (images / "photo.jpg").read_bytes() == jpg)
        check("an unreadable PNG is untouched", (images / "broken.png").read_bytes() == broken)
        # A stray .pngtmp would be published and packed into the archives.
        check("no temp files are left behind", not list(images.glob("*.pngtmp")))

        optimised = (images / "diagram.png").read_bytes()
        check("a second pass changes nothing", oi.run(["rover"], root) == (0, 0))
        check("a wiki with no build output is skipped", oi.run(["copter"], root) == (0, 0))

        (images / "diagram.png").write_bytes(original)
        changed, _ = oi.run(["rover"], root)
        check("a restored original is served from the cache",
              changed == 1 and (images / "diagram.png").read_bytes() == optimised)

        # A build server that cannot write the cache must still build.
        (images / "diagram.png").write_bytes(original)
        cache_dir = root / oi.CACHE_DIR
        mode = cache_dir.stat().st_mode
        os.chmod(cache_dir, 0o500)
        try:
            changed, _ = oi.run(["rover"], root)
            intact = pixels((images / "diagram.png").read_bytes()) == pixels(original)
            check("a read-only cache does not stop the pass", changed == 1 and intact)
        finally:
            os.chmod(cache_dir, mode)


def check_cache_survives_clean_builds():
    original = noisy_png()
    edited = noisy_png(w=241)
    with tempfile.TemporaryDirectory() as td:
        checkout = Path(td) / "checkout"
        checkout.mkdir()
        cache = Path(td) / "published" / "offline.cache" / "images"
        subprocess.run(["git", "init", "--quiet", str(checkout)], check=True)

        def write_image(wiki, data):
            image = checkout / wiki / "build" / "html" / "_images" / "diagram.png"
            image.parent.mkdir(parents=True, exist_ok=True)
            image.write_bytes(data)
            return image

        rover = write_image("rover", original)
        plane = write_image("plane", original)
        with patch.object(oi, "shrink_png", wraps=oi.shrink_png) as encode:
            oi.run(["rover", "plane"], checkout, cache_dir=cache)
            check("identical images in different wikis are encoded only once",
                  encode.call_count == 1, str(encode.call_count))
        optimised = rover.read_bytes()
        check("both wikis receive the same smaller image",
              plane.read_bytes() == optimised and len(optimised) < len(original))

        with patch.object(oi, "shrink_png", wraps=oi.shrink_png) as encode:
            result = oi.run(["rover", "plane"], checkout, cache_dir=cache)
            check("an incremental pass does not re-encode its own output",
                  result == (0, 0) and encode.call_count == 0)

        subprocess.run(["git", "-C", str(checkout), "clean", "-fdx"],
                       check=True, stdout=subprocess.DEVNULL)
        check("production-style git clean removes the build but preserves the cache",
              not rover.exists() and any(cache.iterdir()))
        rover = write_image("rover", original)
        plane = write_image("plane", original)
        with patch.object(oi, "shrink_png", wraps=oi.shrink_png) as encode:
            oi.run(["rover", "plane"], checkout, cache_dir=cache)
            check("a clean rebuild reuses cached results without encoding",
                  encode.call_count == 0 and rover.read_bytes() == optimised and
                  plane.read_bytes() == optimised)

        # Keep the timestamp: the byte hash, not mtime, must notice the edit.
        before = plane.stat()
        plane.write_bytes(edited)
        os.utime(plane, ns=(before.st_atime_ns, before.st_mtime_ns))
        with patch.object(oi, "shrink_png", wraps=oi.shrink_png) as encode:
            oi.run(["rover", "plane"], checkout, cache_dir=cache)
            check("only edited bytes are encoded, even with an unchanged mtime",
                  encode.call_count == 1 and encode.call_args.args[0] == edited)


def check_no_pillow_does_not_poison_cache():
    import builtins
    real_import = builtins.__import__

    def no_pillow(name, *args, **kwargs):
        if name.startswith("PIL"):
            raise ImportError("no Pillow")
        return real_import(name, *args, **kwargs)

    original = noisy_png()
    with tempfile.TemporaryDirectory() as td:
        root = Path(td) / "checkout"
        image = root / "rover" / "build" / "html" / "_images" / "diagram.png"
        image.parent.mkdir(parents=True)
        image.write_bytes(original)
        cache = Path(td) / "cache"
        with patch("builtins.__import__", side_effect=no_pillow), \
                patch.object(Path, "rglob", side_effect=AssertionError("image scan")):
            check("without Pillow the pass skips the scan and cache writes",
                  oi.run(["rover"], root, cache_dir=cache) == (0, 0) and
                  not cache.exists())
        with patch.object(oi, "shrink_png", wraps=oi.shrink_png) as encode:
            oi.run(["rover"], root, cache_dir=cache)
            check("installing Pillow later still optimises the original",
                  encode.call_count == 1 and len(image.read_bytes()) < len(original))

        # Legacy runs without Pillow cached original bytes as a PNG result.
        # Those entries must not suppress optimisation in the new format.
        shutil.rmtree(cache)
        cache.mkdir()
        import hashlib
        (cache / (hashlib.sha256(original).hexdigest()[:32] + ".png")).write_bytes(original)
        image.write_bytes(original)
        with patch.object(oi, "shrink_png", wraps=oi.shrink_png) as encode:
            oi.run(["rover"], root, cache_dir=cache)
            check("legacy no-Pillow results cannot suppress optimisation",
                  encode.call_count == 1 and len(image.read_bytes()) < len(original))


def check_negative_results_are_small():
    optimised = oi.shrink_png(noisy_png())
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        image = root / "rover" / "build" / "html" / "_images" / "diagram.png"
        image.parent.mkdir(parents=True)
        image.write_bytes(optimised)
        cache = root / "cache"
        oi.run(["rover"], root, cache_dir=cache)
        entries = list(cache.iterdir())
        check("an image that cannot shrink gets a small success marker, not another copy",
              len(entries) == 1 and entries[0].read_bytes() == oi.UNCHANGED)
        with patch.object(oi, "shrink_png", wraps=oi.shrink_png) as encode:
            oi.run(["rover"], root, cache_dir=cache)
            check("an unchanged negative result is not recompressed",
                  encode.call_count == 0 and image.read_bytes() == optimised)


def check_failed_encode_is_retried():
    from PIL import Image
    original = noisy_png()
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        image = root / "rover/build/html/_images/diagram.png"
        image.parent.mkdir(parents=True)
        image.write_bytes(original)
        cache = root / "cache"
        with patch.object(Image.Image, "save", side_effect=MemoryError("temporary failure")):
            result = oi.run(["rover"], root, cache_dir=cache)
        check("an encode failure preserves the image without recording a negative result",
              result == (0, 0) and image.read_bytes() == original and not list(cache.iterdir()))
        with patch.object(oi, "shrink_png", wraps=oi.shrink_png) as encode:
            oi.run(["rover"], root, cache_dir=cache)
        check("the next build retries a transient failure successfully",
              encode.call_count == 1 and len(image.read_bytes()) < len(original))

        image.write_bytes(original)
        shutil.rmtree(cache)
        cache.mkdir()
        oi._cache_path(cache, original).with_suffix(".unchanged").write_bytes(b"")
        with patch.object(oi, "shrink_png", wraps=oi.shrink_png) as encode:
            oi.run(["rover"], root, cache_dir=cache)
        check("old empty markers cannot preserve an earlier encode failure",
              encode.call_count == 1 and len(image.read_bytes()) < len(original))


def check_corrupt_cache_is_repaired():
    from PIL import Image
    original = noisy_png()
    optimised = oi.shrink_png(original)
    wrong = io.BytesIO()
    Image.new("RGB", (240, 180), "red").save(wrong, "PNG", optimize=True)
    bad_crc = bytearray(optimised)
    bad_crc[29] ^= 1  # IHDR CRC; signature and IEND still intact.
    cases = [("empty", b""), ("truncated", optimised[:len(optimised) // 2]),
             ("missing IEND CRC", optimised[:-1]), ("bad CRC", bytes(bad_crc)),
             ("different pixels", wrong.getvalue())]
    for name, damaged in cases:
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            image = root / "rover/build/html/_images/diagram.png"
            image.parent.mkdir(parents=True)
            image.write_bytes(original)
            cache = root / "cache"
            cache.mkdir()
            cached = oi._cache_path(cache, original)
            cached.write_bytes(damaged)
            # Even a positive marker must not hide the corrupt PNG entry.
            cached.with_suffix(".unchanged").write_bytes(oi.UNCHANGED)
            with patch.object(oi, "shrink_png", wraps=oi.shrink_png) as encode:
                oi.run(["rover"], root, cache_dir=cache)
            check(f"a {name} cache entry is regenerated before publication",
                  encode.call_count == 1 and image.read_bytes() == optimised and
                  cached.read_bytes() == optimised)
            image.write_bytes(original)
            with patch.object(oi, "shrink_png", side_effect=AssertionError("unnecessary encode")):
                oi.run(["rover"], root, cache_dir=cache)
            check(f"the repaired {name} entry is reused without encoding",
                  image.read_bytes() == optimised)

    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        image = root / "rover/build/html/_images/diagram.png"
        image.parent.mkdir(parents=True)
        image.write_bytes(optimised)
        cache = root / "cache"
        cache.mkdir()
        cached = oi._cache_path(cache, optimised)
        cached.write_bytes(b"")
        oi.run(["rover"], root, cache_dir=cache)
        check("a corrupt entry for an optimal image is replaced by a success marker",
              not cached.exists() and cached.with_suffix(".unchanged").read_bytes() == oi.UNCHANGED)
        with patch.object(oi, "shrink_png", side_effect=AssertionError("repeated encode")):
            oi.run(["rover"], root, cache_dir=cache)
        check("an optimal image with a repaired cache is not encoded again",
              image.read_bytes() == optimised)


def check_atomic_writes():
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        target = root / "image.png"
        target.write_bytes(b"previous image")
        target.chmod(0o644)
        for operation in ("fsync", "replace"):
            with patch.object(oi.os, operation, side_effect=OSError("write failure")):
                try:
                    oi._write_atomic(target, b"new image")
                except OSError:
                    pass
                else:
                    check(f"{operation} failure is reported", False)
            check(f"{operation} failure preserves the old image and removes temporary files",
                  target.read_bytes() == b"previous image" and list(root.iterdir()) == [target])

        barrier = threading.Barrier(2)
        replace = os.replace

        def simultaneous_replace(source, destination):
            barrier.wait(timeout=10)
            replace(source, destination)

        payloads = [b"a" * 8192, b"b" * 16384]
        with patch.object(oi.os, "replace", side_effect=simultaneous_replace), \
                ThreadPoolExecutor(max_workers=2) as pool:
            list(pool.map(lambda body: oi._write_atomic(target, body), payloads))
        check("concurrent writers publish one complete file without temporary-name collisions",
              target.read_bytes() in payloads and list(root.iterdir()) == [target])
        check("atomic replacement preserves web-readable image permissions",
              target.stat().st_mode & 0o777 == 0o644)
        fresh = root / "fresh.png"
        oi._write_atomic(fresh, b"new cached image")
        check("new cache files are readable when copied into the web tree",
              fresh.stat().st_mode & 0o777 == 0o644)


def main():
    print("\nlossless PNG pass\n")
    try:
        import PIL  # noqa: F401
    except ImportError:
        print("  Pillow is not installed; nothing can be verified.\n")
        sys.exit(1)

    check_shrinks_without_changing_pixels()
    check_modes_survive()
    check_failed_encode_is_signalled()
    check_pass_over_a_built_tree()
    check_cache_survives_clean_builds()
    check_no_pillow_does_not_poison_cache()
    check_negative_results_are_small()
    check_failed_encode_is_retried()
    check_corrupt_cache_is_repaired()
    check_atomic_writes()

    print()
    if failures:
        print(f"{failures} CHECK(S) FAILED\n")
        sys.exit(1)
    print("all checks passed\n")


if __name__ == "__main__":
    main()
