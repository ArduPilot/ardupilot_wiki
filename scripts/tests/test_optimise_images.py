"""Tests for scripts/optimise_images.

    python3 scripts/tests/test_optimise_images.py
"""
import io
import os
import sys
import tempfile
from pathlib import Path

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


def check_bad_input_is_returned_untouched():
    original = noisy_png()
    for name, data in [("garbage bytes", b"not an image at all"),
                       ("a truncated PNG", original[:40])]:
        check(f"{name} comes back untouched", oi.shrink_png(data) == data)

    import builtins
    real_import = builtins.__import__

    def no_pillow(name, *a, **k):
        if name.startswith("PIL"):
            raise ImportError("no Pillow")
        return real_import(name, *a, **k)

    builtins.__import__ = no_pillow
    try:
        check("without Pillow the original is returned", oi.shrink_png(original) == original)
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


def main():
    print("\nlossless PNG pass\n")
    try:
        import PIL  # noqa: F401
    except ImportError:
        print("  Pillow is not installed; nothing can be verified.\n")
        sys.exit(1)

    check_shrinks_without_changing_pixels()
    check_modes_survive()
    check_bad_input_is_returned_untouched()
    check_pass_over_a_built_tree()

    print()
    if failures:
        print(f"{failures} CHECK(S) FAILED\n")
        sys.exit(1)
    print("all checks passed\n")


if __name__ == "__main__":
    main()
