"""Tests for the optional archive image downsizing in build_offline_artifacts.

    python3 scripts/tests/test_image_resize.py
"""
import io
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import scripts.build_offline_artifacts as b  # noqa: E402

failures = 0


def check(name, ok, detail=""):
    global failures
    print(("  PASS  " if ok else "  FAIL  ") + name + (f"   {detail}" if detail else ""))
    if not ok:
        failures += 1


def png(w, h):
    from PIL import Image
    out = io.BytesIO()
    Image.new("RGB", (w, h), (123, 200, 50)).save(out, "PNG")
    return out.getvalue()


def jpg(w, h):
    from PIL import Image
    out = io.BytesIO()
    Image.new("RGB", (w, h), (30, 90, 200)).save(out, "JPEG", quality=95)
    return out.getvalue()


def dims(data):
    from PIL import Image
    return Image.open(io.BytesIO(data)).size


def main():
    print("\nimage resize\n")

    check("disabled by default returns None",
          b.IMAGE_MAX_DIM == 0 and b.downsize_image(jpg(4000, 4000), "x.jpg") is None)

    b.IMAGE_MAX_DIM = 1600
    try:
        big = jpg(4000, 3000)
        small = b.downsize_image(big, "photo.jpg")
        check("a large jpeg is downsized", small is not None and len(small) < len(big),
              f"{len(big)} -> {len(small) if small else 'None'}")
        check("and resized to the cap on its long side",
              small is not None and max(dims(small)) == 1600, str(dims(small)) if small else "")

        bigpng = png(3000, 2000)
        spng = b.downsize_image(bigpng, "screenshot.png")
        check("a large png is downsized and stays png",
              spng is not None and spng[:8] == b"\x89PNG\r\n\x1a\n",
              f"{len(bigpng)} -> {len(spng) if spng else 'None'}")

        check("a small image is left alone (None)",
              b.downsize_image(jpg(800, 600), "small.jpg") is None)
        check("a non-image is left alone (None)",
              b.downsize_image(b"\x00\x01\x02", "thing.woff2") is None)
        check("garbage bytes never raise, just None",
              b.downsize_image(b"not an image", "broken.jpg") is None)
    finally:
        b.IMAGE_MAX_DIM = 0

    print()
    if failures:
        print(f"{failures} CHECK(S) FAILED\n")
        sys.exit(1)
    print("all checks passed\n")


if __name__ == "__main__":
    main()
