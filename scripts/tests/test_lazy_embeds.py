"""Tests for scripts/lazy_embeds.py: touch only YouTube iframes, be idempotent,
never leave a page half-written.

    python3 scripts/tests/test_lazy_embeds.py
"""

import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.lazy_embeds import make_embeds_lazy, run  # noqa: E402

failures = 0


def check(name, ok, detail=""):
    global failures
    print(("  PASS  " if ok else "  FAIL  ") + name + (f"   {detail}" if detail else ""))
    if not ok:
        failures += 1


IFRAME = '<iframe allowfullscreen="true" src="https://www.youtube.com/embed/abc" style="border: 0">'


def main():
    print("\nlazy_embeds\n")

    out = make_embeds_lazy(IFRAME)
    check("a youtube iframe gains loading=lazy", 'loading="lazy"' in out, out)
    check("the src is preserved", "youtube.com/embed/abc" in out)

    check("running it twice changes nothing the second time",
          make_embeds_lazy(out) == out)

    check("an iframe that already sets loading is left alone",
          make_embeds_lazy('<iframe loading="eager" src="https://www.youtube.com/embed/x">') ==
          '<iframe loading="eager" src="https://www.youtube.com/embed/x">')

    other = '<iframe src="https://example.com/thing">'
    check("a non-youtube iframe is untouched", make_embeds_lazy(other) == other)

    check("youtube-nocookie is also matched",
          'loading="lazy"' in make_embeds_lazy(
              '<iframe src="https://www.youtube-nocookie.com/embed/x">'))

    # The whole-tree pass leaves no temp files behind.
    with tempfile.TemporaryDirectory() as d:
        root = Path(d)
        page = root / "wikix" / "build" / "html" / "docs" / "p.html"
        page.parent.mkdir(parents=True)
        page.write_text("<html>" + IFRAME + "</html>", encoding="utf-8")
        untouched = root / "wikix" / "build" / "html" / "plain.html"
        untouched.write_text("<html>no video</html>", encoding="utf-8")

        n = run(["wikix"], root)
        check("run rewrites the page with an embed", n == 1, f"{n} changed")
        check("the page on disk is now lazy",
              'loading="lazy"' in page.read_text(encoding="utf-8"))
        check("a page with no embed is not counted",
              "no video" in untouched.read_text(encoding="utf-8"))
        check("no .lazytmp files are left behind",
              not list(root.rglob("*.lazytmp")))

        # Idempotent over the whole tree too.
        check("a second run changes nothing", run(["wikix"], root) == 0)

    print()
    if failures:
        print(f"{failures} CHECK(S) FAILED\n")
        sys.exit(1)
    print("all checks passed\n")


if __name__ == "__main__":
    main()
