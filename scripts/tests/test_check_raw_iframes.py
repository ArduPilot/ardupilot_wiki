"""Tests for scripts/check_raw_iframes.py.

    python3 scripts/tests/test_check_raw_iframes.py
"""
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from check_raw_iframes import raw_iframes  # noqa: E402

failures = 0


def check(name, ok, detail=""):
    global failures
    print(("  PASS  " if ok else "  FAIL  ") + name + (f"   {detail}" if detail else ""))
    if not ok:
        failures += 1


def flagged(rst):
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "page.rst"
        path.write_text(rst, encoding="utf-8")
        return [number for number, _ in raw_iframes(path)]


RAW = '.. raw:: html\n\n   <iframe src="{src}"></iframe>\n'

check("a raw YouTube iframe is flagged",
      flagged(RAW.format(src="https://www.youtube.com/embed/x")) == [3])
check("a raw youtube-nocookie iframe is flagged",
      flagged(RAW.format(src="https://www.youtube-nocookie.com/embed/x")) == [3])
check("a raw Vimeo iframe is flagged",
      flagged(RAW.format(src="https://player.vimeo.com/video/1")) == [3])
check("an iframe of something else is not a video embed",
      flagged(RAW.format(src="https://docs.google.com/forms/d/e/x/viewform")) == [])
check("a raw block inside a note is still built, so still flagged",
      flagged(".. note::\n\n   " + RAW.format(src="https://youtu.be/x").replace("\n", "\n   ")) == [5])
check("a commented-out embed never builds, so is not flagged",
      flagged(".. commented out for now\n\n   " +
              RAW.format(src="https://youtu.be/x").replace("\n", "\n   ")) == [])
check("a bare comment marker hides its block too",
      flagged("..\n\n   " +
              RAW.format(src="https://youtu.be/x").replace("\n", "\n   ")) == [])
check("the directive is matched whatever its case",
      flagged('.. RAW:: HTML\n\n   <iframe src="https://youtu.be/x"></iframe>\n') == [3])
check("a substitution definition builds and is flagged",
      flagged('.. |vid| raw:: html\n\n   <iframe src="https://youtu.be/x"></iframe>\n') == [3])
check("a raw block shown in a literal block is an example, not an embed",
      flagged("For example::\n\n   " + RAW.format(src="https://www.youtube.com/embed/x").replace("\n", "\n   ")) == [])
check("a raw block shown in a code-block is an example too",
      flagged(".. code-block:: rst\n\n   " + RAW.format(src="https://www.youtube.com/embed/x").replace("\n", "\n   ")) == [])
check("the block after a literal block is checked again",
      flagged("Example::\n\n   shown\n\n" + RAW.format(src="https://www.youtube.com/embed/x")) == [7])
check("the youtube directive is what we want",
      flagged(".. youtube:: dQw4w9WgXcQ\n") == [])
check("a wrapped iframe with the URL on its own line is still flagged",
      flagged('.. raw:: html\n\n   <iframe width="560"\n'
              '           src="https://www.youtube.com/embed/x">\n'
              '           </iframe>\n') == [3])
check("a wrapped iframe of something else still passes",
      flagged('.. raw:: html\n\n   <iframe\n'
              '           src="https://docs.google.com/forms/x">\n'
              '           </iframe>\n') == [])

print(f"\n{'all checks passed' if not failures else str(failures) + ' CHECK(S) FAILED'}\n")
sys.exit(1 if failures else 0)
