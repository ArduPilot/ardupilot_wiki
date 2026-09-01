#!/usr/bin/env python3
"""Check that video embeds use the youtube or vimeo directive, not a raw <iframe>.

The lazy_youtube extension makes every ``.. youtube::`` and ``.. vimeo::``
embed load lazily. A video iframe pasted into a ``.. raw:: html`` block
bypasses it::

    .. Bad:
       .. raw:: html

          <iframe src="https://www.youtube.com/embed/dQw4w9WgXcQ"></iframe>

    .. Good:
       .. youtube:: dQw4w9WgXcQ

Iframes of anything else are allowed, and so is a raw block shown as an
example inside a literal or code block.
"""

import argparse
import pathlib
import re
import sys

# Case-insensitive, and the substitution form renders too.
RAW_HTML_RE = re.compile(r"^(\s*)\.\.\s+(?:\|[^|]+\|\s+)?raw::\s+html\s*$",
                         re.IGNORECASE)
CODE_RE = re.compile(r"^\.\.\s+(?:code|code-block|parsed-literal)::")
VIDEO_RE = re.compile(
    r"<iframe[^>]*\b(?:youtube(?:-nocookie)?\.com|youtu\.be|vimeo\.com|peertube)", re.IGNORECASE)


def indent_of(line: str) -> int:
    return len(line) - len(line.lstrip())


DIRECTIVE_RE = re.compile(r"^\.\.\s+(?:\|[^|]+\|\s+)?[\w.-]+::")


def opens_literal(line: str) -> bool:
    """A block whose body never reaches the built page: a code directive or
    a true comment. Other directives still build, nested raw included."""
    text = line.strip()
    if text == "..":
        return True
    if text.startswith(".. "):
        if CODE_RE.match(text):
            return True
        # No directive marker makes it a comment; a directive's body builds.
        return not DIRECTIVE_RE.match(text)
    return text.endswith("::")


def raw_iframes(path: pathlib.Path):
    """Yield (line number, snippet) for every video <iframe> in a built raw html block."""
    skip = None   # indent of the literal or code block being skipped
    raw = None    # indent of the raw html block being gathered
    start = 0
    held = []

    def matches():
        # The whole block at once: an iframe wrapped over lines is one tag.
        text = "\n".join(held)
        for m in VIDEO_RE.finditer(text):
            yield (start + text.count("\n", 0, m.start()),
                   " ".join(m.group(0).split()))

    for number, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        blank = not line.strip()
        if skip is not None:
            if blank or indent_of(line) > skip:
                continue
            skip = None
        if raw is not None:
            if blank or indent_of(line) > raw:
                if not held:
                    start = number
                held.append(line)
                continue
            yield from matches()
            raw, held = None, []
        match = RAW_HTML_RE.match(line)
        if match:
            raw = len(match.group(1))
        elif opens_literal(line):
            skip = indent_of(line)
    if raw is not None:
        yield from matches()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("files", nargs="+", type=pathlib.Path)
    args = parser.parse_args()

    failures = 0
    for path in args.files:
        for number, line in raw_iframes(path):
            failures += 1
            print(f"{path}:{number}: video <iframe> in a raw html block; "
                  "use the youtube or vimeo directive")
            print(f"    {line[:100]}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
