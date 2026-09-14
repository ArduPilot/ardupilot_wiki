#!/usr/bin/env python3
"""Check that every image used by raw html is also declared to Sphinx.

Sphinx copies an image into the build only when a directive references
it. An ``<img src="../_images/logo.png">`` inside a ``.. raw:: html``
block is invisible to it, so the page asks for a file that was never
copied and shows a broken image. Pages that embed images this way
carry a hidden directive for each one::

    .. image:: ../../../images/logos/logo.png
        :scale: 0%

This check reports every raw ``<img>`` whose file has no image or figure
directive with the same file name in the same rst file. Raw blocks shown
as examples inside a literal or code block are skipped.
"""

import argparse
import pathlib
import re
import sys

# Case-insensitive, and the substitution form renders too.
RAW_HTML_RE = re.compile(r"^(\s*)\.\.\s+(?:\|[^|]+\|\s+)?raw::\s+html\s*$",
                         re.IGNORECASE)
CODE_RE = re.compile(r"^\.\.\s+(?:code|code-block|parsed-literal)::")
DIRECTIVE_RE = re.compile(r"^\.\.\s+(?:\|[^|]+\|\s+)?[\w.-]+::")
IMG_RE = re.compile(r"<img\b[^>]*\bsrc\s*=\s*[\"']([^\"']*_images/([^\"'/?#]+))", re.IGNORECASE)
IMAGE_DIRECTIVE_RE = re.compile(r"^\s*\.\.\s+(?:image|figure)::\s+(\S+)")


def indent_of(line: str) -> int:
    return len(line) - len(line.lstrip())


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


def raw_images(lines):
    """Yield (line number, file name) for every <img> in a raw html block that builds."""
    skip = None   # indent of the literal or code block being skipped
    raw = None    # indent of the raw html block being gathered

    for number, line in enumerate(lines, 1):
        blank = not line.strip()
        if skip is not None:
            if blank or indent_of(line) > skip:
                continue
            skip = None
        if raw is not None:
            if blank or indent_of(line) > raw:
                for match in IMG_RE.finditer(line):
                    yield number, match.group(2)
                continue
            raw = None
        match = RAW_HTML_RE.match(line)
        if match:
            raw = len(match.group(1))
        elif opens_literal(line):
            skip = indent_of(line)


def declared_images(lines):
    """File names of every image and figure directive target in the file."""
    names = set()
    for line in lines:
        match = IMAGE_DIRECTIVE_RE.match(line)
        if match:
            names.add(match.group(1).rsplit("/", 1)[-1])
    return names


def missing(path: pathlib.Path):
    """Yield (line number, file name) for every raw <img> with no directive."""
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    declared = declared_images(lines)
    for number, name in raw_images(lines):
        if name not in declared:
            yield number, name


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("files", nargs="+", type=pathlib.Path)
    args = parser.parse_args()

    failures = 0
    for path in args.files:
        for number, name in missing(path):
            failures += 1
            print(f"{path}:{number}: raw html uses {name} but no image or figure "
                  "directive in this file declares it, so Sphinx will not copy it")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
