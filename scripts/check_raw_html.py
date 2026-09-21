#!/usr/bin/env python3
"""Check that the HTML inside ``.. raw:: html`` blocks has balanced tags.

Browsers repair a mismatched or missing closing tag, so a page can look
fine while its markup is broken. Anything that parses the built page
strictly, like an epub reader, rejects the whole page instead. The check
walks every raw html block of a file in order, since a table is often
opened in one block and closed in a later one, and reports:

- a closing tag with no open tag to match, such as ``</tr>`` twice
- a closing tag that skips over a still open one, such as ``<td>`` closed by ``</th>``
- a tag still open at the end of the file
- a tag that is not an HTML element, which usually means a literal ``<``
  in text, such as ``<PARAM_NAME>``, that needs to be written ``&lt;``

Raw blocks shown as examples inside a literal or code block are skipped.
"""

import argparse
import pathlib
import re
import sys
from html.parser import HTMLParser

# Case-insensitive, and the substitution form renders too.
RAW_HTML_RE = re.compile(r"^(\s*)\.\.\s+(?:\|[^|]+\|\s+)?raw::\s+html\s*$",
                         re.IGNORECASE)
CODE_RE = re.compile(r"^\.\.\s+(?:code|code-block|parsed-literal)::")
DIRECTIVE_RE = re.compile(r"^\.\.\s+(?:\|[^|]+\|\s+)?[\w.-]+::")

# Elements that never take a closing tag.
VOID = {"area", "base", "br", "col", "embed", "hr", "img", "input", "link",
        "meta", "param", "source", "track", "wbr"}

# Every element of the HTML living standard plus the few obsolete ones the wiki uses.
ELEMENTS = VOID | set("""
a abbr address article aside audio b bdi bdo blockquote body button canvas
caption center cite code colgroup data datalist dd del details dfn dialog div
dl dt em fieldset figcaption figure font footer form h1 h2 h3 h4 h5 h6 head
header hgroup html i iframe ins kbd label legend li main map mark menu meter
nav noscript object ol optgroup option output p picture pre progress q rp rt
ruby s samp script search section select slot small span strike strong style
sub summary sup table tbody td template textarea tfoot th thead time title tr
tt u ul var video
""".split())


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


def raw_blocks(path: pathlib.Path):
    """Yield (first line number, text) for every raw html block that builds."""
    skip = None   # indent of the literal or code block being skipped
    raw = None    # indent of the raw html block being gathered
    start = 0
    held = []

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
            yield start, "\n".join(held)
            raw, held = None, []
        match = RAW_HTML_RE.match(line)
        if match:
            raw = len(match.group(1))
        elif opens_literal(line):
            skip = indent_of(line)
    if raw is not None and held:
        yield start, "\n".join(held)


class TagBalance(HTMLParser):
    """Track open tags across everything fed in and record each mismatch."""

    def __init__(self):
        super().__init__()
        self.stack = []     # (tag, parser line) for every open element
        self.errors = []    # (parser line, message)

    def handle_starttag(self, tag, attrs):
        if tag not in ELEMENTS:
            self.errors.append((self.getpos()[0], f"<{tag}> is not an HTML tag; write a literal < as &lt;"))
            return
        if tag not in VOID:
            self.stack.append((tag, self.getpos()[0]))

    def handle_startendtag(self, tag, attrs):
        if tag not in ELEMENTS:
            self.handle_starttag(tag, attrs)

    def handle_endtag(self, tag):
        line = self.getpos()[0]
        if tag in VOID:
            return
        if not any(open_tag == tag for open_tag, _ in self.stack):
            self.errors.append((line, f"</{tag}> closes nothing"))
            return
        while self.stack[-1][0] != tag:
            open_tag, opened = self.stack.pop()
            self.errors.append((opened, f"<{open_tag}> is still open when </{tag}> closes it"))
        self.stack.pop()

    def finish(self):
        self.close()
        for open_tag, opened in self.stack:
            self.errors.append((opened, f"<{open_tag}> is never closed"))


def problems(path: pathlib.Path):
    """Yield (line number, message) for every tag problem in the file's raw html."""
    parser = TagBalance()
    lines = []      # parser line index -> file line number
    for start, text in raw_blocks(path):
        lines.extend(range(start, start + text.count("\n") + 1))
        parser.feed(text + "\n")
    if not lines:
        return
    parser.finish()
    for index, message in sorted(parser.errors):
        yield lines[min(index, len(lines)) - 1], message


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("files", nargs="+", type=pathlib.Path)
    args = parser.parse_args()

    failures = 0
    for path in args.files:
        for number, message in problems(path):
            failures += 1
            print(f"{path}:{number}: {message}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
