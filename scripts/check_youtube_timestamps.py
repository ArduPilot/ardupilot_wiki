#!/usr/bin/env python3
"""Check YouTube directives for timestamps embedded in the video ID.

Timestamps must not be appended to the video ID argument.  They belong in
the ``url_parameters`` option instead::

    .. Bad (timestamp in video ID):
       .. youtube:: dQw4w9WgXcQ?t=42
       .. youtube:: dQw4w9WgXcQ&t=42

    .. Good (timestamp as option):
       .. youtube:: dQw4w9WgXcQ
           :url_parameters: ?start=42

Detected patterns: ``[?&](t|start)=<digits>`` anywhere in the video ID.
"""

import argparse
import pathlib
import re
import sys

# Matches a youtube directive whose argument contains a URL timestamp param.
# The directive may start with one or more spaces before ``..``.
YOUTUBE_TIMESTAMP_RE = re.compile(
    r"^\s*\.\.\s+youtube::\s+\S+[?&](t|start)=\d+",
)


def check_file(path: pathlib.Path) -> list[str]:
    errors: list[str] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except UnicodeDecodeError as exc:
        return [f"{path}: UnicodeDecodeError: {exc}"]
    for i, line in enumerate(lines, start=1):
        if YOUTUBE_TIMESTAMP_RE.match(line):
            errors.append(
                f"{path}:{i}: youtube timestamp must use the url_parameters option,"
                " not the video ID (e.g. :url_parameters: ?start=<seconds>)"
            )
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check YouTube directives for timestamps embedded in the video ID"
    )
    parser.add_argument("files", nargs="+", help="RST files to check")
    args = parser.parse_args()

    all_errors: list[str] = []
    for file_name in args.files:
        path = pathlib.Path(file_name)
        if path.suffix.lower() != ".rst":
            continue
        all_errors.extend(check_file(path))

    if all_errors:
        print("YouTube timestamp must not be in the video ID:")
        print("\n".join(all_errors))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
