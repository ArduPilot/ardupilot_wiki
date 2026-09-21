#!/usr/bin/env python3
"""Check ReStructuredText literal block markers for blank lines before and after."""

import argparse
import pathlib
import re

MARKER_RE = re.compile(r"^\s*::\s*$")


def check_file(path):
    errors = []
    lines = path.read_text(encoding="utf-8").splitlines()

    for index, line in enumerate(lines):
        if MARKER_RE.match(line):
            prev_blank = index == 0 or lines[index - 1].strip() == ""
            next_blank = index + 1 >= len(lines) or lines[index + 1].strip() == ""
            if not prev_blank:
                errors.append(
                    f"{path}:{index + 1}: literal block marker must have a blank line before it"
                )
            if not next_blank:
                errors.append(
                    f"{path}:{index + 1}: literal block marker must have a blank line after it"
                )
    return errors


def main():
    parser = argparse.ArgumentParser(description="Check ReST :: code-block spacing")
    parser.add_argument("files", nargs="+", help="RST files to check")
    args = parser.parse_args()

    all_errors = []
    for file_name in args.files:
        path = pathlib.Path(file_name)
        if path.suffix.lower() != ".rst":
            continue
        all_errors.extend(check_file(path))

    if all_errors:
        print("Invalid ReST literal-block spacing detected:")
        print("\n".join(all_errors))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
