#!/usr/bin/env python3
"""Check formatting around :ref: directives that Sphinx does not warn about.

Two rules are enforced:
  1. No alphanumeric/underscore/colon character directly before ``:ref:``
     (missing space before the role).
  2. No whitespace between ``:ref:`` and its opening backtick.
  3. No space between a closing ``:ref:`` backtick and a punctuation character
     ``.``, ``,`` or ``:``.  (trailing space before punctuation).

"""

import argparse
import pathlib
import re
import sys

CHARACTER_BEFORE_RE = re.compile(r"[a-zA-Z0-9_:]:ref:")
CHARACTER_AFTER_RE = re.compile(r":ref:[^`]")
SPACE_BEFORE_PUNCT_RE = re.compile(r"(:ref:`.*?`[_]{0,2}) ([\.,:])")


def check_file(path: pathlib.Path) -> list[str]:
    errors: list[str] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except UnicodeDecodeError as exc:
        return [f"{path}: UnicodeDecodeError: {exc}"]
    for i, line in enumerate(lines, start=1):
        if CHARACTER_BEFORE_RE.search(line):
            errors.append(
                f'{path}:{i}: remove character directly before :ref: directive'
            )
        if CHARACTER_AFTER_RE.search(line):
            errors.append(
                f'{path}:{i}: remove whitespace between :ref: and its opening backtick'
            )
        if SPACE_BEFORE_PUNCT_RE.search(line):
            errors.append(
                f'{path}:{i}: remove space between :ref: closing backtick and punctuation'
            )
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check :ref: directive formatting in RST files"
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
        print("Invalid :ref: directive formatting detected:")
        print("\n".join(all_errors))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
