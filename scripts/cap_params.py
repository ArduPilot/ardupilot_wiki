#!/usr/bin/env python
'''
script to find and optionally replace param refs in rst files
'''

import re
from argparse import ArgumentParser

parser = ArgumentParser('find and optionally replace parameters')
parser.add_argument("--change", action='store_true', help="change matches to use param markup")
parser.add_argument("files", nargs='+')

args = parser.parse_args()

for f in args.files:
    print(f"Processing {f}")
    with open(f, 'r') as in_file:
        txt = in_file.read()
    matches = re.findall(r'[,.\s][A-Z][A-Z0-9]+_[A-Z_]+[,.\s]', txt)
    matches = re.findall(r'[,.\s][A-Z]+_[A-Z_]+[,.\s]', txt)
    changed = False
    for m in matches:
        s = str(m)
        param = s.strip()
        s2 = f"{s[0]}:ref:`{param}<{param}>`{s[-1]}"
        if args.change:
            txt = txt.replace(s, s2)
            changed = True
            print(f"Replaced [{s}] with [{s2}]")
        else:
            print(f"Found [{s}]")
    if changed:
        with open(f, 'w') as out_file:
            out_file.write(txt)
