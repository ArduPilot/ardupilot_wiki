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
    print("Processing %s" % f)
    txt = open(f,'r').read()
    matches = re.findall(r'[,.\s][A-Z][A-Z0-9]+_[A-Z_]+[,.\s]', txt)
    matches = re.findall(r'[,.\s][A-Z]+_[A-Z_]+[,.\s]', txt)
    changed = False
    for m in matches:
        s = str(m)
        param = s.strip()
        s2 = "%s:ref:`%s<%s>`%s" % (s[0], param, param, s[-1])
        if args.change:
            txt = txt.replace(s, s2)
            changed = True
            print("Replaced [%s] with [%s]" % (s, s2))
        else:
            print("Found [%s]" % s)
    if changed:
        open(f,'w').write(txt)

