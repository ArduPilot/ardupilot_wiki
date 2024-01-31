#!/usr/bin/env python3
'''
Tool to rename parameters in rst files

examples:
    do all files: scripts/rename_params.py data/plane4.5-parmchange.txt . --recurse
    do a specific file: scripts/rename_params.py data/plane4.5-parmchange.txt plane/source/docs/tuning-quickstart.rst
    do one directory: scripts/rename_params.py data/plane4.5-parmchange.txt plane/source/docs
'''

import os
import glob

from argparse import ArgumentParser

parser = ArgumentParser(description="parameter conversion tool")

parser.add_argument("--recurse", default=False, action='store_true', help="recurse into subdirectories")
parser.add_argument("--nonref", default=False, action='store_true', help="include non-ref instances")
parser.add_argument("--verbose", default=False, action='store_true', help="verbose messages")
parser.add_argument("param_map", default=None, help="parameter map file")
parser.add_argument("files", nargs="+", default=None, help="directories or files")

args = parser.parse_args()


def load_param_map(fname):
    lines = open(fname, 'r').readlines()
    ret = {}
    for line in lines:
        if line.startswith("#"):
            # allow comments
            continue
        a = line.split()
        if len(a) != 2:
            print("Bad line %s" % line)
            continue
        ret[a[1]] = a[0]
    return ret


def process_file(fname, param_map):
    dname = os.path.dirname(fname)
    bname = os.path.basename(fname)
    if bname.startswith("common-") and dname.find("common") == -1:
        if args.verbose:
            print("Skipping common file %s" % fname)
        return
    needs_write = False
    txt = open(fname, "r").read()

    replacements = [":ref:`PARAMNAME <PARAMNAME>`",
                    ":ref:`PARAMNAME<PARAMNAME>`"]
    if args.nonref:
        replacements.extend(["PARAMNAME"])

    for old_name in param_map.keys():
        new_name = param_map[old_name]
        for r in replacements:
            p1 = r.replace("PARAMNAME", old_name)
            p2 = r.replace("PARAMNAME", new_name)
            if txt.find(p1) != -1:
                txt = txt.replace(p1, p2)
                needs_write = True
    if not needs_write:
        return
    print("Updating %s" % fname)
    open(fname, "w").write(txt)


param_map = load_param_map(args.param_map)
print("Loaded param map for %u parameters" % len(param_map.keys()))

for fname in args.files:
    if os.path.isfile(fname):
        process_file(fname, param_map)
    elif os.path.isdir(fname):
        if args.recurse:
            for root, dirs, files in os.walk(fname):
                for file in files:
                    if file.endswith(".rst"):
                        process_file(os.path.join(root, file), param_map)
        else:
            g = glob.glob(os.path.join(fname, "*.rst"))
            for f in g:
                process_file(f, param_map)
