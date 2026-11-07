#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
Build per-version MAVLink message pages for the vehicle wikis.

The MAVLinkMessages.rst files for each released vehicle version are generated
from the firmware source by Tools/scripts/mavlink_parse.py and stored in the
ArduPilot/ParameterRepository repository. This script fetches them and writes
one orphan page for each of the last four releases next to the vehicle's
MAVLink Messages page, plus a JSON index that the version selector on those
pages reads.

The vehicle's MAVLink Messages page is the "latest" entry. It is rebuilt from
the newest stored file, which ParameterRepository builds from master, so it is
never edited by hand. It keeps the labels other pages link to and says that it
covers unreleased code.

Generated files (git-ignored):
    <site>/source/docs/mavlink-messages-<Vehicle>-<version>.rst
    <site>/source/_static/mavlink-messages-<Vehicle>.json
Rebuilt in place:
    <site>/source/docs/<Prefix>_MAVLink_Messages.rst

Run directly, or through update.py with --mavlinkversioning.
"""

import argparse
import json
import os
import re
import subprocess
import sys
import urllib.request

REPO = "ArduPilot/ParameterRepository"
BRANCH = "main"
TREE_URL = f"https://api.github.com/repos/{REPO}/git/trees/{BRANCH}?recursive=1"
RAW_URL = f"https://raw.githubusercontent.com/{REPO}/{BRANCH}/{{folder}}/MAVLinkMessages.rst"

# repository folder prefix: (wiki site, label prefix used by the existing page)
VEHICLES = {
    "Copter": ("copter", "ArduCopter"),
    "Plane": ("plane", "ArduPlane"),
    "Rover": ("rover", "Rover"),
    "Sub": ("sub", "ArduSub"),
}

LABEL_RE = re.compile(r"^\.\. _(mavlink_\w+):[ \t]*$", re.MULTILINE)
REF_RE = re.compile(r":ref:`(?:([^`<]*?)\s*<)?(mavlink_\w+)>?`")
FOLDER_RE = re.compile(r"^(?P<vehicle>[A-Za-z_]+)-(?P<version>\d+\.\d+)/MAVLinkMessages\.rst$")
TITLE_RE = re.compile(r"^=+\nMAVLink Support\n=+\n", re.MULTILINE)
# the latest page plus this many release cycles, as for the parameter pages
RELEASE_CYCLES = 4
SELECTOR = ".. raw:: html\n   :file: ../_static/mavlink_messages_versioning_script.inc\n"
# the stored top label is mavlink_support, the wiki page it replaces uses <Prefix>_MAVLink_messages
LATEST_LABELS = {"mavlink_support": "MAVLink_messages"}
LATEST_NOTE = ("\n.. note::\n\n   This page covers the development version ({version}, built from master). "
               "It describes unreleased code, so a released firmware may differ. "
               "Use the selector above for a released version.\n")


FIRMWARE_GIT = "https://github.com/ArduPilot/ardupilot.git"
SOURCE_LINK_RE = re.compile(r"(https://github\.com/ArduPilot/ardupilot/tree/)([A-Za-z]+-\d+\.\d+)/")


def fetch(url):
    request = urllib.request.Request(url, headers={"User-Agent": "ardupilot-wiki-build"})
    with urllib.request.urlopen(request, timeout=30) as response:
        return response.read().decode("utf-8")


def firmware_refs():
    """branch and tag names in the firmware repository, from one git ls-remote"""
    output = subprocess.run(["git", "ls-remote", "--heads", "--tags", FIRMWARE_GIT],
                            capture_output=True, text=True, check=True).stdout
    refs = set()
    for line in output.splitlines():
        name = line.split("\t")[-1]
        for prefix in ("refs/heads/", "refs/tags/"):
            if name.startswith(prefix):
                name = name[len(prefix):]
                refs.add(name[:-3] if name.endswith("^{}") else name)  # noqa: FURB188
    return refs


def source_ref(ref, refs):
    """
    the stored files link to <Vehicle>-<major.minor>, which only exists as a
    branch up to 4.5. Later releases use shared ArduPilot-<major.minor>
    branches, and a version with neither is still in development on master.
    """
    if ref in refs:
        return ref
    vehicle, version = ref.split("-", 1)
    shared = f"ArduPilot-{version}"
    if shared in refs:
        return shared
    # Plane 4.0 has no branch, only point release tags such as ArduPlane-4.0.9
    tags = [r for r in refs if re.fullmatch(rf"Ardu{vehicle}-{re.escape(version)}\.\d+", r)]
    if tags:
        return max(tags, key=lambda r: int(r.rsplit(".", 1)[1]))
    return "master"


def available_versions():
    """map each vehicle to its stored versions, newest first"""
    tree = json.loads(fetch(TREE_URL))
    found = {vehicle: [] for vehicle in VEHICLES}
    for entry in tree.get("tree", []):
        match = FOLDER_RE.match(entry["path"])
        if match and match.group("vehicle") in found:
            found[match.group("vehicle")].append(match.group("version"))
    for vehicle in found:
        found[vehicle].sort(key=lambda v: tuple(int(n) for n in v.split(".")), reverse=True)
    return found


def version_has_release(vehicle, version, refs):
    return source_ref(version_tag(vehicle, version), refs) != "master"


def version_tag(vehicle, version):
    return f"{vehicle}-{version}"


def convert(text, vehicle, version, prefix, refs, latest=False):
    """
    turn a stored MAVLinkMessages.rst into a wiki page for one version. The
    latest page keeps the labels of the page it replaces, with no version tag,
    so links from other pages still resolve.
    """
    tag = version_tag(vehicle, version)
    labels = set(LABEL_RE.findall(text))
    text = SOURCE_LINK_RE.sub(lambda m: f"{m.group(1)}{source_ref(m.group(2), refs)}/", text)

    def new_label(name):
        if latest:
            return f"{prefix}_{LATEST_LABELS.get(name, name)}"
        return f"{prefix}_{name}-{tag}"

    text = LABEL_RE.sub(lambda m: f".. _{new_label(m.group(1))}:", text)

    def fix_ref(match):
        caption, target = match.group(1), match.group(2)
        if target not in labels:
            # some versions mention a section the parser left out for that vehicle
            return caption or target.replace("mavlink_", "").replace("_", " ")
        if caption:
            return f":ref:`{caption} <{new_label(target)}>`"
        return f":ref:`{new_label(target)}`"

    text = REF_RE.sub(fix_ref, text)

    # same layout as the versioned parameter pages: the usual title, then the version as a sub-heading
    heading = f"{vehicle} latest" if latest else f"{vehicle} stable {version}"
    title = ("================\nMAVLink Messages\n================\n\n"
             f".. raw:: html\n\n   <h2>MAVLink Messages of {heading}</h2>\n\n{SELECTOR}")
    if latest:
        title += LATEST_NOTE.format(version=version)
    text, count = TITLE_RE.subn(title, text, count=1)
    if count != 1:
        raise ValueError(f"no MAVLink Support title in {tag}")
    # the latest page is in the table of contents, the version pages are only reached through the selector
    return text if latest else ":orphan:\n\n" + text


def build(wiki_dir, only_site=None):
    versions = available_versions()
    refs = firmware_refs()
    for vehicle, (site, prefix) in VEHICLES.items():
        if only_site and only_site != site:
            continue
        docs = os.path.join(wiki_dir, site, "source", "docs")
        static = os.path.join(wiki_dir, site, "source", "_static")
        os.makedirs(static, exist_ok=True)
        index = {f"{vehicle} latest": f"{prefix}_MAVLink_Messages.html"}
        releases = [v for v in versions[vehicle] if version_has_release(vehicle, v, refs)]
        # the newest stored version with no release is built from master: it becomes the latest page
        development = [v for v in versions[vehicle] if v not in releases]
        newest = versions[vehicle][0] if versions[vehicle] else None
        if development and development[0] == newest:
            text = fetch(RAW_URL.format(folder=version_tag(vehicle, development[0])))
            with open(os.path.join(docs, f"{prefix}_MAVLink_Messages.rst"), "w", encoding="utf-8") as f:
                f.write(convert(text, vehicle, development[0], prefix, refs, latest=True))
        if releases:
            # count release cycles by version number, so a vehicle with gaps still stops at the same cycle
            major, minor = (int(n) for n in releases[0].split("."))
            oldest = (major, minor - RELEASE_CYCLES + 1)
            releases = [v for v in releases if tuple(int(n) for n in v.split(".")) >= oldest]
        for version in releases:
            tag = version_tag(vehicle, version)
            text = fetch(RAW_URL.format(folder=tag))
            page = f"mavlink-messages-{tag}"
            with open(os.path.join(docs, f"{page}.rst"), "w", encoding="utf-8") as f:
                f.write(convert(text, vehicle, version, prefix, refs))
            index[f"{vehicle} stable {version}"] = f"{page}.html"
        with open(os.path.join(static, f"mavlink-messages-{vehicle}.json"), "w", encoding="utf-8") as f:
            json.dump(index, f, indent=2)
        latest = newest if development and development[0] == newest else "the committed page"
        print(f"{vehicle}: latest from {latest}, {', '.join(releases)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--site", help="only build this wiki site (copter, plane, rover or sub)")
    parser.add_argument("--wiki-dir", default=os.path.dirname(os.path.abspath(__file__)))
    args = parser.parse_args()
    build(args.wiki_dir, args.site)


if __name__ == "__main__":
    sys.exit(main())
