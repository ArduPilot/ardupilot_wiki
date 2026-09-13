"""Check what a reader receives, by reading the built archives themselves.

    python3 scripts/tests/test_offline_archives.py [wiki ...]

Run after a full update.py. The rewrites are regular expressions over the
theme's HTML, so a theme change would silently stop them matching.
"""

import json
import re
import sys
import tarfile
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
OFFLINE = REPO / "offline"

WIKIS = ["copter", "plane", "rover", "sub", "blimp", "dev",
         "antennatracker", "planner", "planner2", "ardupilot", "mavproxy"]

failures = 0


def check(name, ok, detail=""):
    global failures
    print(("  PASS  " if ok else "  FAIL  ") + name + ("   " + detail if detail else ""))
    if not ok:
        failures += 1


def html_members(archive: Path, limit=None):
    """Yield (name, text) for HTML members, without unpacking to disk."""
    with tarfile.open(archive) as tar:
        seen = 0
        for member in tar:
            if not member.isfile() or not member.name.endswith(".html"):
                continue
            f = tar.extractfile(member)
            if f is None:
                continue
            yield member.name, f.read().decode("utf-8", "replace")
            seen += 1
            if limit and seen >= limit:
                return


def check_image_classification():
    """Same name and bytes is shared; same name, different bytes is a collision."""
    sys.path.insert(0, str(REPO / "scripts"))
    from build_offline_artifacts import classify_images

    with tempfile.TemporaryDirectory() as tmp:
        trees = {"alpha": {"logo.png": b"same", "clash.png": b"A", "own.png": b"mine"},
                 "beta": {"logo.png": b"same", "clash.png": b"B"}}
        for wiki, images in trees.items():
            root = Path(tmp) / wiki / "build" / "html" / "_images"
            root.mkdir(parents=True)
            for name, body in images.items():
                (root / name).write_bytes(body)
        alpha, beta = (str(Path(tmp) / w) for w in ("alpha", "beta"))
        common, per_wiki = classify_images([alpha, beta])

    check("identical bytes under one name are shared", common == {"logo.png"},
          str(sorted(common)))
    check("a name collision is nobody's to share",
          "clash.png" in per_wiki[alpha] and "clash.png" in per_wiki[beta],
          str({w.rsplit("/", 1)[-1]: sorted(n) for w, n in per_wiki.items()}))
    check("a unique image stays with its wiki",
          "own.png" in per_wiki[alpha] and "own.png" not in per_wiki[beta])


def check_still_validation():
    """Only real image bytes may enter the thumbnail cache."""
    sys.path.insert(0, str(REPO / "scripts"))
    from build_offline_artifacts import looks_like_still
    check("a JPEG passes", looks_like_still(b"\xff\xd8\xff\xe0rest"))
    check("a PNG passes", looks_like_still(b"\x89PNG\r\n\x1a\nrest"))
    check("an HTML consent page does not", not looks_like_still(b"<!DOCTYPE html><html>"))
    check("an empty answer does not", not looks_like_still(b""))


def check_embed_rewrite():
    """No iframe survives into a page bound for an archive."""
    sys.path.insert(0, str(REPO / "scripts"))
    from build_offline_artifacts import rewrite_embeds

    wrapped = ('<div class="video_wrapper align-center">'
               '<iframe src="https://www.youtube-nocookie.com/embed/dQw4w9WgXcQ?rel=0">'
               '</iframe></div>')
    vimeo = ('<div class="video_wrapper">'
             '<iframe src="https://player.vimeo.com/video/123456"></iframe></div>')
    form = '<iframe src="https://docs.google.com/forms/d/e/abc/viewform"></iframe>'
    page = "<html><body>" + wrapped + vimeo + form + "</body></html>"
    out = rewrite_embeds(page, "rover", {"yt-dQw4w9WgXcQ": object()})

    check("no iframe survives the rewrite", "<iframe" not in out.lower(), out[:120])
    check("an aligned privacy-mode embed becomes a YouTube card with its still",
          'href="https://www.youtube.com/watch?v=dQw4w9WgXcQ"' in out and
          "yt-dQw4w9WgXcQ.jpg" in out)
    check("a vimeo embed becomes a Vimeo card",
          'href="https://vimeo.com/123456"' in out and "Watch on Vimeo" in out)
    check("an unknown iframe becomes a link card to what it embedded",
          'href="https://docs.google.com/forms/d/e/abc/viewform"' in out and
          "Open in a browser" in out)


def check_shared_images_agree():
    """An image the common archive serves must be what every wiki published."""
    table_path = OFFLINE / "common-files.json"
    if not table_path.is_file():
        check("common file table exists", False, str(table_path))
        return
    table = json.loads(table_path.read_text())
    shared = {name.split("/", 1)[1]: h for name, h in table.items()
              if name.startswith("_images/") and not name.startswith("_images/yt-")}

    import hashlib
    wrong = []
    holders = 0
    for wiki in WIKIS:
        images = REPO / wiki / "build" / "html" / "_images"
        if not images.is_dir():
            continue
        for name, expected in shared.items():
            path = images / name
            if not path.is_file():
                continue
            holders += 1
            if hashlib.sha256(path.read_bytes()).hexdigest()[:16] != expected:
                wrong.append(f"{wiki}/{name}")
    check("every wiki holding a shared image holds the bytes common serves",
          holders and not wrong,
          f"{len(wrong)} disagree, e.g. {wrong[0]}" if wrong
          else f"{len(shared)} shared images checked across {holders} holdings")


def check_offline_only_rewrites():
    """Archived pages neither phone home nor lose their place in a video."""
    sys.path.insert(0, str(REPO / "scripts"))
    from build_offline_artifacts import classify_embed, rewrite_offline_only

    _, watch, _ = classify_embed("https://www.youtube.com/embed/abc123?start=90")
    check("a timestamped embed keeps its place in the watch link",
          watch == "https://www.youtube.com/watch?v=abc123&t=90s", watch)
    _, plain, _ = classify_embed("https://www.youtube.com/embed/abc123")
    check("an untimestamped embed gets a plain watch link",
          plain == "https://www.youtube.com/watch?v=abc123", plain)

    page = ('<script defer data-domain="x" '
            'src="https://plausible.ardupilot.org/js/script.js"></script>'
            '<a href="https://creativecommons.org/licenses/by-sa/3.0/">'
            '<img alt="CC" src="https://i.creativecommons.org/l/by-sa/3.0/88x31.png"/>'
            '</a><p>body</p>')
    out = rewrite_offline_only(page)
    check("the analytics beacon is stripped from archived pages",
          "plausible" not in out, out[:80])
    check("the licence badge becomes local text, attribution kept",
          "i.creativecommons.org" not in out and "CC BY-SA 3.0" in out and
          "creativecommons.org/licenses" in out, out[:120])


def check_folded_lists_agree():
    """The four literal FOLDED_INTO_COMMON copies must name the same wikis."""
    root = Path(__file__).resolve().parents[2]
    sources = {
        "scripts/build_offline_artifacts.py":
            r"FOLD_INTO_COMMON\s*=\s*\{([^}]*)\}",
        "frontend/sw.js":
            r"FOLDED_INTO_COMMON\s*=\s*new Set\(\[([^\]]*)\]\)",
        "common/source/_static/common_offline_page.js":
            r"FOLDED_INTO_COMMON\s*=\s*\[([^\]]*)\]",
        "common/source/_static/common_offline_unpack.js":
            r"FOLDED_INTO_COMMON\s*=\s*\[([^\]]*)\]",
    }
    lists = {}
    for rel, pattern in sources.items():
        m = re.search(pattern, (root / rel).read_text())
        lists[rel] = sorted(re.findall(r"['\"]([^'\"]+)['\"]", m.group(1))) if m else None
    values = set(map(str, lists.values()))
    check("every FOLDED_INTO_COMMON copy names the same wikis",
          None not in lists.values() and len(values) == 1,
          "; ".join(f"{k}: {v}" for k, v in lists.items()))


def check_assets_follow_pages():
    """Wherever the offline page went, its assets must have gone too."""
    ASSETS = ["common_offline.css", "common_offline_page.js",
              "common_offline_export.js", "common_offline_document_builder.js",
              "common_offline_unpack.js",
              "common_offline_update.js"]
    have_page, missing = [], []

    for wiki in WIKIS:
        page = REPO / wiki / "build" / "html" / "docs" / "common-offline.html"
        if not page.is_file():
            continue
        have_page.append(wiki)
        for asset in ASSETS:
            if not (REPO / wiki / "build" / "html" / "_static" / asset).is_file():
                missing.append(f"{wiki}/{asset}")

    check("every wiki with the offline page has its assets",
          have_page and not missing,
          f"{len(missing)} missing, e.g. {missing[0]}" if missing
          else f"{len(have_page)} wikis, {len(ASSETS)} assets each")


def check_archives_carry_current_static():
    """Archives hold the current panel scripts: archive vs built tree, built
    tree vs source."""
    import json
    sys.path.insert(0, str(REPO / "scripts"))
    from build_offline_artifacts import content_hash

    source_dir = REPO / "common" / "source" / "_static"
    shared = (sorted(source_dir.glob("common_offline*.js")) +
              sorted(source_dir.glob("common_offline*.css")))
    if not shared:
        check("archives carry the current panel scripts", False,
              "no common_offline* assets in common/source/_static")
        return

    # 1. archive vs built tree
    stale_archive, checked = [], 0
    for wiki in WIKIS:
        table = OFFLINE / f"{wiki}-files.json"
        built_dir = REPO / wiki / "build" / "html" / "_static"
        if not table.is_file():
            continue          # folded into common, or not built
        entries = json.loads(table.read_text())
        for f in shared:
            key = f"{wiki}/_static/{f.name}"
            built = built_dir / f.name
            if key not in entries or not built.is_file():
                continue
            checked += 1
            if entries[key] != content_hash(built.read_bytes()):
                stale_archive.append(key)

    check("archives hold the same bytes as the built tree",
          not stale_archive,
          f"{len(stale_archive)} stale, e.g. {stale_archive[0]}"
          if stale_archive else f"{checked} asset copies match")

    # 2. built tree vs source, with the copywiki marker normalised out
    marker = re.compile(rb"\[copywiki.*?\]", re.MULTILINE)
    stale_build = []
    for wiki in WIKIS:
        built_dir = REPO / wiki / "build" / "html" / "_static"
        if not built_dir.is_dir():
            continue
        for f in shared:
            built = built_dir / f.name
            if not built.is_file():
                continue
            if marker.sub(b"", f.read_bytes()).strip() != \
               marker.sub(b"", built.read_bytes()).strip():
                stale_build.append(f"{wiki}/_static/{f.name}")

    check("the built tree holds the current source, so a build was not skipped",
          not stale_build,
          f"{len(stale_build)} stale, e.g. {stale_build[0]}"
          if stale_build else f"{len(shared)} assets across the built wikis")


def check_no_dangling_assets():
    """No built page references a local script or stylesheet that is not there."""
    ref = re.compile(rb'(?:src|href)="([^"]+\.(?:js|css))"')
    # No browser loads the theme's IE conditional comments.
    ie_only = re.compile(rb"<!--\[if[^>]*>.*?<!\[endif\]-->", re.DOTALL)
    missing, checked = {}, 0
    for wiki in WIKIS:
        root = REPO / wiki / "build" / "html"
        if not root.is_dir():
            continue
        seen = set()
        for page in root.rglob("*.html"):
            for m in ref.finditer(ie_only.sub(b"", page.read_bytes())):
                url = m.group(1).decode("utf-8", "replace")
                # Local references only.
                if url.startswith(("http://", "https://", "//", "data:")):
                    continue
                seen.add((url.split("?")[0], page))
        for url, page in seen:
            # Root-relative URLs resolve against frontend/, the deployed webroot.
            base = (REPO / "frontend") if url.startswith("/") else page.parent
            target = base / url.lstrip("/")
            checked += 1
            if not target.exists():
                missing.setdefault(url, page.relative_to(REPO).as_posix())

    # Known upstream faults (KNOWN_UPSTREAM_ISSUES.md), allowed but reported.
    known = {u for u in missing if u.endswith("useralerts.js")}   # issue 9
    ours = {u: p for u, p in missing.items() if u not in known}

    check("no built page references a script or stylesheet that is missing",
          not ours,
          "; ".join(f"{u} (e.g. {p})" for u, p in list(ours.items())[:3]) or
          f"{checked} references resolve" +
          (f", plus {len(known)} known upstream "
           f"(KNOWN_UPSTREAM_ISSUES.md)" if known else ""))


def main():
    wikis = [w for w in (sys.argv[1:] or WIKIS)]
    print("\noffline archives: what the reader receives\n")

    check("the build promoted cleanly, leaving no staging generation behind",
          not (OFFLINE.with_name("offline.new")).exists() and
          not (OFFLINE.with_name("offline.old")).exists())
    check_image_classification()
    check_still_validation()
    check_embed_rewrite()
    check_shared_images_agree()
    check_folded_lists_agree()
    check_offline_only_rewrites()

    checked = 0
    for wiki in wikis:
        archive = OFFLINE / f"{wiki}-offline.tar.gz"
        if not archive.is_file():
            continue

        remote_donate = []
        phones_home = []
        live_iframes = []
        local_donate = 0
        pages = 0
        # A sample: the control is in every sidebar.
        for name, html in html_members(archive, limit=200):
            pages += 1
            if "paypalobjects" in html:
                remote_donate.append(name)
            if ("plausible.ardupilot.org" in html or
                    "i.creativecommons.org" in html):
                phones_home.append(name)
            if "<iframe" in html.lower():
                live_iframes.append(name)
            if 'href="https://ardupilot.org/donate"' in html and ">Donate</a>" in html:
                local_donate += 1

        if not pages:
            continue
        checked += 1

        check(f"{wiki}: no page ships a live iframe",
              not live_iframes,
              f"{len(live_iframes)} pages, e.g. {live_iframes[0]}" if live_iframes
              else f"{pages} pages sampled")
        check(f"{wiki}: no page phones home for analytics or the badge",
              not phones_home,
              f"{len(phones_home)} of {pages}, e.g. {phones_home[0]}" if phones_home
              else f"{pages} pages sampled")
        check(f"{wiki}: no page ships a remote donate image",
              not remote_donate,
              f"{len(remote_donate)} of {pages} still reference paypalobjects"
              if remote_donate else f"{pages} pages sampled")

        check(f"{wiki}: the donate control survives as a link",
              local_donate > 0,
              f"{local_donate} of {pages} pages")

    check_assets_follow_pages()
    check_archives_carry_current_static()
    check_no_dangling_assets()

    check("archives were present to test", checked > 0,
          f"{checked} wikis" if checked else "run a full update.py first")

    print("\n" + (f"{failures} CHECK(S) FAILED\n" if failures else "all checks passed\n"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
