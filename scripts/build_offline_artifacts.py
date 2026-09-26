"""Build the offline archives after Sphinx has run.

For each wiki, packs build/html into <destdir>/offline/<wiki>-offline.tar.gz
and writes <wiki>-files.json (path -> content hash) for differential updates.
Images used by two or more wikis go once into common-offline.tar.gz. Pages are
rewritten on the way in so they work offline (video embeds become stills, the
donate image becomes a link); rewritten files are also published loose under
offline/files/. offline-manifest.json lists every archive with its size, page
count and build id, and is written last.
"""

import gzip
import hashlib
import io
import json
import os
import re
import shutil
import tarfile
import time
import urllib.error
import urllib.request
from collections import defaultdict
from contextlib import contextmanager
from pathlib import Path

# The payload is mostly PNG and JPEG; higher levels buy a percent or two.
# Spent once per nightly build; every save pays the wire cost repeatedly.
GZIP_LEVEL = 9

# Written into the manifest; unset means this site's own /offline/.
ARTIFACT_BASE_ENV = "ARDUPILOT_OFFLINE_BASE"

# Long-side pixel cap for archive images, 0 = off.
IMAGE_MAX_DIM = int(os.environ.get("ARDUPILOT_OFFLINE_MAX_IMAGE_DIM", "0"))

# Capitalising the directory name gave "Dev" and "Ardupilot".
DISPLAY_NAMES = {
    "copter": "Copter",
    "plane": "Plane",
    "rover": "Rover",
    "sub": "Sub",
    "blimp": "Blimp",
    "dev": "Developer",
    "antennatracker": "Antenna Tracker",
    "planner": "Mission Planner",
    "planner2": "APM Planner 2",
    "ardupilot": "About",
    "mavproxy": "MAVProxy",
}


def _normalise(info: tarfile.TarInfo) -> tarfile.TarInfo:
    """Zero what varies between builds, so unchanged archives are byte-identical."""
    info.mtime = 0
    info.uid = info.gid = 0
    info.uname = info.gname = ""
    info.mode = 0o644 if not info.isdir() else 0o755
    return info


def publish(path: Path, data: bytes) -> None:
    """Write beside the target and rename, so a reader never gets a partial file."""
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)


@contextmanager
def reproducible_tar(path: Path):
    """tar.gz writer whose output depends only on the files' contents, published by rename."""
    tmp = path.with_name(path.name + ".tmp")
    try:
        with open(tmp, "wb") as raw:
            with gzip.GzipFile(fileobj=raw, mode="wb",
                               compresslevel=GZIP_LEVEL, mtime=0) as gz:
                with tarfile.open(fileobj=gz, mode="w") as tar:
                    yield tar
    except BaseException:
        tmp.unlink(missing_ok=True)
        raise
    os.replace(tmp, path)


def log(msg):
    print(f"[build_offline_artifacts]: {msg}", flush=True)


def classify_images(wikis):
    """Return (names shared by 2+ wikis, {wiki: names it carries itself}).

    Shared means the same name AND the same bytes everywhere. Two wikis using
    one name for different pictures is a collision, not sharing: folding either
    copy would show the other wiki the wrong image offline, so every owner
    keeps its own.
    """
    owners = defaultdict(dict)
    for wiki in wikis:
        images = Path(wiki) / "build" / "html" / "_images"
        if not images.is_dir():
            continue
        for path in images.iterdir():
            if path.is_file():
                owners[path.name][wiki] = content_hash(path.read_bytes())

    common = set()
    per_wiki = defaultdict(set)
    for name, copies in owners.items():
        if len(copies) > 1 and len(set(copies.values())) == 1:
            common.add(name)
        else:
            for wiki in copies:
                per_wiki[wiki].add(name)
    return common, per_wiki


# Any iframe is a dead box offline, so each one becomes a card.
WRAPPED_IFRAME_RE = re.compile(
    r'<div class="video_wrapper[^"]*"[^>]*>\s*'
    r'<iframe[^>]*src=["\']([^"\']+)["\'][^>]*>\s*</iframe>\s*</div>',
    re.IGNORECASE)
BARE_IFRAME_RE = re.compile(
    r'<iframe[^>]*src=["\']([^"\']+)["\'][^>]*>(?:\s*</iframe>)?', re.IGNORECASE)

YOUTUBE_SRC_RE = re.compile(
    r'https?://(?:www\.)?(?:youtube(?:-nocookie)?\.com/embed/|youtu\.be/)'
    r'([A-Za-z0-9_-]+)', re.IGNORECASE)
VIMEO_SRC_RE = re.compile(r'https?://player\.vimeo\.com/video/(\d+)', re.IGNORECASE)

THUMB_URL = "https://img.youtube.com/vi/{}/hqdefault.jpg"
VIMEO_OEMBED_URL = ("https://vimeo.com/api/oembed.json"
                    "?url=https%3A%2F%2Fvimeo.com%2F{}")


def classify_embed(src: str):
    """(still key, watch link, verb) for an iframe src; key None when unknown."""
    m = YOUTUBE_SRC_RE.match(src)
    if m:
        watch = f"https://www.youtube.com/watch?v={m.group(1)}"
        # A timestamped embed keeps its place in the video.
        t = re.search(r"[?&](?:start|t)=(\d+)", src)
        if t:
            watch += f"&t={t.group(1)}s"
        return (f"yt-{m.group(1)}", watch, "Watch on YouTube")
    m = VIMEO_SRC_RE.match(src)
    if m:
        return (f"vimeo-{m.group(1)}", f"https://vimeo.com/{m.group(1)}",
                "Watch on Vimeo")
    return (None, src, "Open in a browser")


def collect_embeds(wikis):
    """Still key -> where its thumbnail comes from, for every embedded video."""
    wanted = {}
    for wiki in wikis:
        root = Path(wiki) / "build" / "html"
        if not root.is_dir():
            continue
        for page in root.rglob("*.html"):
            for src in BARE_IFRAME_RE.findall(
                    page.read_text(encoding="utf-8", errors="replace")):
                key = classify_embed(src)[0]
                if key is None:
                    continue
                wanted[key] = (THUMB_URL.format(key[3:]) if key.startswith("yt-")
                               else VIMEO_OEMBED_URL.format(key[6:]))
    return wanted


def looks_like_still(data: bytes) -> bool:
    """JPEG or PNG magic; a consent page cached as a still poisons every build."""
    return data.startswith(b"\xff\xd8\xff") or data.startswith(b"\x89PNG\r\n\x1a\n")


def fetch_thumbnails(specs, cache: Path):
    """A still per video, cached across builds; a missing one is not fatal."""
    cache.mkdir(parents=True, exist_ok=True)
    have, failed = {}, []
    for key in sorted(specs):
        path = cache / f"{key}.jpg"
        cached = path.read_bytes() if path.is_file() else b""
        if not looks_like_still(cached):
            try:
                url = specs[key]
                # Vimeo publishes no fixed still URL; its oEmbed answer names one.
                if "oembed" in url:
                    with urllib.request.urlopen(url, timeout=15) as r:
                        url = json.loads(r.read()).get("thumbnail_url")
                    if not url:
                        raise ValueError("no thumbnail_url")
                with urllib.request.urlopen(url, timeout=15) as r:
                    data = r.read()
                if not looks_like_still(data):
                    raise ValueError("not an image")
                publish(path, data)
            except (urllib.error.URLError, OSError, ValueError):
                failed.append(key)
                continue
        have[key] = path
    if failed:
        # Usually a deleted or private video; this is the only place it shows.
        log(f"  no still for {len(failed)} video(s), so those cards link "
            f"without one. Usually deleted or private:")
        for key in failed:
            log(f"    {key}")
    return have


def embed_card(src: str, wiki: str, thumbs) -> str:
    """A still linking to what the iframe embedded, styled inline for the export's sake."""
    key, link, verb = classify_embed(src)
    link = link.replace("&", "&amp;").replace('"', "&quot;")
    label = (f"&#9654; {verb} "
             '<span style="opacity:.8">(needs a connection)</span>')

    still = ''
    if key in thumbs:
        still = (f'<img src="/{wiki}/_images/{key}.jpg" alt="" '
                 'onerror="this.remove()" '
                 'style="position:absolute;top:0;left:0;width:100%;height:100%;'
                 'object-fit:cover;border-radius:4px">')
    return (
        f'<a class="ap-video" href="{link}" data-ap-external="1" '
        'style="display:block;position:relative;max-width:640px;margin:1em 0;'
        'text-decoration:none;background:#2f2f2f;border-radius:4px">'
        '<span style="display:block;padding-bottom:56.25%"></span>'
        '<span style="position:absolute;top:0;left:0;right:0;bottom:0;'
        'display:flex;align-items:center;justify-content:center;color:#b0b0b0;'
        'font-size:.95em;text-align:center;padding:0 16px">'
        'No preview available</span>'
        f'{still}'
        '<span style="position:absolute;left:0;right:0;bottom:0;'
        'padding:8px 10px;background:rgba(0,0,0,.72);color:#fff;'
        f'font-size:.9em;border-radius:0 0 4px 4px">{label}</span></a>')


def rewrite_embeds(html: str, wiki: str, thumbs) -> str:
    """Swap every iframe in a page for its card; nothing live is archived."""
    html = WRAPPED_IFRAME_RE.sub(lambda m: embed_card(m.group(1), wiki, thumbs), html)
    return BARE_IFRAME_RE.sub(lambda m: embed_card(m.group(1), wiki, thumbs), html)


# The theme's donate control is a remote <input type="image">, a grey box offline.
DONATE_RE = re.compile(
    r'<input[^>]*paypalobjects\.com[^>]*>',
    re.IGNORECASE)

DONATE_LINK = (
    '<a href="https://ardupilot.org/donate" data-ap-external="1" '
    'style="display:inline-block;padding:8px 22px;border-radius:4px;'
    'background:#ffc439;color:#111;font-weight:700;text-decoration:none;'
    'font-family:system-ui,-apple-system,sans-serif;font-size:15px">'
    'Donate</a>'
    '<div style="margin-top:6px;font-size:12px;opacity:.75">needs a connection</div>'
)


# Saved pages must not call home: the analytics beacon goes, and the
# licence badge becomes text since its image lives on a remote host.
ANALYTICS_RE = re.compile(
    r'<script[^>]*\bsrc="[^"]*plausible[^"]*"[^>]*>\s*</script>\s*', re.IGNORECASE)
CC_BADGE_RE = re.compile(r'<img[^>]*i\.creativecommons\.org[^>]*/?>', re.IGNORECASE)


def rewrite_offline_only(html: str) -> str:
    html = ANALYTICS_RE.sub("", html)
    return CC_BADGE_RE.sub('<span class="cc-license">CC BY-SA 3.0</span>', html)


def rewrite_donate(html: str) -> str:
    """Replace the remote donate image with a link that survives offline."""
    return DONATE_RE.sub(DONATE_LINK, html)


SITE_LINK_RE = re.compile(
    r'(href|src)="https?://(?:www\.)?ardupilot\.org(/[^"]*)"', re.IGNORECASE)


def rewrite_site_links(html: str, wikis) -> str:
    """Make absolute ardupilot.org links root-relative; only matters on a mirror."""
    def swap(m):
        attr, path = m.group(1), m.group(2)
        first = path.lstrip('/').split('/')[0]
        if first not in wikis:
            return m.group(0)
        return f'{attr}="{path}"'

    return SITE_LINK_RE.sub(swap, html)


def downsize_image(data: bytes, name: str) -> "bytes | None":
    """Resize an image over IMAGE_MAX_DIM, or None; never raises."""
    if not IMAGE_MAX_DIM:
        return None
    ext = name.rsplit(".", 1)[-1].lower() if "." in name else ""
    if ext not in ("jpg", "jpeg", "png"):
        return None
    try:
        from PIL import Image
        img = Image.open(io.BytesIO(data))
        w, h = img.size
        # An image that already fits would only lose quality and gain a loose copy.
        if max(w, h) <= IMAGE_MAX_DIM:
            return None
        scale = IMAGE_MAX_DIM / max(w, h)
        img = img.resize((round(w * scale), round(h * scale)), Image.LANCZOS)
        out = io.BytesIO()
        if ext in ("jpg", "jpeg"):
            img.convert("RGB").save(out, "JPEG", quality=82, optimize=True)
        else:
            img.save(out, "PNG", optimize=True)
        result = out.getvalue()
        return result if len(result) < len(data) else None
    except Exception:
        return None


def add_image(tar, path, arcname: str, files, loose_dir):
    """Add an image, downsized if enabled; a downsized copy is published loose."""
    data = path.read_bytes()
    smaller = downsize_image(data, arcname)
    if smaller is not None:
        add_bytes(tar, arcname, smaller, files, loose_dir=loose_dir)
        return
    tar.add(path, arcname=arcname, filter=_normalise)
    if files is not None:
        files[arcname] = content_hash(data)


def add_bytes(tar, arcname: str, data: bytes, files=None, loose_dir=None):
    """Add generated bytes; with loose_dir, publish them there for updates too."""
    info = tarfile.TarInfo(arcname)
    info.size = len(data)
    tar.addfile(_normalise(info), io.BytesIO(data))
    if files is not None:
        files[arcname] = content_hash(data)
    if loose_dir is not None:
        # Stored as <arcname>.gz for gzip_static.
        dest = Path(loose_dir) / (arcname + ".gz")
        dest.parent.mkdir(parents=True, exist_ok=True)
        publish(dest, gzip.compress(data, compresslevel=9, mtime=0))


def content_hash(data: bytes) -> str:
    """Freshness check, not a security boundary; must match hashBytes in the client."""
    return hashlib.sha256(data).hexdigest()[:16]


def raw_size(path: Path) -> int:
    """Uncompressed size from the gzip trailer, exact below 4 GiB."""
    with open(path, "rb") as f:
        f.seek(-4, os.SEEK_END)
        return int.from_bytes(f.read(4), "little")


def dir_size(path: Path) -> int:
    return sum(f.stat().st_size for f in path.rglob("*") if f.is_file())


def build_id() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


# Wikis packed into the common archive instead of one of their own.
FOLD_INTO_COMMON = {"ardupilot"}


def write_common_archive(wikis, common_names, out_dir: Path, thumbs,
                         files=None, folded=(), per_wiki=None) -> int:
    """The shared images plus any folded wiki."""
    archive = out_dir / "common-offline.tar.gz"
    seen = set()
    with reproducible_tar(archive) as tar:
        for wiki in wikis:
            images = Path(wiki) / "build" / "html" / "_images"
            if not images.is_dir():
                continue
            for name in sorted(common_names - seen):
                path = images / name
                if path.is_file():
                    add_image(tar, path, f"_images/{name}", files, out_dir / "files")
                    seen.add(name)
        # Stills live with the shared images, where the worker and exporter look.
        for key, path in sorted(thumbs.items()):
            add_bytes(tar, f"_images/{key}.jpg", path.read_bytes(), files,
                      loose_dir=out_dir / "files")
        # Written exactly as write_wiki_archive would, unique images included.
        for wiki in folded:
            add_wiki_tree(tar, wiki, (per_wiki or {}).get(wiki, set()),
                          out_dir, thumbs, set(wikis), files)
    return archive.stat().st_size


# Historical parameter pages are 4 to 6 MB each; they are offered individually.
PARAM_VERSION_RE = re.compile(
    r"^parameters-(?P<vehicle>[A-Za-z]+)-(?P<channel>stable|beta|latest)-"
    r"V?(?P<version>[0-9][0-9A-Za-z.\-]*)\.html$"
)


def param_version_of(rel: Path):
    """A description of a historical parameter page, or None if it is not one."""
    if len(rel.parts) != 2 or rel.parts[0] != "docs":
        return None
    m = PARAM_VERSION_RE.match(rel.name)
    if not m:
        return None
    return {
        "file": rel.as_posix(),
        "channel": m.group("channel"),
        "version": m.group("version"),
        "label": f"{m.group('version')}" +
                 ("" if m.group("channel") == "stable" else f" {m.group('channel')}"),
    }


def param_versions_for(html_root: Path) -> list:
    """Every historical parameter page this wiki built, newest first."""
    out = []
    docs = html_root / "docs"
    if not docs.is_dir():
        return out
    for path in sorted(docs.glob("parameters-*.html")):
        info = param_version_of(path.relative_to(html_root))
        if not info:
            continue
        info["bytes"] = path.stat().st_size
        out.append(info)

    def key(e):
        nums = [int(n) for n in re.findall(r"\d+", e["version"])] or [0]
        return (nums, e["channel"] != "stable")

    out.sort(key=key, reverse=True)

    # Default to the newest stable: not a beta, and not master (parameters.html).
    for e in out:
        if e["channel"] == "stable":
            e["default"] = True
            break
    return out


def add_wiki_tree(tar, wiki: str, exclusive: set, out_dir: Path, thumbs,
                  wikis, files=None) -> None:
    """One wiki's pages, assets and unique images, into an open tar."""
    html_root = Path(wiki) / "build" / "html"
    for path in sorted(html_root.rglob("*")):
        if not path.is_file():
            continue
        rel = path.relative_to(html_root)
        parts = rel.parts
        # Shared images travel in the common archive instead.
        if parts and parts[0] == "_images" and rel.name not in exclusive:
            continue
        # Never fold the offline artefacts back into themselves.
        if parts and parts[0] == "offline":
            continue
        # Offered separately, see param_versions_for.
        if param_version_of(rel):
            continue
        arcname = f"{wiki}/{rel.as_posix()}"
        if path.suffix == ".html":
            html = path.read_text(encoding="utf-8", errors="replace")
            rewritten = rewrite_site_links(
                rewrite_donate(rewrite_offline_only(
                    rewrite_embeds(html, wiki, thumbs))), wikis)
            if rewritten != html:
                add_bytes(tar, arcname, rewritten.encode("utf-8"), files,
                          loose_dir=out_dir / "files")
                continue
        # Unique images may be downsized; css, js and fonts go as-is.
        if parts and parts[0] == "_images":
            add_image(tar, path, arcname, files, out_dir / "files")
            continue
        tar.add(path, arcname=arcname, filter=_normalise)
        if files is not None:
            files[arcname] = content_hash(path.read_bytes())


def write_wiki_archive(wiki: str, exclusive: set, out_dir: Path, thumbs,
                       wikis, files=None) -> int:
    """Pages, static assets and the images this wiki carries itself."""
    archive = out_dir / f"{wiki}-offline.tar.gz"
    with reproducible_tar(archive) as tar:
        add_wiki_tree(tar, wiki, exclusive, out_dir, thumbs, wikis, files)
    return archive.stat().st_size


def write_file_table(out_dir: Path, name: str, files: dict) -> Path:
    """Archive path -> content hash, so an update fetches only what changed."""
    path = out_dir / f"{name}-files.json"
    publish(path, json.dumps(files, separators=(",", ":"), sort_keys=True).encode("utf-8"))
    return path


def refresh_static(wikis) -> int:
    """Copy source/_static over build/html/_static; --fast leaves it stale."""
    copied = 0
    for wiki in wikis:
        src = Path(wiki) / "source" / "_static"
        dst = Path(wiki) / "build" / "html" / "_static"
        if not src.is_dir() or not dst.is_dir():
            continue
        for path in sorted(src.rglob("*")):
            if not path.is_file():
                continue
            target = dst / path.relative_to(src)
            if target.is_file() and target.read_bytes() == path.read_bytes():
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
            copied += 1
    if copied:
        log(f"refreshed {copied} static file(s) the build had left stale")
    return copied


def promote(staging: Path, live: Path) -> None:
    """The finished set swaps in by rename, never file by file."""
    old = live.with_name(live.name + ".old")
    if old.exists():
        shutil.rmtree(old)
    if live.exists():
        os.rename(live, old)
    os.rename(staging, live)
    shutil.rmtree(old, ignore_errors=True)


def build(wikis, destdir: Path) -> Path:
    live_dir = Path(destdir) / "offline"

    built = [w for w in wikis if (Path(w) / "build" / "html" / "index.html").is_file()]
    if not built:
        log("no built wikis found; nothing to do")
        return live_dir

    # Everything is written to a staging generation; a failure anywhere leaves
    # the live set untouched and consistent, and readers never see a mix.
    out_dir = live_dir.with_name(live_dir.name + ".new")
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # An archive is only as current as the tree it is read from.
    refresh_static(built)

    log(f"classifying images across {len(built)} wikis")
    common_names, per_wiki = classify_images(built)

    embeds = collect_embeds(built)
    log(f"fetching stills for {len(embeds)} embedded videos")
    # Cached under destdir, which on the build server is outside the git
    # checkout (update.sh runs git clean -x there) and outside the promoted
    # offline/ generation, so it is neither wiped nightly nor published.
    thumbs = fetch_thumbnails(embeds, Path(destdir) / "offline.cache" / "thumbs")

    folded = [w for w in built if w in FOLD_INTO_COMMON]
    log(f"writing common archive ({len(common_names)} shared images, "
        f"{len(thumbs)} video stills" +
        (f", and {', '.join(folded)} folded in" if folded else "") + ")")
    common_files = {}
    common_bytes = write_common_archive(built, common_names, out_dir, thumbs,
                                        common_files, folded=folded,
                                        per_wiki=per_wiki)
    write_file_table(out_dir, "common", common_files)
    # Folded pages are served from the common cache, so they count as common's.
    common_pages = sum(
        sum(1 for _ in (Path(w) / "build" / "html").rglob("*.html"))
        for w in folded)
    # A now-folded wiki's old archive and file table must not stay published.
    for wiki in folded:
        for stale in (out_dir / f"{wiki}-offline.tar.gz",
                      out_dir / f"{wiki}-files.json"):
            if stale.exists():
                stale.unlink()
                log(f"  removed {stale.name}, now folded into common")

    entries = []
    for wiki in built:
        if wiki in FOLD_INTO_COMMON:
            continue
        html_root = Path(wiki) / "build" / "html"
        wiki_files = {}
        size = write_wiki_archive(wiki, per_wiki.get(wiki, set()), out_dir,
                                  thumbs, set(built), wiki_files)
        write_file_table(out_dir, wiki, wiki_files)
        pages = sum(1 for _ in html_root.rglob("*.html"))
        raw = raw_size(out_dir / f"{wiki}-offline.tar.gz")
        entries.append({
            "id": wiki,
            "name": DISPLAY_NAMES.get(wiki, wiki.capitalize()),
            "mb": round(size / 1048576),
            "pages": pages,
            # Compressed and decompressed sizes; the panel shows both.
            "bytes": size,
            "raw_bytes": raw,
            # No .gz: gzip_static serves <name>.tar.gz for <name>.tar.
            "archive": f"{wiki}-offline.tar",
            "files": f"{wiki}-files.json",
        })
        versions = param_versions_for(html_root)
        if versions:
            entries[-1]["param_versions"] = versions
        log(f"  {wiki}: {size / 1048576:.0f} MB, {pages} pages" +
            (f", {len(versions)} parameter versions offered separately"
             if versions else ""))

    entries.sort(key=lambda e: -e["mb"])

    artifact_base = os.environ.get(ARTIFACT_BASE_ENV, "")
    if artifact_base:
        log(f"archives will be served from {artifact_base}")
    else:
        log("archives will be served from this site's own /offline/")

    manifest = {
        "generated": build_id(),
        "artifact_base": artifact_base,
        "common": {
            "id": "common",
            "name": "Shared images (required)",
            "images": len(common_names) + len(thumbs),
            "mb": round(common_bytes / 1048576),
            "pages": common_pages,
            "required": True,
            "bytes": common_bytes,
            "raw_bytes": raw_size(out_dir / "common-offline.tar.gz"),
            "archive": "common-offline.tar",
            "files": "common-files.json",
        },
        "wikis": entries,
    }

    manifest_path = out_dir / "offline-manifest.json"
    publish(manifest_path, json.dumps(manifest, indent=2).encode("utf-8"))
    promote(out_dir, live_dir)
    log(f"wrote {live_dir / 'offline-manifest.json'}")
    return live_dir


if __name__ == "__main__":
    import sys
    wiki_list = sys.argv[1:] or [
        "copter", "plane", "rover", "sub", "blimp", "dev",
        "antennatracker", "planner", "planner2", "ardupilot", "mavproxy",
    ]
    build(wiki_list, Path("."))
