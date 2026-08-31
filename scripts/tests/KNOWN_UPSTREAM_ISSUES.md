# Bugs found in the wiki, not caused by the offline work

Each of these was hit while building offline copies, and each is a pre-existing
fault in the wiki, its build scripts, or its theme. They are recorded here
rather than worked around silently, because a workaround in the offline code
would hide a problem that affects the live site too.

Companion to `KNOWN_MARKUP_ISSUES.md`, which covers content that is not
well-formed XML.

None of these are fixed by the offline branch. Each needs a decision from
whoever owns the code in question.

---

## 1. `build_parameters.py` fetches only `master`, then checks out commits that are not on it

**Where:** `build_parameters.py`, `setup()`

**Symptom:** the run appears to work, then produces empty vehicle directories
and no parameter files at all. The error is buried among hundreds of lines:

```
[ERROR]: Git command failed: git checkout --force de5add012ea2155c8dabc57270c8520a189c7208
[ERROR]: Error: fatal: unable to read tree (de5add012ea2155c8dabc57270c8520a189c7208)
```

`../new_params_mversion/` is then created with a directory per vehicle and
nothing in any of them, which reads like a permissions or path problem rather
than a missing git object.

**Cause:** `setup()` runs

```sh
git fetch origin master
```

but the versions to build come from `firmware.ardupilot.org` and are checked
out **by commit hash**, and those commits live on release branches and tags.
A clone that has only ever fetched `master` does not contain them.

**Reproduce:** on a clone that has not fetched tags,

```sh
git -C ../ardupilot cat-file -t de5add012ea2155c8dabc57270c8520a189c7208
# fatal: git cat-file: could not get object info
```

**Confirmed fix:** fetching every ref makes the object present, with no other
change:

```sh
git -C ../ardupilot fetch --all --tags
git -C ../ardupilot cat-file -t de5add012ea2155c8dabc57270c8520a189c7208
# commit
```

The offline branch adds that fetch to `setup()`.

**Worth noting:** the script also force-checks-out `master` over whatever branch
is present, and `git clean -f -d`, so it will discard uncommitted work in the
firmware repo without warning. Anyone running it by hand should check the repo
is clean first.

---

## 2. The donate button cannot render without a network, and shows nothing when it fails

**Where:** `sphinx_rtd_theme/z_sidebar_additions.html` (ArduPilot's fork of the
theme, a separate repository), present on all 3,958 pages.

```html
<form style="margin:auto;" action="https://ardupilot.org/donate">
<input type="image" src="https://www.paypalobjects.com/en_US/i/btn/btn_donate_LG.gif"
       border="0" name="submit" title="PayPal - The safer, easier way to pay online!"
       alt="Donate" />
</form>
```

**Symptom:** offline, the image cannot load and a broken `<input type="image">`
renders as a small grey box. The `alt="Donate"` is not shown, so the control
reads as nothing at all: not a button, not a link, not an error.

**Why it matters beyond offline:** any reader whose network blocks
`paypalobjects.com`, which ad and tracker blockers commonly do, sees the same
grey box on every page of the wiki.

**Suggested fix, at the source:** an `<a>` styled as a button, or a locally
hosted image. Either keeps the control meaningful when the remote asset does
not arrive.

The offline branch rewrites it inside archived copies only
(`rewrite_donate()` in `scripts/build_offline_artifacts.py`), because the theme
is a repository this one cannot change. `scripts/tests/test_offline_archives.py`
asserts no archived page still references `paypalobjects`, so a theme change
that alters this markup fails a check rather than silently reaching readers.

---

## 3. `images/rpanion.png` is referenced by every wiki and copied into none

**Where:** `common/source/docs/common-commercial-support.rst`, which is copied
into all 11 wikis.

**Symptom:** a broken image on `common-commercial-support` in every wiki,
**including on the live site.** This is not an offline-only fault.

**Reproduce:**

```sh
ls images/rpanion.png                       # exists in source
ls */build/html/_images/rpanion.png         # copied nowhere
```

Measured across a full build: of 13,295 image references in 11 wikis, this is
the only one that resolves to nothing. Every other image is present.

---

## 4. `update.cron` and `update.sh` disagree about where the build lives

**Where:** `scripts/update.cron` and `update.sh`.

`update.cron` does

```sh
cd $HOME/build_wiki
cp $HOME/build_wiki/ardupilot_wiki/update.sh $HOME/cron
```

while `update.sh` itself does

```sh
cd $HOME/ardupilot_wiki
...
pushd ardupilot_wiki
```

so one expects the checkout at `$HOME/build_wiki/ardupilot_wiki` and the other
at `$HOME/ardupilot_wiki/ardupilot_wiki`. Both cannot be right. Anyone standing
up a build server from these scripts hits it immediately, and the failure is a
`cd` that silently lands somewhere unexpected.

---

## 5. The generated reference pages are large enough to hang a browser

**Where:** generated by the build, not authored.

| page | size | DOM elements |
| --- | --- | --- |
| `copter/docs/binary-features.html` | 8.0 MB | not measured |
| `copter/docs/parameters.html` | 5.8 MB | 215,470 |

**Measured on `parameters.html`**, live site against the offline branch, same
page, same browser, back to back:

| | ardupilot.org | offline branch, served from cache |
| --- | --- | --- |
| the page's bytes have all arrived | 1,970 ms | 273 ms |
| the browser has read the page to the end | 4,048 ms | 1,569 ms |
| the page is fully built and displayed | 8,178 ms | 2,241 ms |
| everything has finished loading | 8,289 ms | 2,344 ms |
| **spent building the page after the bytes arrived** | **6,208 ms** | **1,968 ms** |
| elements on the page | 216,620 | 208,674 |

(The figures come from the browser's own navigation timings: `responseEnd`,
`domInteractive`, `domComplete` and `loadEventEnd` respectively, for anyone
wanting to reproduce them.)

Eight seconds on the live site, of which six are spent rendering rather than
fetching. The transfer is not the problem: the document is 6.1 MB but gzips to
454 KB, and arrives in under two seconds.

**Correcting an earlier measurement.** This note previously recorded
`domInteractive` at 33,512 ms and described the renderer freezing outright.
That figure does not reproduce and should not be quoted. Re-measured on the
live site, the browser finishes reading the page at 4,048 ms. The original was
taken under some condition not captured at the time, most likely an attached
debugger, and it overstated the fault by roughly eight times. The page is
genuinely slow; it is not half-a-minute slow.

**Not a caching or hosting problem.** Nothing is being fetched in the
right-hand column above. The `content-visibility: auto` rule for these pages
(in `common/_templates/layout.html` on the offline branch) accounts for most of
the difference in the rendering row, cutting it about threefold. That is a
worthwhile improvement and it is still a two-second page, so the underlying
fault stands.

**Suggested fix:** split the page. Note that `--paramversioning` is *not* that
split, which was worth measuring and has now been measured: a per-version page
carries nearly the whole list anyway.

| page | size | tags |
| --- | --- | --- |
| `parameters.html` (latest dev) | 5.9 MB | 419,450 |
| `parameters-Copter-stable-V4.7.0.html` | 5.1 MB | 392,510 |
| `parameters-Copter-stable-V4.5.0.html` | 3.9 MB | not measured |

Versioning divides the list by firmware version, but every version still
documents nearly every parameter, so it removes about 14% and leaves the page
in the same condition. A useful split has to be by parameter group, which is
how the page is already structured and how its sidebar already navigates it.

**A visible consequence of the parse time.** The version selector these pages
carry is populated by script:

```js
document.addEventListener("DOMContentLoaded", function() {
  fetch("../_static/parameters-Copter.json") ...
```

That first line tells the browser to wait until it has read the entire page
before doing anything, which on this page is 4.0 s on the live site and 1.6 s
on the offline branch. The dropdown therefore sits empty for that whole period,
even though the data it needs is a 1 KB file that was ready immediately. Filling
it from a script placed next to the `<select>` itself, rather than waiting for
the rest of the document, would populate it at once. This is upstream code in
`build_parameters.py`, not the theme.

---

## 6. The wiki links to itself by absolute URL in a few hundred places

**Where:** 212 links across 98 `.rst` files, plus about ten per page injected by
`sphinx_rtd_theme/z_sidebar_additions.html` and `common/_templates/z_top_menu.html`,
giving 38,977 across a full build.

Harmless on `ardupilot.org`, where they are same-origin. On any mirror, staging
site or offline copy they walk the reader back to the live site, and offline
they fail outright.

`common_conf.py` already has `wiki_base_url = 'https://ardupilot.org/'` but uses
it only for intersphinx. Making it the single source for the site's own address
would let a mirror differ by one value.

Note the distinction that has to survive any such change: of the 212 source
links, about 182 point at wiki content and should follow the site, while
roughly 26 are genuinely different services (`/discord` 24, `/donate` 2, plus
the firmware server and the forum) and must stay absolute.

---

## 7. The wiki's PNGs had never been through a lossless optimiser

**Status: fixed on this branch**, by `scripts/optimise_images.py`, running as a
pass over the built output. Kept here because the numbers were measured for
this branch and the earlier figure in this file was wrong.

Images arrive from whatever tool each contributor happened to use, and nothing
in the build re-deflated them. Redoing the deflate stream at maximum effort
recovers real bytes with every pixel unchanged.

**The corrected measurement.** This entry previously claimed 16%, taken from a
sample of the largest files. Measured properly across every distinct built PNG:

| | before | after | saved |
| --- | --- | --- | --- |
| 1,635 distinct PNGs | 354.5 MB | 325.5 MB | **8.2%** |
| as served, 6,294 occurrences | 1,364.1 MB | 1,230.8 MB | **9.8%** (133 MB) |

All 1,635 were verified pixel-identical by decoding both and comparing, and
none failed. **923 of the 1,635, or 56%, were already optimal** and gave back
nothing: the wide spread is the point, and the honest headline is about 10% of
PNG bytes rather than the 16% a sample of the biggest files suggested.

A cold run takes about 2 minutes. Results cache on the content hash in
`.image-cache/`, so later builds only touch new or edited images.

**Why lossless and not something with a better ratio.** JPEG or WebP would save
considerably more, and both were measured: JPEG at q85 gave 69% on these files.
But the saving is worst exactly where the risk is highest.
`AEROFOX-H7_pinout.png` is a 9,449px pinout diagram, deliberately large so pin
labels stay readable when zoomed, and it gives back only 28% to JPEG while
gaining ringing artefacts along every hard edge. Resizing has the same problem:
a cap at 800px would touch 1,239 of 2,331 images, and Sphinx links every
thumbnail to the full-size file, so shrinking the original also removes the
zoomed view a reader gets by clicking.

Anything requiring re-encoding, resizing, or a format change (JPEG, WebP, AVIF)
remains a different proposal with different trade-offs, and was deliberately
kept out of this one.

**Why over the build output rather than the repository.** Committing
recompressed images churns the repository, has to be redone whenever anyone
adds one, asks contributors to remember something, and lands reviewers with a
commit touching thousands of binary files they can only take on trust. Git also
keeps the old blobs forever, so the repository would grow rather than shrink.
The build pass costs nothing ongoing, covers every future image without anyone
acting, and keeps the originals exactly as their authors supplied them.

**It fails safe.** The original bytes are returned when Pillow is unavailable,
when anything raises, when the result is not smaller, or when the result is not
pixel-identical. The worst case is that it does nothing.

---

## 8. The build emits two classes of Sphinx warning, both pre-existing

Recorded because a reviewer running the build will see them and reasonably ask
whether the offline branch introduced them. It did not. Both were reproduced by
building the same tree with and without the offline changes.

**`undefined label`, 8 occurrences, 6 distinct labels.** Present in every build,
with or without `--paramversioning`:

```
undefined label: 'max_pos_xy'   undefined label: 'max_vel_xy'
undefined label: 'max_vel_z'    undefined label: 'max_vel_yaw'
undefined label: 'max_pos_z'
```

These are `:ref:` targets that no page defines. The reference renders as plain
text, so the reader sees an unlinked phrase rather than an error.

**`Duplicate explicit target name`, 240 occurrences.** These appear *only* under
`--paramversioning`:

```
Duplicate explicit target name: "rngfnda_grf_st-sub-stable-v4.7.0"
Duplicate explicit target name: "rngfnda_grf_st-rover-beta-v4.7.0"
```

One parameter's target name collides with itself once a page exists per
firmware version. Since `update.sh` in production passes `--paramversioning`,
the live build emits these too.

**Reproduced:**

| build | `undefined label` | `Duplicate explicit target name` |
| --- | --- | --- |
| without `--paramversioning` | 8 | 0 |
| with `--paramversioning` | 8 | 240 |

Neither is fixed here, and neither is worked around. The duplicate targets are
worth a look by whoever owns `build_parameters.py`, since 240 of them will bury
any genuinely new warning in the same build.

## 9. Sub and Blimp load a user-alerts script for a page they do not have

**Where:** `sub/source/conf.py` and `blimp/source/conf.py` (`html_js_files`),
against `common/source/_static/useralerts.js` and
`common/source/docs/common-user-alerts.rst` (their `copywiki` markers).

**Symptom:** every page of the Sub and Blimp wikis requests
`_static/useralerts.js` and receives 404. Live on ardupilot.org today:

```
ardupilot.org/copter/_static/useralerts.js  ->  200
ardupilot.org/sub/_static/useralerts.js     ->  404
ardupilot.org/blimp/_static/useralerts.js   ->  404
```

**Cause:** six wikis declare `./useralerts.js` in `html_js_files`; four are
listed in that file's `copywiki` marker. Ordinary template drift, all on master:
`988225988` added user alerts, `1437da11a` set the marker to the four vehicles
that then existed, `15bc5face` initialised Blimp from Copter and `dfeeba78b`
initialised Sub from Blimp, each inheriting the `conf.py` line. Nobody widened
the marker.

**The fix is to remove the declaration, NOT to widen the marker.** Widening it
looks like the obvious repair and is a regression. The script writes its table
into `$('.useralerts-list tbody')`, an element that exists only on the
`common-user-alerts` page, and that page's own marker is
`copter,plane,rover,antennatracker` - so Sub and Blimp do not have it. The
script also branches on the URL for copter, plane, rover and tracker only, so
`vehicle` would be `undefined` on those two wikis in any case.

Shipping the file to them therefore renders nothing, and replaces a cheap local
404 with a successful load that then calls
`$.getScript('https://firmware.ardupilot.org/useralerts/manifest.js')` on every
page view, cross-origin, to populate a selector matching nothing. Measured that
way round before it was reverted.

**Consequence as it stands:** one failed request per page on two wikis. Nothing
is broken for readers, because there is no alerts page on those wikis to break.

**Worth deciding separately:** whether Sub and Blimp should have user alerts at
all. Making that work is three changes, not one - the page's marker, the
script's marker, and `sub`/`blimp` branches in the script's vehicle detection -
and it is a product question rather than a build bug.

**Also visible from here:** the script is loaded on every page of all four
wikis that have it, and fetches the external manifest on each one, while doing
anything only on the single alerts page. Roughly 860 requests' worth of that on
Copter alone per full crawl.

**How it was found:** `test_offline_archives.py`, `check_no_dangling_assets()`,
which resolves every local `.js` and `.css` reference in every built page
against the file it names. 9,073 references, and this was the only pair that did
not resolve.

---

## The theme asks for a nav separator that does not exist

`_static/css/ardupilot.css` line 60 sets

```css
background: url(../images/mainnav-sep-2.gif) repeat-y right;
```

and `_static/images/mainnav-sep-2.gif` is not shipped. There is no such file
anywhere in this repository, and the request 404s on every page of every wiki.

Not ours: `ardupilot.css` comes from the theme, which is a separate repository,
so it cannot be fixed from here. Worth knowing about because it is a guaranteed
failed request on every page load, and because with a saved wiki it fails
offline too, where it shows up in the console as a 504 and looks like a fault in
the offline feature rather than a missing decoration.

Found by an acceptance pass on 11 August 2026.
