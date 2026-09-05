# Wiki markup that is not well-formed XML

These are pre-existing content bugs, not introduced by the offline work. Browsers
forgive them, so nothing looks wrong on the site. They matter because Sphinx's
epub builder emits XHTML, which is parsed strictly, and a single one of these
makes the generated page unopenable in a reader.

Found by validating every page of a generated epub as XML:

```sh
cd ardupilot && make epub
python3 - <<'PY'
import zipfile, xml.dom.minidom as md
z = zipfile.ZipFile('build/epub/ArduPilot.epub')
for n in (x for x in z.namelist() if x.endswith('.xhtml')):
    try: md.parseString(z.read(n))
    except Exception as e: print(n, e)
PY
```

## 1. `</dev>` typo

`common/source/docs/common-commercial-support.rst`, line 240:

```html
<div class="line">I can assist in developing new autopilots and their successful submission.</dev>
```

`</dev>` should be `</div>`. Edit the file in `common/`; the copies in each
wiki's `source/docs/` are generated from it by `update.py`.

## 2. Unclosed `<p>` tags

`ardupilot/source/index.rst`, in the first `.. raw:: html` block, lines 6, 9
and 12:

```html
<p style="text-align:center;color:green;"><strong>
<p style="text-align:center;color:red;"><strong>
<p style="text-align:left; color:black;">
```

None is closed. The parser reaches the end of the document with a `<p>` still
open and rejects the page. Add the matching `</p>`.

## Not to be confused with

The markup this branch injects had the same class of problem and is fixed:
the theme's `layout.html` (sphinx_rtd_theme) self-closes its void elements and gives boolean
attributes values, because it is injected into the `<head>` of every page of
every wiki, so one unclosed tag there broke every page of a book at once.
