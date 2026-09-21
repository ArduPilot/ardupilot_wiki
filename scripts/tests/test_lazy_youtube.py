"""Tests for scripts/extensions/lazy_youtube.py.

    python3 scripts/tests/test_lazy_youtube.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "extensions"))
try:
    import lazy_youtube  # noqa: E402
    from sphinxcontrib.youtube import youtube  # noqa: E402
except ImportError as ex:
    print(f"  SKIP  sphinxcontrib-youtube is not installed ({ex}); pip install -r requirements.txt\n")
    sys.exit(0)

failures = 0


def check(name, ok, detail=""):
    global failures
    print(("  PASS  " if ok else "  FAIL  ") + name + (f"   {detail}" if detail else ""))
    if not ok:
        failures += 1


class Translator:
    """The parts of Sphinx's HTML translator the visitor uses."""

    def __init__(self):
        self.body = []

    def starttag(self, node, tag, **attrs):
        return f"<{tag} " + " ".join(f'{k}="{v}"' for k, v in attrs.items()) + ">"


class App:
    def __init__(self):
        self.nodes = {}
        self.extensions = []

    def setup_extension(self, name):
        self.extensions.append(name)

    def add_node(self, node_class, override=False, **visitors):
        self.nodes[node_class] = (override, visitors)


def node(**extra):
    n = youtube.youtube()
    attrs = {"id": "abc123", "aspect": None, "width": None, "height": None,
             "url_parameters": "", "platform_url": None,
             "platform_url_privacy": None, "privacy_mode": None, "align": None}
    attrs.update(extra)
    for key, value in attrs.items():
        n[key] = value
    return n


app = App()
lazy_youtube.setup(app)
check("sphinxcontrib.youtube is set up first, whatever conf.py's order",
      app.extensions == ["sphinxcontrib.youtube"])
check("the youtube node is re-registered with override",
      youtube.youtube in app.nodes and app.nodes[youtube.youtube][0])
visit, depart = app.nodes[youtube.youtube][1]["html"]

for label, extra in (("a plain embed", {}), ("an aligned embed", {"align": "center"}),
                     ("a privacy-mode embed", {"privacy_mode": True})):
    t = Translator()
    visit(t, node(**extra))
    iframes = [b for b in t.body if b.startswith("<iframe")]
    check(f"{label} gets loading=lazy",
          len(iframes) == 1 and 'loading="lazy"' in iframes[0], iframes[0][:80] if iframes else "no iframe")
    check(f"{label} keeps its src", "youtube" in iframes[0] and "abc123" in iframes[0])

t = Translator()
visit(t, node())
check("the tag carries loading exactly once", t.body[-2].count("loading=") == 1)

earlier = Translator()
earlier.body = ["<iframe>an earlier embed, not this one</iframe>"]
visit(earlier, node())
check("only the output this visit appended is touched", "loading=" not in earlier.body[0])

# The visitor's output changing shape must be said, not silently accepted.
warned = []
lazy_youtube.logger.warning = lambda msg, **kw: warned.append(msg)
lazy_youtube._lazy(lambda self, node: self.body.append("<div>no iframe</div>"))(Translator(), node())
check("a visitor that renders no iframe raises a build warning",
      len(warned) == 1 and "not lazy" in warned[0])
check("nothing else in the output changed",
      t.body[0].startswith("<div") and t.body[-1] == "</iframe></div>")

print()
if failures:
    print(f"{failures} CHECK(S) FAILED\n")
    sys.exit(1)
print("all checks passed\n")
