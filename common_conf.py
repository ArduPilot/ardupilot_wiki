# This contains common configuration information for the ardupilot wikis.
# This information is imported by the conf.py files in each of the sub wikis


# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'sphinx.ext.mathjax',     # For :math: element rendering
    'sphinx.ext.ifconfig',
    'sphinxcontrib.youtube',  # For youtube embedding
    'sphinxcontrib.jquery',
    'sphinx_tabs.tabs'        # For clickable tabs
]

# Set False to re-enable warnings for non-local images.
disable_non_local_image_warnings = True


# wiki_base_url='https://dl.dropboxusercontent.com/u/3067678/share2/wiki'
# intersphinx_base_url=wiki_base_url+'/%s/build/html/'

wiki_base_url = 'https://ardupilot.org/'
intersphinx_base_url = wiki_base_url + '%s/'


# Where to point the base of the build for the main site menu
html_context = {'target': '/'}
# This needs to change to the actual URL root once the theme updated.

# Known wiki keys (single source of truth)
WIKI_KEYS = [
    'antennatracker',
    'ardupilot',
    'blimp',
    'copter',
    'dev',
    'mavproxy',
    'plane',
    'planner',
    'planner2',
    'rover',
    'sub',
]

# Build mapping programmatically (remote auto-discovery by using None for objects.inv)
intersphinx_mapping = {k: (intersphinx_base_url % k, None) for k in WIKI_KEYS}


# PATCH REMOVE NON-LOCAL IMAGE WARNINGS
# From:
#  http://stackoverflow.com/questions/12772927/specifying-an-online-image-in-sphinx-restructuredtext-format
#  And https://github.com/sphinx-doc/sphinx/issues/2429


if disable_non_local_image_warnings:
    import sphinx.environment
    from docutils.utils import get_source_line

    def _warn_node(self, msg, node, **kwargs):
        if not msg.startswith('nonlocal image URI found:'):
            self._warnfunc(msg, '%s:%s' % get_source_line(node), **kwargs)

    sphinx.environment.BuildEnvironment.warn_node = _warn_node
# ENDPATH


def setup(app):
    app.add_css_file("common_theme_override.css")
