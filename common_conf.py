# -*- coding: utf-8 -*-
#
# This contains common configuration information for the ardupilot wikis.
# This information is imported by the conf.py files in each of the sub wikis

import warnings
from packaging import version
import pkg_resources

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'sphinx.ext.imgmath',
    'sphinx.ext.ifconfig',
    'sphinxcontrib.youtube',  # For youtube embedding
]


def custom_formatwarning(msg, *args, **kwargs):
    # ignore everything except the message
    return str(msg) + '\n'


warnings.formatwarning = custom_formatwarning

# Check if sphinxcontrib.youtube version is high enough to handle vimeo and older python versions
if version.parse(pkg_resources.get_distribution('sphinxcontrib-youtube').version) < version.parse('1.0.1'):
    warnings.warn('\033[93mModule sphinxcontrib-youtube is outdated. PDF documentation cannot be built. ' +
                  'Please run "python3 -m pip install --upgrade sphinxcontrib-youtube"')
    try:
        # Check if sphinxcontrib.vimeo extension is present, fallback to using that to handle vimeo
        import sphinxcontrib.vimeo  # noqa: F401
        extensions.append('sphinxcontrib.vimeo')  # For vimeo embedding
    except ImportError:  # change to ModuleNotFoundError when only python >=3.6 is supported
        warnings.warn('\033[93mModule sphinxcontrib-youtube is old and sphinxcontrib-vimeo is not installed.' +
                      'Please run the wiki build setup script.')

# Set False to re-enable warnings for non-local images.
disable_non_local_image_warnings = True


# wiki_base_url='https://dl.dropboxusercontent.com/u/3067678/share2/wiki'
# intersphinx_base_url=wiki_base_url+'/%s/build/html/'

wiki_base_url = 'https://ardupilot.org/'
intersphinx_base_url = wiki_base_url + '%s/'


# Where to point the base of the build for the main site menu
html_context = {'target': '/'}
# This needs to change to the actual URL root once the theme updated.

# Example configuration for intersphinx: refer to the Python standard library.
intersphinx_mapping = {'copter': (intersphinx_base_url % 'copter',
                                  None),
                       'plane': (intersphinx_base_url % 'plane',
                                  None),  # noqa: E127
                       'rover': (intersphinx_base_url % 'rover',
                                  None),  # noqa: E127
                       'planner': (intersphinx_base_url % 'planner',
                                  None),  # noqa: E128
                       'planner2': (intersphinx_base_url % 'planner2',
                                  None),  # noqa: E128
                       'dev': (intersphinx_base_url % 'dev',
                                  None),  # noqa: E127
                       'antennatracker': (intersphinx_base_url % 'antennatracker',
                                  None),  # noqa: E128
                       'ardupilot': (intersphinx_base_url % 'ardupilot',
                                  None),  # noqa: E128
                       'mavproxy': (intersphinx_base_url % 'mavproxy',
                                  None),  # noqa: E128
                       'blimp': (intersphinx_base_url % 'blimp',
                                  None),  # noqa: E127
                      }  # noqa: E124

# PATCH REMOVE NON-LOCAL IMAGE WARNINGS
# From:
#  http://stackoverflow.com/questions/12772927/specifying-an-online-image-in-sphinx-restructuredtext-format
#  And https://github.com/sphinx-doc/sphinx/issues/2429

# Set False to re-enable warnings for non-local images.
disable_non_local_image_warnings = True

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
