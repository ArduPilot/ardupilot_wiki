# -*- coding: utf-8 -*-
#
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
