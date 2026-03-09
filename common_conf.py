# This contains common configuration information for the ardupilot wikis.
# This information is imported by the conf.py files in each of the sub wikis


# Parallel reading of source files (use all available CPUs)
parallel_read_safe = True
parallel_write_safe = True

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'sphinx.ext.mathjax',  # For :math: element rendering
    'sphinx.ext.ifconfig',
    'sphinxcontrib.youtube',  # For youtube embedding
    'sphinxcontrib.jquery',
    'sphinx_tabs.tabs',  # For clickable tabs
]

# wiki_base_url='https://dl.dropboxusercontent.com/u/3067678/share2/wiki'
# intersphinx_base_url=wiki_base_url+'/%s/build/html/'

wiki_base_url = 'https://ardupilot.org/'
intersphinx_base_url = wiki_base_url + '%s/'


# Where to point the base of the build for the main site menu
html_context = {'target': '/'}
# This needs to change to the actual URL root once the theme updated.

# Don't generate search index for versioned parameter pages
html_search_options = {
    'dict_max_word_length': 40,  # Skip very long parameter names
}

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


# Suppress warnings that slow down builds (already have nitpicky = False)
suppress_warnings = [
    'epub.unknown_project_files',  # Suppress epub warnings
]

disable_non_local_image_warnings = True

if disable_non_local_image_warnings:
    suppress_warnings.append('image.nonlocal_uri')  # Suppress external image warnings


def setup(app):
    app.add_css_file("common_theme_override.css")
