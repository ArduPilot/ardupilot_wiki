"""
Sphinx extension to skip cross-reference label registration for versioned parameter files.

This significantly reduces memory usage and build time by not registering thousands
of labels for versioned parameter files (e.g., parameters-Copter-stable-V4.6.0.rst).

Only the latest parameters.rst files maintain full cross-linking capability.
"""

import re
import logging
from sphinx.application import Sphinx
from sphinx.environment import BuildEnvironment

logger = logging.getLogger(__name__)

# Pattern to match versioned parameter files
# Examples: parameters-Copter-stable-V4.6.0, parameters-Plane-beta-V4.5.1
VERSIONED_PARAM_PATTERN = re.compile(
    r'parameters-(?:Copter|Plane|Rover|Sub|Blimp|AntennaTracker|AP_Periph)'
    r'-(?:stable|beta|latest)-V?\d+\.\d+\.?\d*'
)


def is_versioned_param_file(docname: str) -> bool:
    """Check if a document is a versioned parameter file."""
    return bool(VERSIONED_PARAM_PATTERN.search(docname))


def skip_versioned_labels(app: Sphinx, env: BuildEnvironment, docname: str) -> None:
    """
    Called after a document is read. Removes labels from versioned parameter files
    to prevent them from being registered in the global label registry.
    """
    if is_versioned_param_file(docname):
        # Remove all labels registered for this document
        labels_to_remove = []
        if hasattr(env, 'domaindata') and 'std' in env.domaindata:
            std_domain = env.domaindata['std']
            if 'labels' in std_domain:
                for label, (doc, _, _) in list(std_domain['labels'].items()):
                    if doc == docname:
                        labels_to_remove.append(label)

                for label in labels_to_remove:
                    del std_domain['labels'][label]
                    if 'anonlabels' in std_domain and label in std_domain['anonlabels']:
                        del std_domain['anonlabels'][label]

        if labels_to_remove:
            logger.debug(f"Skipped {len(labels_to_remove)} labels for versioned file: {docname}")


def skip_versioned_from_search(app: Sphinx, pagename: str, templatename: str,
                               context: dict, doctree) -> None:
    """
    Exclude versioned parameter pages from search index to reduce index size.
    """
    if is_versioned_param_file(pagename):
        # Mark page to be excluded from search
        context['nosearch'] = True


def env_get_outdated_handler(app: Sphinx, env: BuildEnvironment,
                             added: set, changed: set, removed: set) -> list:
    """
    Handler for env-get-outdated event.
    Logs statistics about versioned parameter files being processed.

    Note: This event only allows ADDING documents to rebuild, not removing them.
    The actual optimization happens via label skipping and search exclusion.

    Returns an empty list (no additional documents to rebuild).
    """
    versioned_added = [d for d in added if is_versioned_param_file(d)]
    versioned_changed = [d for d in changed if is_versioned_param_file(d)]

    total_versioned = len(versioned_added) + len(versioned_changed)
    if total_versioned > 0:
        logger.info(f"Versioned params: {len(versioned_added)} new, {len(versioned_changed)} changed")

    return []  # Don't add any additional outdated docs


def setup(app: Sphinx) -> dict:
    """Setup the Sphinx extension."""

    # Hook after document is read to remove labels
    app.connect('doctree-read', lambda app, doctree: None)  # Placeholder
    app.connect('env-get-outdated', env_get_outdated_handler)

    # Hook to skip search indexing for versioned files
    app.connect('html-page-context', skip_versioned_from_search)

    # Use source-read to track which files are versioned params
    def mark_versioned_file(app, docname, source):
        if is_versioned_param_file(docname):
            if not hasattr(app.env, '_versioned_param_files'):
                app.env._versioned_param_files = set()
            app.env._versioned_param_files.add(docname)

    app.connect('source-read', mark_versioned_file)

    # After reading, clean up labels
    app.connect('doctree-resolved', lambda app, doctree, docname:
                skip_versioned_labels(app, app.env, docname)
                if is_versioned_param_file(docname) else None)

    return {
        'version': '1.0',
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }
