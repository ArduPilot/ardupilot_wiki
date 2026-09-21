"""Shared RST parameter-section deduplication helper.

Used by build_parameters.py (RNGFND sections) and update.py (NET_ sections).
Both scripts run from the repo root, so ``from scripts.dedupe_params import``
works as a namespace-package import without an __init__.py.
"""

import logging
import os
import re

_logger = logging.getLogger(__name__)


def dedupe_rst_parameter_sections(
    filepath: str,
    section_anchor_re: re.Pattern,
    next_item_re: re.Pattern,
    key_group: int = 0,
) -> None:
    """Remove duplicate RST parameter-section header blocks from *filepath* in-place.

    A section header block begins with a label matching ``section_anchor_re``.
    When the same key (extracted via ``key_group`` of the match) appears more
    than once, the repeated header block is dropped.  Scanning for the end of a
    header block stops at the first line that matches ``next_item_re`` or
    another ``section_anchor_re``.

    Args:
        filepath:          Path to the RST file to process in-place.
        section_anchor_re: Compiled regex matching a section header anchor line.
        next_item_re:      Compiled regex marking the first content line of the
                           section (used to stop consuming/skipping the block).
        key_group:         Group index of ``section_anchor_re`` used as the dedup
                           key.  0 = the full match string; use a capturing group
                           index to differentiate numbered families (e.g. RNGFND1
                           vs RNGFND2).
    """
    _logger.debug("Dedupe check: %s", filepath)
    if not os.path.exists(filepath):
        _logger.warning("File not found for deduplication: %s", filepath)
        return
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
    except (OSError, UnicodeDecodeError) as e:
        _logger.error("Failed to read %s for deduplication: %s", filepath, e)
        return

    seen_keys: set = set()
    output: list = []
    i = 0
    while i < len(lines):
        m = section_anchor_re.match(lines[i].strip())
        if m:
            key = m.group(key_group)
            is_duplicate = key in seen_keys
            seen_keys.add(key)

            if not is_duplicate:
                output.append(lines[i])
            i += 1

            # Consume the section header block (blank lines + title + underline)
            # until we reach the first content item or another section anchor.
            while i < len(lines) \
                    and not next_item_re.match(lines[i]) \
                    and not section_anchor_re.match(lines[i].strip()):
                if not is_duplicate:
                    output.append(lines[i])
                i += 1
            continue

        output.append(lines[i])
        i += 1

    if len(output) != len(lines):
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.writelines(output)
            _logger.debug(
                "Dedupe applied to %s: removed %d line(s)",
                filepath, len(lines) - len(output),
            )
        except (OSError, UnicodeDecodeError) as e:
            _logger.error("Failed to write deduped file %s: %s", filepath, e)


def dedupe_old_rangefinder_parameters(filepath: str) -> None:
    """Remove duplicate RNGFNDx_ parameter sections from *filepath* in-place."""
    dedupe_rst_parameter_sections(
        filepath,
        section_anchor_re=re.compile(r'^\.\.\s_parameters_RNGFND([0-9A-Za-z]+)_.*:$', re.IGNORECASE),
        next_item_re=re.compile(r'^\.\.\s_RNGFND[0-9A-Za-z]+.*:$', re.IGNORECASE),
        key_group=1,
    )


def dedupe_periph_net_parameters(filepath: str) -> None:
    """Remove duplicate NET_ parameter sections from *filepath* in-place."""
    dedupe_rst_parameter_sections(
        filepath,
        section_anchor_re=re.compile(r'^\.\. _parameters_NET_:\s*$', re.IGNORECASE),
        next_item_re=re.compile(r'^\.\. _NET_[A-Za-z0-9_+:]+:'),
    )
