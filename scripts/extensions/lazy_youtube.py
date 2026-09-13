"""Sphinx extension: every video embed sphinxcontrib-youtube renders is
loading="lazy", so a page does not fetch a player for a video nobody has
played. Listed after sphinxcontrib.youtube in conf; overrides its HTML
visitors and touches nothing else."""

from sphinx.util import logging
from sphinxcontrib.youtube import peertube, utils, vimeo, youtube

logger = logging.getLogger(__name__)


def _lazy(visit):
    def visit_lazy(self, node):
        start = len(self.body)
        visit(self, node)
        # The visitor appended the wrapper div, the iframe tag, then the close.
        for i in range(len(self.body) - 1, start - 1, -1):
            tag = self.body[i]
            if tag.startswith("<iframe"):
                if "loading=" not in tag:
                    self.body[i] = tag.replace("<iframe", '<iframe loading="lazy"', 1)
                return
        logger.warning("lazy_youtube: sphinxcontrib-youtube rendered no <iframe>; "
                       "this embed is not lazy", location=node)
    return visit_lazy


def setup(app):
    app.setup_extension("sphinxcontrib.youtube")
    for node_class, visitors in (
        (youtube.youtube, youtube._NODE_VISITORS),
        (vimeo.vimeo, utils._NODE_VISITORS),
        (peertube.peertube, peertube._NODE_VISITORS),
    ):
        visitors = dict(visitors)
        visit, depart = visitors["html"]
        visitors["html"] = (_lazy(visit), depart)
        app.add_node(node_class, override=True, **visitors)
    return {"parallel_read_safe": True, "parallel_write_safe": True}
