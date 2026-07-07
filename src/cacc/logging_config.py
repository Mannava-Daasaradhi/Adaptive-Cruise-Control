"""Central logging: console (INFO) + rotating file ``logs/cacc.log`` (DEBUG).

Call :func:`setup_logging` once at every entry point (script, notebook, test
session). All package modules log through ``logging.getLogger(__name__)``,
which puts them under the ``cacc`` logger tree configured here. When an error
pops up, ``logs/cacc.log`` has the DEBUG-level trail.
"""

from __future__ import annotations

import logging
import logging.handlers
from pathlib import Path

_FORMAT = "%(asctime)s %(levelname)-7s %(name)s: %(message)s"


def setup_logging(
    log_dir: str | Path = "logs",
    console_level: int = logging.INFO,
    file_level: int = logging.DEBUG,
) -> logging.Logger:
    """Configure the ``cacc`` logger tree. Idempotent.

    Returns the root ``cacc`` logger. The rotating file handler keeps
    ``cacc.log`` plus 3 backups of 2 MB each in ``log_dir`` (created if
    missing, gitignored).
    """
    root = logging.getLogger("cacc")
    if getattr(root, "_cacc_configured", False):
        return root
    root.setLevel(logging.DEBUG)

    console = logging.StreamHandler()
    console.setLevel(console_level)
    console.setFormatter(logging.Formatter("%(levelname)-7s %(name)s: %(message)s"))
    root.addHandler(console)

    log_dir = Path(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    file_handler = logging.handlers.RotatingFileHandler(
        log_dir / "cacc.log", maxBytes=2_000_000, backupCount=3, encoding="utf-8"
    )
    file_handler.setLevel(file_level)
    file_handler.setFormatter(logging.Formatter(_FORMAT))
    root.addHandler(file_handler)

    root._cacc_configured = True  # type: ignore[attr-defined]
    root.debug("logging configured (file: %s)", log_dir / "cacc.log")
    return root
