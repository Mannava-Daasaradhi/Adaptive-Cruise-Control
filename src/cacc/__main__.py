"""``python -m cacc`` — same entry point as the ``cacc`` console script."""

import sys

from cacc.cli import main

if __name__ == "__main__":  # guard required: evaluate uses process pools
    sys.exit(main())
