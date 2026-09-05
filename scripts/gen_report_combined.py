#!/usr/bin/env python3
"""Generate primary reports with the causal 5 Hz evaluator (no legacy simulator)."""
import argparse
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('keys', nargs='*', metavar='g1|g2|g3')
    parser.add_argument('--phase', default='realtime')
    args = parser.parse_args()
    if any(key not in {'g1', 'g2', 'g3'} for key in args.keys):
        parser.error('keys must be g1, g2 or g3')
    sys.path.insert(0, str(Path(__file__).resolve().parent / 'rerun68'))
    from analyze import main as generate
    for key in args.keys or ['g1', 'g2', 'g3']:
        generate(key, args.phase)


if __name__ == '__main__':
    main()
