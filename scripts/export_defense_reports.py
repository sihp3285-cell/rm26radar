#!/usr/bin/env python3
"""Export six final reports and their directly linked evidence, without raw data."""
import hashlib
import json
import shutil
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'log'
DEST = ROOT / 'reports'
NAMES = ('report_68g1', 'report_68g2', 'report_68g3',
         'report_68g1_armor_dyn', 'report_68g3_cov_cmp', 'report_68g3_navgrid_cmp')


class Links(HTMLParser):
    def __init__(self):
        super().__init__()
        self.paths = []

    def handle_starttag(self, tag, attrs):
        for key, value in attrs:
            if key in ('href', 'src') and value:
                url = urlsplit(value)
                if not url.scheme and not url.netloc and url.path:
                    self.paths.append(unquote(url.path))


def main():
    selected = set()
    for name in NAMES:
        report = SOURCE / name / 'report.html'
        selected.add(report)
        parser = Links()
        parser.feed(report.read_text())
        selected.update((report.parent / link).resolve() for link in parser.paths)
    for path in selected:
        path.relative_to(SOURCE)
        if not path.is_file() or path.stat().st_size > 10 * 1024 * 1024:
            raise ValueError(f'Missing or unexpectedly large report dependency: {path}')
        if path.suffix not in ('.html', '.json', '.patch', '.png', '.svg', '.css', '.js'):
            raise ValueError(f'Unexpected report dependency: {path}')
    manifest = {}
    for path in sorted(selected):
        relative = path.relative_to(SOURCE)
        target = DEST / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        assert hashlib.sha256(target.read_bytes()).hexdigest() == digest
        manifest[str(relative)] = digest
    (DEST / 'sha256.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(f'Exported and verified {len(manifest)} files, '
          f'{sum((DEST / p).stat().st_size for p in manifest):,} bytes')


if __name__ == '__main__':
    main()
