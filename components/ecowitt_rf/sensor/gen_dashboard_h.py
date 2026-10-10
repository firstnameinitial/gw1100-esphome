# SPDX-License-Identifier: GPL-3.0-only
"""Generate the embedded dashboard without changing its runtime bytes."""
from pathlib import Path

root = Path(__file__).resolve().parent
html = (root / 'dashboard.html').read_text(encoding='utf-8')
out = [
    '// SPDX-License-Identifier: GPL-3.0-only',
    '#pragma once',
    '// AUTO-GENERATED from dashboard.html — do not edit by hand.',
    '// Served at / by the ecowitt_rf web dashboard.',
    'static const char DASHBOARD_HTML[] PROGMEM =',
]
for line in html.split('\n'):
    # C/C++ \? prevents legacy trigraph interpretation while preserving the byte.
    escaped = line.replace(chr(92), chr(92) * 2).replace('"', chr(92) + '"').replace('?', chr(92) + '?')
    out.append('    "' + escaped + chr(92) + 'n"')
out.append('    ;')
(root / 'dashboard_html.h').write_text('\n'.join(out) + '\n', encoding='utf-8')
