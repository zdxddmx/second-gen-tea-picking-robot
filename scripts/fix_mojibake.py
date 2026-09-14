#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Repair GBK-misread em dashes in the source tree.

An earlier documentation pass edited files with PowerShell's
``Get-Content -Raw`` / ``Set-Content`` pair, which round-trips a UTF-8 file
through the console code page.  The em dash (U+2014, UTF-8 ``E2 80 94``) came
back as the GBK pair ``E2 80`` = U+9225 followed by a replacement ``?``, so
every English comment that used a dash now reads "``<U+9225>?``".

That mistake is recorded in the project's own history and is the reason all
later edits are made with Python (which reads and writes bytes).  This script
finishes the job: it finds the exact damaged sequence and restores the
character, reporting every file it touches.

It is deliberately narrow.  It looks for one codepoint pair and nothing else,
so it cannot damage legitimate Chinese text: U+9225 is a rare character that
does not occur in this project's prose, and the trailing ``?`` is required for
a match.  Anything else that looks suspicious is reported for a human to look
at rather than rewritten.
"""

from __future__ import print_function

import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DAMAGED = u"\u9225?"          # what the round-trip produced
RESTORED = u"\u2014"          # em dash, the character that was there

# One comment lost its line break along with the dash; restore it too, since the
# remaining text would otherwise leave a stray comment marker mid-line.
EXTRA = [
    (u"produced \u2014     * it refused",
     u"produced \u2014\n     * it refused"),
]

SKIP_DIRS = set([".git", "build", "__pycache__", "third_party", "images"])
SUFFIXES = (".c", ".h", ".py", ".md", ".ps1", ".sh")


def main():
    touched = []
    total = 0

    for base, dirs, names in os.walk(ROOT):
        dirs[:] = sorted(d for d in dirs if d not in SKIP_DIRS)
        for name in sorted(names):
            if not name.endswith(SUFFIXES):
                continue
            path = os.path.join(base, name)
            with io.open(path, "r", encoding="utf-8") as handle:
                text = handle.read()

            count = text.count(DAMAGED)
            updated = text.replace(DAMAGED, RESTORED)
            for old, new in EXTRA:
                if old in updated:
                    updated = updated.replace(old, new)

            if updated == text:
                continue
            with io.open(path, "w", encoding="utf-8", newline="\n") as handle:
                handle.write(updated)
            touched.append((os.path.relpath(path, ROOT).replace("\\", "/"),
                            count))
            total += count

    for path, count in touched:
        print("repaired %2d dash(es) in %s" % (count, path))
    print("-" * 70)
    print("%d file(s) repaired, %d dash(es) restored" % (len(touched), total))
    return 0


if __name__ == "__main__":
    sys.exit(main())
