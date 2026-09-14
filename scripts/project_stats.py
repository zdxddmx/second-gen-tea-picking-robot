#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Count the files and lines this project claims in its README and report.

The completion report and both READMEs quote file counts and line counts for the
C sources, the Python sources and the documentation.  Those numbers drift every
time a file is added, and a hand-edited count that is one revision out of date is
exactly the kind of small inaccuracy that makes a reader distrust the large
numbers too.  Counting them mechanically removes the possibility.

What is deliberately excluded, and why:

* ``firmware/Drivers/third_party`` - vendored STM32F4 HAL, CMSIS and FreeRTOS.
  Counting them would inflate the project by ~123 files this project did not
  write.  They are reported separately for that reason.
* ``build/`` - compiler output.
* ``__pycache__`` - byte-code caches.
* ``.git/`` - version-control metadata.

Line counts are newline counts, which is the same convention ``wc -l`` uses and
the reason every source file in this repository ends with a newline.
"""

from __future__ import print_function

import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SKIP_DIRS = set(["third_party", "build", "__pycache__", ".git", ".vscode"])

# Files quoted by name in the report; counted separately from docs/ so that the
# two totals cannot double-count each other.
ROOT_DOCS = ["README.md", "README_EN.md", "PROJECT_COMPLETION_REPORT.md"]


def count_lines(path):
    with io.open(path, "rb") as handle:
        return handle.read().count(b"\n")


def walk(relative, suffixes):
    """Return [(repo-relative path, line count)] for matching files."""
    found = []
    base_dir = os.path.join(ROOT, relative)
    for base, dirs, names in os.walk(base_dir):
        dirs[:] = sorted(d for d in dirs if d not in SKIP_DIRS)
        for name in sorted(names):
            if name.endswith(suffixes):
                path = os.path.join(base, name)
                found.append((os.path.relpath(path, ROOT).replace("\\", "/"),
                              count_lines(path)))
    return found


def report(label, files):
    total = sum(lines for _, lines in files)
    print("%-46s %4d files %8d lines" % (label, len(files), total))
    return len(files), total


def main():
    print("=" * 74)
    print(" project statistics (vendored third-party sources excluded)")
    print("=" * 74)

    c = walk("firmware", (".c", ".h"))
    c_tests = [f for f in c if f[0].startswith("firmware/Tests/")]
    py = walk(".", (".py",))
    docs = walk("docs", (".md",))
    hw = walk("hardware", (".md",))
    root_docs = [(name, count_lines(os.path.join(ROOT, name)))
                 for name in ROOT_DOCS
                 if os.path.isfile(os.path.join(ROOT, name))]

    c_n = report("C sources + headers", c)
    report("  of which firmware/Tests", c_tests)
    py_n = report("Python sources", py)
    doc_n = report("Documentation (docs/*.md)", docs)
    hw_n = report("Hardware docs (hardware/*.md)", hw)
    root_n = report("README zh + en + completion report", root_docs)

    vendored = 0
    for _, _, names in os.walk(os.path.join(ROOT, "firmware", "Drivers",
                                            "third_party")):
        vendored += len(names)

    print("-" * 74)
    files = c_n[0] + py_n[0] + doc_n[0] + hw_n[0] + root_n[0]
    lines = c_n[1] + py_n[1] + doc_n[1] + hw_n[1] + root_n[1]
    print("%-46s %4d files %8d lines" % ("TOTAL (own code + documentation)",
                                         files, lines))
    print("%-46s %4d files" % ("vendored third-party (not counted)", vendored))
    print("=" * 74)
    return 0


if __name__ == "__main__":
    sys.exit(main())
