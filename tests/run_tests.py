"""Zero-dependency test runner for the Python half of the project.

Why this file exists
--------------------
``pytest`` is the right tool and the test modules in this directory are written
to be ``pytest``-compatible: they contain plain ``test_*`` functions with plain
``assert`` statements and no pytest-specific imports.  But the machine this
project was developed on had **no network access and therefore no pytest**, so a
suite that could only be run with pytest would have been a suite nobody could
run -- including the reviewer opening the repository.

This runner discovers ``test_*`` functions in every ``test_*.py`` module next to
it and executes them with the standard library only.  That gives three things:

* ``python tests/run_tests.py`` works everywhere, with no installation step;
* ``python -m pytest tests/`` works too, and picks up exactly the same tests;
* the discovery logic is deliberately the same shape as pytest's, so a test that
  passes here passes there.

Every test must be self-contained: no shared mutable state between tests, no
reliance on execution order, and no network or hardware access.
"""

from __future__ import print_function

import importlib.util
import os
import sys
import time
import traceback
from typing import Callable, List, Sequence, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# The tests import packages from the repository root (`host`, `tools`,
# `simulator`), so the root has to be on the path before anything is loaded.
for _p in (ROOT, HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _load_module(path: str):
    """Import a module from an explicit file path.

    The tests live in a directory that is not a package, so they cannot be
    imported by name.  Loading by path keeps the runner independent of the
    current working directory, which matters because this script is invoked both
    from the repository root and from inside ``tests/``.
    """
    name = os.path.splitext(os.path.basename(path))[0]
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError("cannot load %s" % path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _discover() -> List[Tuple[str, str, Callable[[], None]]]:
    """Collect ``(module, name, callable)`` for every ``test_*`` function."""
    found = []  # type: List[Tuple[str, str, Callable[[], None]]]
    for filename in sorted(os.listdir(HERE)):
        if not (filename.startswith("test_") and filename.endswith(".py")):
            continue
        path = os.path.join(HERE, filename)
        module = _load_module(path)
        for attr in sorted(dir(module)):
            if not attr.startswith("test_"):
                continue
            fn = getattr(module, attr)
            if callable(fn):
                found.append((filename, attr, fn))
    return found


def _run_one(filename: str, name: str, fn: Callable[[], None]) -> bool:
    """Run one test, printing its outcome.  Returns True when it passed."""
    start = time.time()
    try:
        fn()
    except Exception:  # noqa: BLE001 - the runner must report *any* failure
        elapsed = time.time() - start
        print("    FAIL %s::%s  (%.2fs)" % (filename, name, elapsed))
        for line in traceback.format_exc().rstrip().splitlines():
            print("      " + line)
        return False
    elapsed = time.time() - start
    print("    ok   %-46s (%.2fs)" % (name, elapsed))
    return True


def main(argv: Sequence[str]) -> int:
    """Discover and run every test, then print a summary.

    ``--list`` prints the discovered tests without running them, which is useful
    when checking that a newly added file is actually being picked up.
    """
    tests = _discover()
    list_only = "--list" in argv

    print("=" * 62)
    print(" tea-picking robot - Python test suite")
    print(" %d test(s) discovered in %s" % (len(tests), os.path.relpath(HERE, ROOT)))
    print("=" * 62)

    if list_only:
        current = None
        for filename, name, _ in tests:
            if filename != current:
                current = filename
                print("\n[%s]" % current)
            print("    %s" % name)
        return 0

    passed = 0
    failed = 0
    current = None
    start = time.time()
    for filename, name, fn in tests:
        if filename != current:
            current = filename
            print("\n[%s]" % current)
        if _run_one(filename, name, fn):
            passed += 1
        else:
            failed += 1

    print("\n" + "=" * 62)
    print(" tests run    : %d" % (passed + failed))
    print(" passed       : %d" % passed)
    print(" failed       : %d" % failed)
    print(" wall time    : %.2f s" % (time.time() - start))
    print(" RESULT: %s" % ("PASS" if failed == 0 else "FAIL"))
    print("=" * 62)
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
