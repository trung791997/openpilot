"""The standalone tools must run on Python 3.9.

They exist to work on a laptop with no openpilot build, and the Python they will actually meet
there is the macOS Command Line Tools build — **3.9**. That is older than this repo's own
target (`pyproject.toml` sets py311), so the normal lint and test run does not notice a 3.10+
or 3.11+ construct creeping into them. It gets noticed when a user runs the tool and it dies
at import, which is exactly what happened with

    from datetime import UTC, datetime
    ImportError: cannot import name 'UTC'   # datetime.UTC is 3.11+

These tests check the two failure shapes separately: syntax the 3.9 parser rejects, and names
that parse fine but do not exist until later. The second list is not exhaustive and cannot be
— it is a ratchet for things that have actually bitten, so add to it when one does.
"""

import ast
import os
import re

import pytest

_HERE = os.path.dirname(__file__)
_TOOLS = os.path.normpath(os.path.join(_HERE, "..", ".."))

# Tools a user runs directly on a machine with no build. Anything added here must stay 3.9-safe.
STANDALONE = ["konik_login.py", "plain_http.py", "konik_preflight.py", "konik_fetch.py"]

# (regex, what it is, first version). Runtime names, so the parser will not catch them.
TOO_NEW = [
  (r"\bdatetime\.UTC\b", "datetime.UTC", "3.11"),
  (r"from datetime import (?:[^\n]*[, ])?UTC\b", "from datetime import UTC", "3.11"),
  (r"\bdatetime\.datetime\.UTC\b", "datetime.datetime.UTC", "3.11"),
  (r"\bitertools\.pairwise\b", "itertools.pairwise", "3.10"),
  (r"\bitertools\.batched\b", "itertools.batched", "3.12"),
  (r"\btomllib\b", "tomllib", "3.11"),
  (r"\bExceptionGroup\b", "ExceptionGroup", "3.11"),
  (r"\benum\.StrEnum\b|\bStrEnum\b", "StrEnum", "3.11"),
  (r"\bzoneinfo\b|\bZoneInfo\b", "zoneinfo", "3.9 stdlib but tzdata-dependent"),
  (r"\.removeprefix\(|\.removesuffix\(", "str.removeprefix/removesuffix", "3.9 — ok, listed for review"),
  (r"\bhashlib\.file_digest\b", "hashlib.file_digest", "3.11"),
  (r"\basyncio\.TaskGroup\b", "asyncio.TaskGroup", "3.11"),
]
# Entries whose "first version" is actually <= 3.9 and are only here for visibility.
ALLOWED = {"str.removeprefix/removesuffix", "zoneinfo"}


def _read(name):
  with open(os.path.join(_TOOLS, name)) as f:
    return f.read()


@pytest.mark.parametrize("name", STANDALONE)
class TestPy39:
  def test_parses_under_the_39_grammar(self, name):
    """Catches match statements, PEP 604 outside annotations, PEP 695 generics, etc."""
    src = _read(name)
    try:
      ast.parse(src, filename=name, feature_version=(3, 9))
    except SyntaxError as e:
      pytest.fail(f"{name} is not valid Python 3.9 syntax: line {e.lineno}: {e.msg}")

  def test_uses_no_names_newer_than_39(self, name):
    src = _read(name)
    # Strip comments so a note *about* a construct does not trip its own check.
    body = "\n".join(line.split("#", 1)[0] for line in src.splitlines())
    offenders = []
    for pattern, label, version in TOO_NEW:
      if label in ALLOWED:
        continue
      m = re.search(pattern, body)
      if m:
        line = body[:m.start()].count("\n") + 1
        offenders.append(f"{label} (needs {version}) at line {line}")
    assert not offenders, f"{name} uses names unavailable on 3.9: " + "; ".join(offenders)

  def test_postponed_annotations_are_enabled(self, name):
    """`X | None` in a signature is only safe on 3.9 because annotations are strings."""
    src = _read(name)
    if re.search(r"->\s*[\w\[\]]+\s*\|", src) or re.search(r":\s*[\w\[\]]+\s*\|\s*None\s*[=,)]", src):
      assert "from __future__ import annotations" in src, (
        f"{name} uses PEP 604 unions in annotations, which need "
        + "`from __future__ import annotations` to work on 3.9"
      )


class TestTheRegressionThatPromptedThis:
  def test_preflight_does_not_import_datetime_utc(self):
    """The exact break a user hit: ImportError on `from datetime import UTC` under 3.9."""
    src = _read("konik_preflight.py")
    body = "\n".join(line.split("#", 1)[0] for line in src.splitlines())
    assert "from datetime import UTC" not in body
    assert "datetime.UTC" not in body
    assert "timezone.utc" in body, "the 3.9-safe spelling should be used instead"

  def test_timezone_is_imported_where_it_is_used(self):
    src = _read("konik_preflight.py")
    assert re.search(r"from datetime import .*\btimezone\b", src)
