"""Tests for tools/konik_fetch.py."""

from __future__ import annotations

import ast
import importlib.util
import os
import subprocess

import pytest

_HERE = os.path.dirname(__file__)
_SCRIPT = os.path.normpath(os.path.join(_HERE, "..", "..", "konik_fetch.py"))


def _load():
  spec = importlib.util.spec_from_file_location("konik_fetch", _SCRIPT)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


fetch = _load()


class TestParseSegments:
  def test_parse_segments(self):
    assert fetch.parse_segments("0-2,5") == {0, 1, 2, 5}
    assert fetch.parse_segments(None) is None
    assert fetch.parse_segments("") is None
    assert fetch.parse_segments("  ") is None


class TestSegmentOf:
  def test_segment_of_url_with_query_string(self):
    url = "https://h/connectdata/d/00000231--5782493b00/7/rlog.zst?sig=x"
    assert fetch.segment_of(url) == 7

    url_qlog = "https://example.com/connectdata/d/00000231--5782493b00/42/qlog.zst?sig=xyz&foo=bar"
    assert fetch.segment_of(url_qlog) == 42

  def test_segment_of_invalid_url_raises(self):
    with pytest.raises(ValueError, match="cannot extract segment"):
      fetch.segment_of("https://example.com/rlog.zst")


class TestDestination:
  def test_destination_layout_strips_dongle(self):
    out = "/tmp/download_dir"
    route = "11c8fa231c0499ed|00000231--5782493b00"
    dst = fetch.destination(out, route, 7, "rlog.zst")
    expected = os.path.join(out, "00000231--5782493b00", "7", "rlog.zst")
    assert dst == expected


class TestDownloadOne:
  def test_download_one_success_and_token_in_stdin_only(self, tmp_path):
    dst = str(tmp_path / "route" / "0" / "rlog.zst")
    token = "TEST_JWT_SECRET_TOKEN_12345"
    url = "https://example.com/segment/0/rlog.zst?sig=1"
    recorded = {}

    def fake_run(argv, **kw):
      recorded["argv"] = argv
      recorded["input"] = kw.get("input")
      part_path = dst + ".part"
      with open(part_path, "wb") as f:
        f.write(b"content-of-segment")
      return subprocess.CompletedProcess(argv, 0)

    ok = fetch.download_one(url, token, dst, run=fake_run)
    assert ok is True
    assert os.path.exists(dst)
    assert not os.path.exists(dst + ".part")
    assert os.path.getsize(dst) == len(b"content-of-segment")

    argv_str = " ".join(recorded["argv"])
    assert token not in argv_str, "token must not appear in argv"
    assert recorded["argv"][1:3] == ["-K", "-"]
    assert token in recorded["input"].decode("utf-8"), "token must appear in input (stdin)"

  def test_download_one_failure_cleans_up(self, tmp_path):
    dst = str(tmp_path / "route" / "0" / "rlog.zst")
    token = "TEST_JWT_SECRET_TOKEN_12345"
    url = "https://example.com/segment/0/rlog.zst?sig=1"

    def fake_run(argv, **kw):
      part_path = dst + ".part"
      with open(part_path, "wb") as f:
        f.write(b"partial-data")
      return subprocess.CompletedProcess(argv, 22)

    ok = fetch.download_one(url, token, dst, run=fake_run)
    assert ok is False
    assert not os.path.exists(dst)
    assert not os.path.exists(dst + ".part")

  def test_download_one_skips_existing_non_empty(self, tmp_path):
    dst = str(tmp_path / "route" / "0" / "rlog.zst")
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    with open(dst, "wb") as f:
      f.write(b"pre-existing non-empty file")

    def fake_run(*a, **kw):
      raise AssertionError("run should not be called for existing non-empty file")

    ok = fetch.download_one("https://example.com/0/rlog.zst", "token", dst, run=fake_run)
    assert ok is True


class TestMain:
  def test_main_missing_token_returns_2(self, tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(fetch, "_token", lambda host: None)
    rc = fetch.main(["--route", "dongle|route1", "--out", str(tmp_path)])
    assert rc == 2
    err = capsys.readouterr().err
    assert "tools/konik_login.py" in err

  def test_main_empty_logs_returns_3(self, tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(fetch, "_token", lambda host: "mock-token")
    monkeypatch.setattr(fetch, "get_json", lambda url, headers=None: {"logs": [], "qlogs": []})
    rc = fetch.main(["--route", "dongle|route1", "--out", str(tmp_path)])
    assert rc == 3
    err = capsys.readouterr().err
    assert "rlogs are not uploaded yet" in err

  def test_main_segments_filter(self, tmp_path, monkeypatch):
    monkeypatch.setattr(fetch, "_token", lambda host: "mock-token")
    urls = [
      "https://example.com/d/00000001--abc/0/rlog.zst?sig=0",
      "https://example.com/d/00000001--abc/1/rlog.zst?sig=1",
      "https://example.com/d/00000001--abc/2/rlog.zst?sig=2",
      "https://example.com/d/00000001--abc/5/rlog.zst?sig=5",
    ]
    monkeypatch.setattr(fetch, "get_json", lambda url, headers=None: {"logs": urls})

    downloaded = []

    def fake_dl(url, token, dst, **kw):
      downloaded.append((url, dst))
      os.makedirs(os.path.dirname(dst), exist_ok=True)
      with open(dst, "wb") as f:
        f.write(b"data")
      return True

    monkeypatch.setattr(fetch, "download_one", fake_dl)
    rc = fetch.main(["--route", "dongle|00000001--abc", "--out", str(tmp_path), "--segments", "1,5"])
    assert rc == 0
    assert len(downloaded) == 2
    segs = [fetch.segment_of(u) for u, _ in downloaded]
    assert segs == [1, 5]

  def test_main_dry_run_calls_no_download(self, tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(fetch, "_token", lambda host: "mock-token")
    urls = [
      "https://example.com/d/00000001--abc/0/rlog.zst?sig=0",
      "https://example.com/d/00000001--abc/1/rlog.zst?sig=1",
    ]
    monkeypatch.setattr(fetch, "get_json", lambda url, headers=None: {"logs": urls})

    def fail_dl(*a, **kw):
      raise AssertionError("download_one must not be called during dry-run")

    monkeypatch.setattr(fetch, "download_one", fail_dl)
    rc = fetch.main(["--route", "dongle|00000001--abc", "--out", str(tmp_path), "--dry-run"])
    assert rc == 0
    out = capsys.readouterr().out
    assert "0 " in out
    assert "1 " in out
    assert "00000001--abc/0/rlog.zst" in out
    assert "00000001--abc/1/rlog.zst" in out

  def test_main_one_failed_download_returns_1(self, tmp_path, monkeypatch):
    monkeypatch.setattr(fetch, "_token", lambda host: "mock-token")
    urls = [
      "https://example.com/d/00000001--abc/0/rlog.zst?sig=0",
      "https://example.com/d/00000001--abc/1/rlog.zst?sig=1",
    ]
    monkeypatch.setattr(fetch, "get_json", lambda url, headers=None: {"logs": urls})

    def fake_dl_fail(url, token, dst, **kw):
      if "/0/" in url:
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, "wb") as f:
          f.write(b"data")
        return True
      return False

    monkeypatch.setattr(fetch, "download_one", fake_dl_fail)
    rc = fetch.main(["--route", "dongle|00000001--abc", "--out", str(tmp_path)])
    assert rc == 1


class TestPy39Compatibility:
  def test_ast_parse_feature_version_39(self):
    with open(_SCRIPT) as f:
      src = f.read()
    tree = ast.parse(src, filename="konik_fetch.py", feature_version=(3, 9))
    assert tree is not None
