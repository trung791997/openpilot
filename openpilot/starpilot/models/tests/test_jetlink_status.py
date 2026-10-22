"""Jetlink's home icon and Android Auto card read one status, off the drawing thread, never keeping stale latency."""
import json
import threading
import time
from dataclasses import replace
from types import SimpleNamespace as NS
from unittest.mock import patch

import pytest

from openpilot.starpilot.models import jetlink_adapter as adapter, jetlink_status
from openpilot.starpilot.models.jetlink_status import JetlinkStatus, JetlinkStatusFeed, home_label, hud_icon, read_status


def status(**kw):
  base = {'enabled': True, 'mode': 'usb', 'present': True, 'port_state': 'connected', 'active': False, 'reason': None, 'running': None,
          'active_model': None, 'model': None, 'progress': None}
  return NS(**{**base, **kw})


def read(value, latency=24.6):
  with patch.object(adapter, 'link_on', return_value=True), \
       patch.object(adapter, 'hooks', return_value=NS(status=lambda: value)), \
       patch.object(jetlink_status, 'read_latency', return_value=latency):
    return read_status()


def test_off_reads_nothing_of_jetlink():
  with patch.object(adapter, 'link_on', return_value=False), patch.object(adapter, 'hooks') as hooks:
    assert read_status() == JetlinkStatus()
    hooks.assert_not_called()


@pytest.mark.parametrize('kw,expected', [
  ({'active': True, 'running': 'running'}, JetlinkStatus('Computer', 'active', 24.6, 'connected')),
  ({'running': 'ready'}, JetlinkStatus('Computer', 'ready', port='connected')),
  ({'running': 'joining'}, JetlinkStatus('Computer', 'connecting', port='connected')),
  ({'running': 'retrying'}, JetlinkStatus('Computer', 'connecting', port='connected')),
  ({'running': None}, JetlinkStatus('Computer', 'connected', port='connected')),
  ({'reason': 'service stopped'}, JetlinkStatus('Computer', 'unavailable', port='connected')),
  ({'present': False, 'port_state': 'device'}, JetlinkStatus('Computer', 'disconnected', port='device')),
  ({'present': False, 'port_state': 'empty'}, JetlinkStatus('Computer', 'disconnected')),
  ({'present': False, 'port_state': None}, JetlinkStatus('Computer', 'disconnected')),
  ({'mode': 'ios', 'active': True}, JetlinkStatus('iPhone', 'active', 24.6, 'connected')),
  ({'enabled': False}, JetlinkStatus()),
])
def test_states(kw, expected):
  assert replace(read(status(**kw)), hud='off', model=None, progress=None) == expected


# jetlink/openpilot/status.py: Status.active (green), present/reason (crossed), progress['stage'] and running
@pytest.mark.parametrize('kw,icon', [
  ({'active': True, 'running': 'running'}, 'active'),
  ({'running': 'ready'}, 'linked'),
  ({'running': None, 'progress': {'stage': 'ready', 'frac': 1.0}}, 'linked'),
  ({'running': 'joining'}, 'loading'),
  ({'running': 'retrying'}, 'loading'),
  ({'progress': {'stage': 'download', 'frac': 0.4}}, 'loading'),
  ({'progress': {'stage': 'connect', 'frac': 0.}}, 'loading'),
  ({'progress': {'stage': 'failed', 'frac': 0.}}, 'crossed'),
  ({'reason': 'service stopped'}, 'crossed'),
  ({'present': False}, 'crossed'),
])
def test_hud_icon(kw, icon):
  assert hud_icon(read(status(**kw))) == icon


def test_hud_icon_falls_through_to_chestnut_when_off():
  assert hud_icon(JetlinkStatus()) is None
  assert read(status(enabled=False)).hud == 'off'
  assert hud_icon(JetlinkStatus(state='unavailable')) == 'crossed'  # unreadable or stale
  assert hud_icon(read(None)) == 'crossed'


@pytest.mark.parametrize('kw,label', [
  ({'active': True, 'running': 'running', 'model': 'Pick', 'active_model': 'Pick'}, 'Jetlink: Pick'),
  # not built yet: the stand-in drives once linked (Status.active_model), else the pick it will run
  ({'running': 'ready', 'model': 'Pick', 'active_model': 'Standin'}, 'Jetlink: Standin'),
  ({'running': None, 'model': 'Pick'}, 'Jetlink: Pick'),
  ({'running': None}, 'Jetlink ready'),
  ({'progress': {'stage': 'download', 'frac': 0.427}, 'model': 'Pick'}, 'Jetlink 42%'),
  ({'progress': {'stage': 'build', 'frac': 0.}}, 'Jetlink loading'),
  ({'progress': {'stage': 'download', 'frac': 1.}}, 'Jetlink loading'),
  ({'running': 'joining'}, 'Jetlink loading'),
  ({'progress': {'stage': 'failed', 'frac': 0.5}}, 'Jetlink error'),
  ({'reason': 'service stopped'}, 'Jetlink error'),
  ({'present': False}, 'Jetlink: no host'),
  ({'enabled': False}, ''),
])
def test_home_label(kw, label):
  assert home_label(read(status(**kw))) == label


def test_home_label_off_and_unreadable():
  assert home_label(JetlinkStatus()) == ''
  assert home_label(read(None)) == 'Jetlink error'


def test_no_usable_jetlink_is_unavailable():
  assert read(None) == JetlinkStatus(state='unavailable')


def test_latency_record_round_trip_and_staleness(tmp_path):
  path = str(tmp_path / 'jetlink' / 'latency.json')
  jetlink_status._write_latency(NS(client=NS(last_timings=(1, 2, 24600))), path)
  assert jetlink_status.read_latency(path) == pytest.approx(24.6)
  jetlink_status._write_latency(NS(client=None), path)  # the small model drives
  assert jetlink_status.read_latency(path) is None
  with open(path, 'w') as f:
    json.dump({'at': time.monotonic() - jetlink_status.LATENCY_FRESH_S - 1, 'server_ms': 24.6}, f)
  assert jetlink_status.read_latency(path) is None
  assert jetlink_status.read_latency(str(tmp_path / 'missing.json')) is None


def test_feed_polls_off_thread_expires_and_disables():
  arrived = threading.Event()
  threads = []
  def fake():
    threads.append(threading.get_ident())
    arrived.set()
    return JetlinkStatus('iPhone', 'active', 24.6, 'connected')
  feed = JetlinkStatusFeed(read=fake, period=0.01)
  try:
    assert feed.snapshot() == JetlinkStatus()
    feed.set_enabled(True)
    assert arrived.wait(2)
    assert threading.get_ident() not in threads
    assert feed.snapshot().latency_ms == 24.6
    feed.set_enabled(False)
    assert feed.snapshot() == JetlinkStatus()
  finally:
    feed.close()


def test_feed_stale_and_failed_reads_drop_latency_and_port():
  feed = JetlinkStatusFeed(read=lambda: JetlinkStatus('iPhone', 'active', 24.6, 'connected'))
  feed._enabled = True
  feed.poll()
  assert feed.snapshot().port == 'connected'
  feed._read_at = 0
  assert feed.snapshot() == JetlinkStatus('iPhone', 'unavailable')
  feed.read = lambda: 1 / 0
  feed.poll()
  assert feed.snapshot() == JetlinkStatus(state='unavailable')
