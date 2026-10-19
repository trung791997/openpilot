"""manager's Jetlink power-off on the way down: only on DoShutdown with the link on, and bounded (StarPilot#231)."""
from types import SimpleNamespace

from openpilot.starpilot import jetlink_adapter
from openpilot.starpilot.system.jetlink import lifecycle


class Params:
  def __init__(self, **values):
    self.values = values

  def get_bool(self, key):
    return bool(self.values.get(key))


class Clock:
  def __init__(self):
    self.t = 0.

  def __call__(self):
    return self.t

  def sleep(self, dt):
    self.t += dt


def _link(monkeypatch, on, steps):
  calls = []
  def shutdown_step(should_shutdown, reason=''):
    calls.append((should_shutdown, reason))
    return steps(len(calls))
  monkeypatch.setattr(jetlink_adapter, 'hooks', lambda: SimpleNamespace(shutdown_step=shutdown_step))
  monkeypatch.setattr(jetlink_adapter, 'link_on', lambda: on)
  return calls


def test_reboot_or_link_off_never_powers_the_host_off(monkeypatch):
  calls = _link(monkeypatch, True, lambda n: True)
  lifecycle.shutdown(Params(DoReboot=True))
  assert calls == []
  calls = _link(monkeypatch, False, lambda n: True)
  lifecycle.shutdown(Params(DoShutdown=True))
  assert calls == []


def test_shutdown_steps_until_done(monkeypatch):
  calls = _link(monkeypatch, True, lambda n: n >= 3)
  clock = Clock()
  lifecycle.shutdown(Params(DoShutdown=True), clock=clock, sleep=clock.sleep)
  assert calls == [(True, 'DoShutdown')] * 3


def test_shutdown_is_capped(monkeypatch):
  calls = _link(monkeypatch, True, lambda n: False)   # a host that never answers
  clock = Clock()
  lifecycle.shutdown(Params(DoShutdown=True), clock=clock, sleep=clock.sleep)
  assert clock.t <= lifecycle.SHUTDOWN_CAP_S + 0.2 and clock.t >= lifecycle.SHUTDOWN_CAP_S - 0.2
  assert len(calls) > 1
