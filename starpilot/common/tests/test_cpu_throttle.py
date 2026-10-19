import pytest

from openpilot.starpilot.common import cpu_throttle


def test_offline_cpu_placeholders_do_not_hide_sustained_load(monkeypatch):
  monkeypatch.setattr(cpu_throttle, "_online_cpu_count", lambda: 4)

  usage = cpu_throttle._online_cpu_usage([80, 80, 80, 80, 0, 0, 0, 0])
  average = sum(usage) / len(usage)

  assert usage == [80.0, 80.0, 80.0, 80.0]
  assert cpu_throttle._compute_throttle_factor(average, 0) == pytest.approx(1.75)


def test_idle_reserved_cores_do_not_hide_affinity_core_saturation(monkeypatch):
  monkeypatch.setattr(cpu_throttle, "_online_cpu_count", lambda: 8)

  usage = cpu_throttle._online_cpu_usage([74, 82, 100, 85, 92, 98, 0, 47], cores=[2])

  assert usage == [100.0]
  assert cpu_throttle._compute_throttle_factor(sum(usage) / len(usage), 1) == 4.0


@pytest.mark.parametrize(("online_spec", "expected"), (
  ("0-3", 4),
  ("0-3,6-7", 6),
))
def test_online_cpu_count_parser(monkeypatch, tmp_path, online_spec, expected):
  online_path = tmp_path / "online"
  online_path.write_text(online_spec, encoding="utf-8")
  monkeypatch.setattr(cpu_throttle, "Path", lambda _: online_path)
  cpu_throttle._online_cpu_count.cache_clear()

  assert cpu_throttle._online_cpu_count() == expected


def test_own_cpu_is_left_out_of_the_throttle(monkeypatch):
  monkeypatch.setattr(cpu_throttle, "_online_cpu_count", lambda: 8)
  clock = iter(range(0, 1000, 10))
  monkeypatch.setattr(cpu_throttle.time, "monotonic", lambda: next(clock))

  def settled(usage, cores, own):
    monkeypatch.setattr(cpu_throttle, "_throttle_state", {})
    cpu_throttle.device_cpu_throttle_factor(usage, name="own", cores=cores, own_percent=own)
    return cpu_throttle.device_cpu_throttle_factor(usage, name="own", cores=cores, own_percent=own)

  usage = [50, 50, 78, 50, 50, 50, 50, 50]
  # onroad 2026-10-19: core 2 at 78%, ~30% of it V-ASM. Counting itself it stretched 1.5x; the others' 48% is not busy
  assert settled(usage, [2], 0.0) == pytest.approx(1.5, abs=0.01)
  assert settled(usage, [2], 30.0) == 1.0
  # three cores, own 30% spread as 10% each: 84% -> 74%, at the busy line
  assert settled([84, 84, 84, 50], [0, 1, 2], 30.0) == 1.0


def test_others_load_still_throttles(monkeypatch):
  monkeypatch.setattr(cpu_throttle, "_online_cpu_count", lambda: 8)
  monkeypatch.setattr(cpu_throttle, "_throttle_state", {})
  clock = iter([0.0, 10.0])
  monkeypatch.setattr(cpu_throttle.time, "monotonic", lambda: next(clock))
  cpu_throttle.device_cpu_throttle_factor([0, 0, 100, 0], name="busy", cores=[2], own_percent=10.0)
  # others at 90% of core 2: target 1 + (90 - 74) / 8 = 3.0, reached after the 10 s low-pass
  assert cpu_throttle.device_cpu_throttle_factor([0, 0, 100, 0], name="busy", cores=[2], own_percent=10.0) == pytest.approx(3.0, abs=0.01)


def test_own_cpu_percent_low_pass():
  t = {"wall": 0.0, "cpu": 0.0}
  own = cpu_throttle.OwnCpuPercent(tau=1.0, clock=lambda: t["wall"], cpu=lambda: t["cpu"])
  assert own.update() == 0.0
  for _ in range(100):  # 0.4 s of CPU per 1 s of wall: 40% of one core
    t["wall"] += 0.1
    t["cpu"] += 0.04
    own.update()
  assert own.percent == pytest.approx(40.0, abs=0.5)
