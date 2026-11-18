"""Tests for Alpha XCP Radar Lag Removal toggle, XcpRadarLogger shm export, and BoschALeadPolicy fusion."""
from __future__ import annotations

from collections import deque
import json
from types import SimpleNamespace
from unittest.mock import Mock

from opendbc.car import gen_empty_fingerprint
from opendbc.car.honda.bosch_a_radar import BOSCH_A_FREQ_HZ
from opendbc.car.honda.interface import CarInterface as HondaInterface
from opendbc.car.honda.values import CAR as HondaCAR, HondaFlags
from openpilot.common.params import Params
from openpilot.starpilot.car.honda.radar import BoschALeadPolicy
from openpilot.starpilot.car.honda.xcp_radar_assist import KEY, LABEL, capability, enabled
from openpilot.starpilot.car.honda.xcp_radar_logger import NUM_SLOTS, STRIDE, XcpRadarLogger
from openpilot.starpilot.galaxy.settings import AuthorityContext, SettingsGateway
from openpilot.starpilot.ui.feature_settings_owner import FeatureSettingsOwner
from openpilot.starpilot.ui.feature_settings_state import FeaturePage


class _DummyKalman:
  def __init__(self, dt: float):
    from openpilot.selfdrive.controls.radard import KalmanParams
    kp = KalmanParams(dt)
    self.A = kp.A
    self.C = kp.C
    self.K = kp.K


def _civic_cp(*, radar_unavailable: bool = False):
  return HondaInterface.get_params(
    HondaCAR.HONDA_CIVIC_BOSCH,
    gen_empty_fingerprint(),
    [],
    True,
    False,
    radar_unavailable,
  )


def _dummy_radar():
  return SimpleNamespace(
    ready=True,
    v_ego=20.0,
    v_ego_hist=deque([20.0], maxlen=4),
    tracks={},
    kalman_params=_DummyKalman(1.0 / BOSCH_A_FREQ_HZ),
  )


def _make_sm(mono_ns: int, *, a_ego: float = 0.0, standstill: bool = False, frame: int = 1):
  class _SM(dict):
    def __init__(self):
      super().__init__({
        "carState": SimpleNamespace(aEgo=a_ego, standstill=standstill),
        "modelV2": SimpleNamespace(leadsV3=[]),
      })
      self.recv_frame = {"radarTracks": frame}
      self.logMonoTime = {"radarTracks": mono_ns, "modelV2": mono_ns}
  return _SM()


def _make_point(track_id: int = 22, d_rel: float = 28.0, y_rel: float = 0.2, v_rel: float = -2.0, measured: bool = True):
  return SimpleNamespace(
    trackId=track_id,
    dRel=d_rel,
    yRel=y_rel,
    vRel=v_rel,
    deprecated=SimpleNamespace(measured=measured),
  )


def test_alpha_toggle_gate_and_galaxy_integration(tmp_path):
  params = Params(str(tmp_path))
  cp = _civic_cp()
  assert not enabled(params, cp)
  assert capability(cp, params) is not None

  # Unsupported variants must remain hidden/disabled.
  radarless = _civic_cp(radar_unavailable=True)
  assert capability(radarless, params) is None
  assert not enabled(params, radarless)

  other = SimpleNamespace(
    brand="toyota",
    carFingerprint="TOYOTA_RAV4_TSS2_2023",
    radarUnavailable=False,
    notCar=False,
    passive=False,
    dashcamOnly=False,
  )
  assert capability(other, params) is None

  # Galaxy gateway surfaces Alpha row on both Vehicle Settings and Long Planner (profiles), default Off.
  ctx_holder = SimpleNamespace(cp=cp, raw=cp.to_bytes(), parked=True, saved=False)
  ctx_src = Mock(sample=lambda: AuthorityContext(
    ctx_holder.parked, ctx_holder.cp, ctx_holder.raw, editing_saved_vehicle=ctx_holder.saved
  ))
  gateway = SettingsGateway(params, ctx_src, clock=lambda: 10)
  try:
    for page_name in ("vehicle", "profiles"):
      page = gateway.page(page_name, "sess", b"gen")
      row = next(r for r in page["rows"] if r["label"] == LABEL)
      assert row["value"] == "Off" and row["defaultValue"] == "Off"
      assert row["choices"] == ["Off", "On"]
      assert row["available"] is True

    # Toggle On via Galaxy preview -> confirm.
    page = gateway.page("vehicle", "sess", b"gen")
    idx = next(i for i, r in enumerate(page["rows"]) if r["label"] == LABEL)
    intent = gateway.preview(page["view"], idx, 1, "sess", b"gen")
    assert intent["proposed"] == "On"
    assert "Alpha XCP" in intent["question"] and "read-only for logging" in intent["question"]
    assert gateway.confirm(intent["intent"], "sess", b"gen")
    assert enabled(params, cp) is True

    # Release branch suppresses capability and forces enabled() False even if previously saved b"1".
    params.put_bool("IsReleaseBranch", True, block=True)
    assert capability(cp, params) is None
    assert enabled(params, cp) is False
    params.put_bool("IsReleaseBranch", False, block=True)
    assert enabled(params, cp) is True

    # Native / saved-vehicle mode (galaxy=False) hides Alpha toggle.
    non_galaxy = FeatureSettingsOwner(
      params, lambda _: True, vehicle_fingerprint=lambda: cp.carFingerprint, vehicle_params=lambda: cp, galaxy=lambda: False
    )
    assert all(r.key != KEY for r in non_galaxy.snapshot(FeaturePage.VEHICLE, parked=True, system_long=True, lateral_context=True, metric=False).rows)
    assert all(r.key != KEY for r in non_galaxy.snapshot(FeaturePage.PROFILES, parked=True, system_long=True, lateral_context=True, metric=False).rows)
  finally:
    gateway.close()


def test_policy_xcp_off_is_strictly_stock_and_on_removes_lag(tmp_path):
  t0 = 10_000_000_000
  shm_path = tmp_path / "xcp_radar_points.json"
  xcp_payload = {
    "mono_ns": t0,
    "points": [[31.39, 0.2]],
    "tracks_by_op_id": {
      "22": {
        "slot": 33,
        "rtid": 403,
        "age": 40,
        "stage": "measured",
        "ghost": False,
        "flags": 0xE0C0,
        "dRel_m": 31.39,
        "yRel_m": 0.2,
        "vRel_mps": -18.5,
        "ax_mps2": -3.2,
        "kin_age_ms": 25.0,
        "trust_accel": True,
      }
    },
    "targets": [],
  }
  shm_path.write_bytes(json.dumps(xcp_payload).encode("utf-8"))

  # Run baseline stock policy (xcp_assist=False, no file) vs xcp_assist=False with live file: must be identical.
  pol_stock = BoschALeadPolicy(xcp_assist=False, xcp_path=None)
  pol_off = BoschALeadPolicy(xcp_assist=False, xcp_path=shm_path)
  pol_on = BoschALeadPolicy(xcp_assist=True, xcp_path=shm_path)

  rad_stock, rad_off, rad_on = _dummy_radar(), _dummy_radar(), _dummy_radar()
  # Simulate 0x2C8 U11 railed at -12.0 m/s & unmeasured coast while actual XCP target is closing at -18.5 m/s, -3.2 m/s^2.
  rr = SimpleNamespace(points=[_make_point(22, 28.0, 0.2, -12.0, measured=False)])
  for step in range(3):
    ts = t0 + step * 50_000_000
    xcp_payload["mono_ns"] = ts
    shm_path.write_bytes(json.dumps(xcp_payload).encode("utf-8"))
    sm = _make_sm(ts, a_ego=0.2, frame=step + 1)
    pol_stock.update_tracks(sm, rr, rad_stock)
    pol_off.update_tracks(sm, rr, rad_off)
    pol_on.update_tracks(sm, rr, rad_on)

  t_stock, t_off, t_on = rad_stock.tracks[22], rad_off.tracks[22], rad_on.tracks[22]
  assert (t_off.vRel, t_off.vLeadK, t_off.aLeadK, t_off.cnt, t_off.direct) == (
    t_stock.vRel, t_stock.vLeadK, t_stock.aLeadK, t_stock.cnt, t_stock.direct
  )
  assert t_on.vRel == -18.5
  assert t_on.direct is True and t_on.measured is True
  assert t_on.vLead < t_off.vLead - 5.0
  assert t_on.aLeadK <= -1.4  # -3.2 + 0.2 = -3.0 blended with KF


def test_policy_gates_reject_stale_coasting_ghost_lowconf_and_clutter():
  t0 = 20_000_000_000
  cases = [
    ("stale_snapshot", t0 - 250_000_000, {"stage": "measured", "kin_age_ms": 20.0, "ghost": False, "flags": 0xE0C0, "age": 30, "dRel_m": 30.0, "yRel_m": 0.1, "vRel_mps": -8.0, "ax_mps2": -2.0, "trust_accel": True}, -2.0),
    ("stale_kin", t0, {"stage": "measured", "kin_age_ms": 250.0, "ghost": False, "flags": 0xE0C0, "age": 30, "dRel_m": 30.0, "yRel_m": 0.1, "vRel_mps": -8.0, "ax_mps2": -2.0, "trust_accel": True}, -2.0),
    ("coasting", t0, {"stage": "coasting", "kin_age_ms": 20.0, "ghost": False, "flags": 0xA0C0, "age": 30, "dRel_m": 30.0, "yRel_m": 0.1, "vRel_mps": -8.0, "ax_mps2": -2.0, "trust_accel": False}, -2.0),
    ("ghost", t0, {"stage": "measured", "kin_age_ms": 20.0, "ghost": True, "flags": 0xE0E0, "age": 30, "dRel_m": 30.0, "yRel_m": 0.1, "vRel_mps": -8.0, "ax_mps2": -2.0, "trust_accel": False}, -2.0),
    ("low_conf", t0, {"stage": "measured", "kin_age_ms": 20.0, "ghost": False, "flags": 0xF040, "age": 30, "dRel_m": 30.0, "yRel_m": 0.1, "vRel_mps": -8.0, "ax_mps2": -2.0, "trust_accel": False}, -2.0),
    ("immature_age", t0, {"stage": "measured", "kin_age_ms": 20.0, "ghost": False, "flags": 0xE0C0, "age": 4, "dRel_m": 30.0, "yRel_m": 0.1, "vRel_mps": -8.0, "ax_mps2": -2.0, "trust_accel": False}, -2.0),
    ("stationary_clutter", t0, {"stage": "measured", "kin_age_ms": 20.0, "ghost": False, "flags": 0xE0C0, "age": 30, "dRel_m": 30.0, "yRel_m": 0.1, "vRel_mps": -19.8, "ax_mps2": 0.0, "trust_accel": True}, -1.5),
  ]
  for name, snap_mono, target, op_vrel in cases:
    pol = BoschALeadPolicy(xcp_assist=True, xcp_path=None)
    rad = _dummy_radar()
    pol.set_xcp_snapshot({"mono_ns": snap_mono, "tracks_by_op_id": {"22": target}, "targets": [target]})
    rr = SimpleNamespace(points=[_make_point(22, 28.0, 0.1, op_vrel, measured=True)])
    pol.update_tracks(_make_sm(t0, frame=1), rr, rad)
    assert rad.tracks[22].vRel == op_vrel, f"{name} should have been rejected"


def test_xcp_logger_shm_export_and_trailer_fallback_replay(tmp_path):
  t0 = 30_000_000_000
  ui_path = tmp_path / "xcp_radar_points.json"
  assist_path = tmp_path / "xcp_radar_assist.json"
  cp = SimpleNamespace(brand="honda", flags=int(HondaFlags.BOSCH), passive=False)
  logger = XcpRadarLogger(cp, shm_path=ui_path, assist_shm_path=assist_path)
  logger.state = "READY"
  logger.last_v_ego = 18.0
  logger.slot_cache[33].update({
    "X_raw": int(round(31.59 * 128)),
    "Vx_raw": int(round(-6.4 * 256)),
    "Ax_raw": int(round(-2.6 * 2048)),
    "Y_raw": int(round(0.25 * 128)),
    "Vy_raw": 0,
    "age": 45,
    "flags": 0xE0C0,
    "subflags": 0x0400,
    "qual": 25,
    "cls_conf": 85,
    "radar_track_id": 403,
    "kin_updated_ns": t0 - 20_000_000,
    "lat_updated_ns": t0 - 20_000_000,
    "age_updated_ns": t0 - 20_000_000,
    "flags_updated_ns": t0 - 20_000_000,
    "updated_ns": t0 - 20_000_000,
    "last_rtt_ns": 10_000_000,
  })
  logger.active_slots[33] = logger.slot_cache[33]
  # dv = 2.2 < 3.5 with 3.39 m trailer offset populates tracks_by_op_id["22"] cleanly.
  logger.observe_radar_tracks(SimpleNamespace(points=[SimpleNamespace(trackId=22, dRel=28.20, yRel=0.20, vRel=-4.20)]), t0)
  logger._maybe_log_20hz(t0)

  ui_data = json.loads(ui_path.read_text())
  assert set(ui_data) == {"mono_ns", "points"}
  data = json.loads(assist_path.read_text())
  assert "22" in data["tracks_by_op_id"]
  assert len(data["targets"]) == 1

  # radard slightly earlier (t0 - 15 ms within future skew tolerance) reads assist shm file cleanly.
  pol = BoschALeadPolicy(xcp_assist=True, xcp_path=assist_path)
  rad = _dummy_radar()
  rr = SimpleNamespace(points=[_make_point(22, 28.20, 0.20, -4.20, measured=True)])
  pol.update_tracks(_make_sm(t0 - 15_000_000, a_ego=-0.1, frame=1), rr, rad)
  expected_vrel = round(int(round(-6.4 * 256)) / 256.0, 4)
  assert rad.tracks[22].vRel == expected_vrel
  assert rad.tracks[22].aLeadK < -1.0

  # close() unlinks both shm files cleanly.
  logger.close()
  assert not ui_path.exists()
  assert not assist_path.exists()

  # Fallback via targets list recovers larger dv (4.6 m/s) when tracks_by_op_id is absent (e.g. pre-fix rlog replay on s4 tid 22).
  pol_fb = BoschALeadPolicy(xcp_assist=True, xcp_path=None)
  rad_fb = _dummy_radar()
  pol_fb.set_xcp_snapshot({"mono_ns": t0, "targets": data["targets"]})
  rr_lag = SimpleNamespace(points=[_make_point(22, 28.20, 0.20, -1.80, measured=True)])
  pol_fb.update_tracks(_make_sm(t0, a_ego=-0.1, frame=1), rr_lag, rad_fb)
  assert rad_fb.tracks[22].vRel == expected_vrel


def test_handoff_guard_avoids_aleadk_step_spike():
  t0 = 40_000_000_000
  pol = BoschALeadPolicy(xcp_assist=True, xcp_path=None)
  rad = _dummy_radar()
  # Frames 1..3: lagged 0x280 at vRel = -1.0 m/s
  rr = SimpleNamespace(points=[_make_point(22, 35.0, 0.2, -1.0, measured=True)])
  for step in range(3):
    pol.update_tracks(_make_sm(t0 + step * 50_000_000, a_ego=0.0, frame=step + 1), rr, rad)
  assert abs(rad.tracks[22].aLeadK) < 0.2

  # Frame 4: XCP activates at vRel = -4.0 m/s with ax = -0.2 m/s^2 -> must NOT spike to -3.0 from KF step!
  t_on = t0 + 3 * 50_000_000
  target = {"stage": "measured", "kin_age_ms": 20.0, "ghost": False, "flags": 0xE0C0, "age": 30,
            "dRel_m": 35.0, "yRel_m": 0.2, "vRel_mps": -4.0, "ax_mps2": -0.2, "trust_accel": True}
  pol.set_xcp_snapshot({"mono_ns": t_on, "tracks_by_op_id": {"22": target}, "targets": [target]})
  pol.update_tracks(_make_sm(t_on, a_ego=0.0, frame=4), rr, rad)
  assert rad.tracks[22].vRel == -4.0
  assert -0.6 < rad.tracks[22].aLeadK <= 0.1

  # Exhaust hold (4 frames after expiry) -> transition back to 0x280 (-1.0) must NOT spike upward!
  pol.set_xcp_snapshot(None)
  for extra in range(1, 6):
    ts = t_on + extra * 50_000_000
    pol.update_tracks(_make_sm(ts, a_ego=0.0, frame=4 + extra), rr, rad)
  assert rad.tracks[22].vRel == -1.0
  assert rad.tracks[22].aLeadK < 0.5
