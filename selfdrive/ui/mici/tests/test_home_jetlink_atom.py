import pytest

from openpilot.selfdrive.ui.mici.layouts.home import JetlinkStatusAtom
from jetlink.openpilot.status import Status


def _status(**kw):
  base = dict(enabled=True, mode='usb', transport='USB', present=True, port='host', ready=True, reason=None,
              progress=None, model='Mountain Dew V2', default_model=None, standin=None)
  base.update(kw)
  return Status(**base)


@pytest.mark.parametrize("kw,expected", [
  (dict(), ('ready', "Jetlink: Mountain Dew V2")),
  (dict(model=None), ('ready', "Jetlink ready")),
  (dict(present=False), ('disconnected', "Jetlink: no host")),
  (dict(ready=False), ('uncompiled', "Jetlink: not built")),
  (dict(progress={'stage': 'build', 'frac': 0.42}), ('loading', "Jetlink 42%")),
  (dict(progress={'stage': 'failed', 'frac': 0.0}), ('failed', "Jetlink failed")),
  (dict(reason="no warp built for this camera"), ('failed', "Jetlink error")),
])
def test_describe(kw, expected):
  assert JetlinkStatusAtom.describe(_status(**kw)) == expected


def test_describe_hidden():
  assert JetlinkStatusAtom.describe(None) is None
  assert JetlinkStatusAtom.describe(_status(enabled=False)) is None
