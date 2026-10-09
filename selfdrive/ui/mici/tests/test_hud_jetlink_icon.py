import pytest

from jetlink.openpilot.status import Status
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import jetlink_hud_state


def _st(**kw):
  base = dict(enabled=True, mode='usb', transport='usb', present=True, port=None, ready=True, reason=None,
              progress=None, model=None, default_model=None, standin=False)
  base.update(kw)
  return Status(**base)


@pytest.mark.parametrize("status,big,seen,want", [
  (_st(), True, True, 'green'),
  (_st(), False, True, 'orange'),
  (_st(), False, False, 'loading'),
  (_st(progress={'stage': 'joining'}), False, True, 'loading'),
  (_st(progress={'stage': 'failed'}), False, True, 'crossed'),
  (_st(present=False), False, True, 'crossed'),
  (_st(reason='no host'), False, True, 'crossed'),
  (None, False, True, 'crossed'),
  (_st(present=False), True, True, 'green'),
])
def test_jetlink_hud_state(status, big, seen, want):
  assert jetlink_hud_state(status, big, seen) == want
