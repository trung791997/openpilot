"""The comma connects the phone-calls link to the car the head unit names, when no car is chosen yet."""
import threading
from unittest.mock import Mock

from openpilot.starpilot.system.android_auto import supervisor as supervisor_module
from openpilot.starpilot.system.android_auto.supervisor import Supervisor

CAR = 'F8:36:9B:0A:7D:C8'
ADAPTER = '76:63:E9:53:CA:2C'


def fixture(companion=''):
  unit = Supervisor.__new__(Supervisor)
  unit.config = {'receiver_address': ADAPTER, 'companion_address': companion, 'companion_name': ''}
  unit._lock = threading.RLock()
  unit._shared_bluetooth_owner = Mock()
  unit._shared_bluetooth_owner.prepare_companion.return_value = {'address': CAR, 'name': 'HandsFreeLink'}
  unit.log = Mock()
  return unit


def test_connects_and_saves_the_car_the_head_unit_names(monkeypatch):
  saved = []
  monkeypatch.setattr(supervisor_module.identity_store, 'save_config', lambda config: saved.append(dict(config)))
  unit, session = fixture(), Mock(car_address=CAR, phone_link_ready=False)
  unit._connect_phone_link(session, CAR)
  args, kwargs = unit._shared_bluetooth_owner.prepare_companion.call_args
  assert args[0] == CAR and args[1]() == CAR and kwargs == {'connect': True}
  assert unit.config['companion_address'] == CAR and saved[-1]['companion_name'] == 'HandsFreeLink'
  assert session.phone_link_ready


def test_a_refused_car_is_not_saved(monkeypatch):
  monkeypatch.setattr(supervisor_module.identity_store, 'save_config', Mock(side_effect=AssertionError))
  unit, session = fixture(), Mock(car_address=CAR, phone_link_ready=False)
  unit._shared_bluetooth_owner.prepare_companion.side_effect = RuntimeError('Selected paired hands-free car is unavailable')
  unit._connect_phone_link(session, CAR)
  assert unit.config['companion_address'] == '' and not session.phone_link_ready
  assert unit.log.call_args.args[0] == 'phone_link_failed'


def test_skips_when_a_car_is_chosen_or_the_head_unit_names_the_adapter(monkeypatch):
  started = []
  monkeypatch.setattr(supervisor_module.threading, 'Thread', lambda **kwargs: started.append(kwargs) or Mock())
  fixture(companion='AA:BB:CC:DD:EE:FF')._start_phone_link(Mock(car_address=CAR))
  fixture()._start_phone_link(Mock(car_address=ADAPTER))
  fixture()._start_phone_link(Mock(car_address=''))
  assert started == []
  fixture()._start_phone_link(Mock(car_address=CAR))
  assert len(started) == 1 and started[0]['args'][1] == CAR
