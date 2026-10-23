import base64
from pathlib import Path
from unittest.mock import patch

import pytest

from openpilot.starpilot.connect import provider as p


class ParamsFiles:
  def __init__(self, root):
    self.root = root
    root.mkdir()

  def get_param_path(self, key):
    return str(self.root / key)

  def check_key(self, key):
    assert key in (*p.STATE_KEYS, *p.TRANSIENT_KEYS, 'ConnectProvider')

  def put(self, key, value, block=True):
    self.check_key(key)
    (self.root / key).write_bytes(value.encode() if isinstance(value, str) else value)

  def remove(self, key):
    (self.root / key).unlink(missing_ok=True)

  def get(self, key):
    path = self.root / key
    return path.read_text() if path.exists() else None


@pytest.fixture
def environment(tmp_path):
  return tmp_path / 'cloud', ParamsFiles(tmp_path / 'params')


def choose(root, name, boot='first'):
  return p.select_provider(name, p.status(root)['revision'], lambda: True, root=root, boot=lambda: boot)


def comma_state(root):
  """Provider state as a build that allowed comma leaves it."""
  p._write(root / 'provider.json', {'active': 'comma', 'selected': 'comma', 'requestedBoot': ''})


def test_comma_cannot_be_selected_or_listed(environment):
  root, _ = environment
  with pytest.raises(ValueError):
    choose(root, 'comma')
  assert [entry['id'] for entry in p.status(root)['providers']] == ['konik']
  assert p.status(root)['active'] == 'konik'


def test_comma_state_switches_to_konik_and_keeps_the_comma_identity(environment):
  root, params = environment
  comma_state(root)
  params.put('DongleId', '0123456789abcdef')
  params.put('PairingEmail', 'owner@example.test')
  params.put('AccessToken', 'transient')
  original = {key: Path(params.get_param_path(key)).read_bytes() if Path(params.get_param_path(key)).exists() else None for key in p.STATE_KEYS}
  p.activate_at_boot(params, root=root, boot=lambda: 'first')
  assert params.get('DongleId') is None and params.get('AccessToken') is None
  assert params.get('ConnectProvider') == 'konik'
  assert p.configuration(root)['active'] == 'konik'
  assert p._read(root / 'comma' / 'params.json') == {key: None if raw is None else base64.b64encode(raw).decode()
                                                     for key, raw in original.items()}
  params.put('DongleId', 'fedcba9876543210')   # Konik registered
  p.activate_at_boot(params, root=root, boot=lambda: 'second')
  assert params.get('DongleId') == 'fedcba9876543210'


def test_bad_switch_journal_keeps_the_identity(environment):
  root, params = environment
  comma_state(root)
  params.put('DongleId', '0123456789abcdef')
  bad = dict.fromkeys(p.STATE_KEYS)
  bad['DongleId'] = 'not base64!'
  p._write(root / 'switch.json', {'target': 'konik', 'values': bad})
  with pytest.raises(ValueError):
    p.activate_at_boot(params, root=root, boot=lambda: 'second')
  assert params.get('DongleId') == '0123456789abcdef'


def test_default_boot_never_sends_a_comma_registration_to_konik(environment):
  root, params = environment
  params.put('DongleId', '0123456789abcdef')
  p.activate_at_boot(params, root=root)
  assert params.get('ConnectProvider') == 'konik'
  assert params.get('DongleId') is None
  assert base64.b64decode(p._read(root / 'comma' / 'params.json')['DongleId']).decode() == '0123456789abcdef'


def test_corrupt_provider_goes_offline_without_selecting_other_account(environment):
  root, _ = environment
  root.mkdir()
  (root / 'provider.json').write_text('{')
  with patch.object(p, 'root_path', return_value=root):
    assert p.active_provider() == p.OFFLINE


def test_konik_identity_is_separate_private_and_stable(environment):
  root, _ = environment
  with patch.object(p, 'root_path', return_value=root):
    first = p.konik_key_pair()
    assert p.konik_key_pair() == first
  assert first[0] == 'RS256' and 'RSA PRIVATE KEY' in first[1]
  assert (root / 'konik/identity.json').stat().st_mode & 0o777 == 0o600


def test_recordings_are_never_uploaded_under_another_provider(tmp_path):
  old = tmp_path / 'old--0'
  old.mkdir()
  konik = tmp_path / 'new--0'
  konik.mkdir()
  (konik / p.ROUTE_MARKER).write_text('konik')
  assert p.owns_recording(old / 'qlog.zst', tmp_path, 'comma')
  assert not p.owns_recording(old / 'qlog.zst', tmp_path, 'konik')
  assert p.owns_recording(konik / 'qlog.zst', tmp_path, 'konik')
  assert not p.owns_recording(konik / 'qlog.zst', tmp_path, 'comma')
  assert not p.owns_recording(tmp_path / '../elsewhere', tmp_path, 'comma')
  (konik / p.ROUTE_MARKER).write_text('unknown')
  assert not p.owns_recording(konik / 'qlog.zst', tmp_path, 'comma')
  (konik / p.ROUTE_MARKER).unlink()
  (konik / p.ROUTE_MARKER).symlink_to('/dev/null')
  assert not p.owns_recording(konik / 'qlog.zst', tmp_path, 'comma')


def test_actual_params_registration_switch_and_cache_isolation(tmp_path):
  from openpilot.common.params import Params
  params = Params(str(tmp_path / 'real-params'))
  root = tmp_path / 'real-cloud'
  comma_state(root)
  params.put('DongleId', '0123456789abcdef', block=True)
  params.put('ApiCache_Device', '{"owner":"original"}', block=True)
  p.activate_at_boot(params, root=root, boot=lambda: 'next')
  assert params.get('ConnectProvider') == 'konik'
  assert params.get('DongleId') is None and params.get('ApiCache_Device') is None


def test_registration_uses_konik_key_and_query_contract(environment):
  from openpilot.system.athena import registration
  from types import SimpleNamespace
  import jwt
  root, params = environment
  with patch.object(p, 'root_path', return_value=root):
    algorithm, private, public = p.konik_key_pair()
  response = SimpleNamespace(status_code=200, json=lambda: {'dongle_id': 'fedcba9876543210', 'access_token': ''})
  with patch.object(registration, 'get_key_pair', return_value=(algorithm, private, public)), \
       patch.object(registration, 'api_get', return_value=response) as request, \
       patch.object(registration.HARDWARE, 'get_imei', return_value='test-imei'), \
       patch.object(registration.HARDWARE, 'get_serial', return_value='test-serial'):
    assert registration.register_konik(params) == 'fedcba9876543210'
  args = request.call_args.kwargs
  assert args['public_key'] == public and args['timeout'] == 10
  assert args['serial'] == 'test-serial' and args['imei'] == 'test-imei'
  assert jwt.decode(args['register_token'], public, algorithms=['RS256'])['register'] is True
  assert params.get('DongleId') == 'fedcba9876543210'


def test_api_never_sends_credentials_when_configuration_invalid(environment):
  from openpilot.common import api
  root, _ = environment
  root.mkdir()
  (root / 'provider.json').write_text('{')
  with patch.object(p, 'root_path', return_value=root), patch.object(api.requests, 'request') as request:
    assert api.get_key_pair() == (None, None, None)
    with pytest.raises(RuntimeError):
      api.api_get('v1/me', access_token='private')
    request.assert_not_called()


def test_cloudlogs_stay_with_their_provider(environment):
  root, _ = environment
  with patch.object(p, 'root_path', return_value=root), patch.object(p.Paths, 'swaglog_root', return_value='/tmp/cloudlogs'):
    assert p.cloudlog_root() == '/tmp/cloudlogs/konik'   # Konik by default
    comma_state(root)
    assert p.cloudlog_root() == '/tmp/cloudlogs'
    p._write(root / 'provider.json', {'active': 'konik', 'selected': 'konik', 'requestedBoot': ''})
    assert p.cloudlog_root() == '/tmp/cloudlogs/konik'


def test_bootlogs_are_owned_per_file_without_retagging_legacy(tmp_path):
  boot = tmp_path / 'boot'
  boot.mkdir()
  (boot / 'old.zst').touch()
  (boot / 'new.zst').touch()
  (boot / '.cloud-new.zst').write_text('konik')
  assert p.owns_recording(boot / 'old.zst', tmp_path, 'comma')
  assert not p.owns_recording(boot / 'old.zst', tmp_path, 'konik')
  assert p.owns_recording(boot / 'new.zst', tmp_path, 'konik')
  assert not p.owns_recording(boot / 'new.zst', tmp_path, 'comma')
  (boot / '.cloud-new.zst').write_text('')
  assert not p.owns_recording(boot / 'new.zst', tmp_path, 'comma')
  assert not p.owns_recording(boot / 'new.zst', tmp_path, 'konik')


def test_galaxy_identity_stays_physical_when_konik_config_is_corrupt(environment):
  root, params = environment
  params.put('DongleId', '0123456789abcdef')
  comma_state(root)
  p.activate_at_boot(params, root=root, boot=lambda: 'second')
  params.put('DongleId', 'fedcba9876543210')
  with patch.object(p, 'root_path', return_value=root):
    assert p.galaxy_device_id(params) == '0123456789abcdef'
    (root / 'provider.json').write_text('{')
    assert p.galaxy_device_id(params) == '0123456789abcdef'


def test_live_url_needs_a_cloud_and_a_registered_device():
  with patch.object(p, 'recording_device_id', return_value='0123456789abcdef'):
    assert p.live_url(None, p.PROVIDERS['konik']) == 'https://stable.konik.ai/0123456789abcdef/live'
    assert p.live_url(None, p.OFFLINE) is None
  with patch.object(p, 'recording_device_id', return_value=None):
    assert p.live_url(None, p.PROVIDERS['konik']) is None
