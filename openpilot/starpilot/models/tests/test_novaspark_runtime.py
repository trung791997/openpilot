"""NovaSpark selection: it loads only on the f6fc4e3 runtime and falls back to the default after a failed start."""
import hashlib
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from openpilot.starpilot.models import manager
from openpilot.starpilot.models.catalog import DEFAULT_SMALL
from openpilot.starpilot.models.manager import ModelError, ModelManager, atomic_json, catalog, preferences, \
  randomize_next_start, resolve_runtime
from openpilot.starpilot.models.runtime_bridge import legacy_hcq, runtime
from openpilot.starpilot.models.runtime_bridge.bootstrap import NOVASPARK_REVISION
from openpilot.starpilot.models.tests import test_manager

SHA = 'ab' * 32


class NovaSparkSelectionTest(unittest.TestCase):
  setUp = test_manager.ModelManagerTest.setUp

  def install(self):
    entries = catalog(self.root)
    entries['novaspark'] = dict(entries['novaspark'], artifact_sha256=hashlib.sha256(self.data).hexdigest(),
                                artifact_size=len(self.data))
    path = self.root / 'novaspark/novaspark_driving_tinygrad.pkl'
    path.parent.mkdir(parents=True)
    path.write_bytes(self.data)
    patcher = patch('openpilot.starpilot.models.manager.catalog', return_value=entries)
    patcher.start()
    self.addCleanup(patcher.stop)
    return path

  def test_catalog_row_is_local_and_pinned_to_f6(self):
    row = catalog(self.root)['novaspark']
    self.assertTrue(row['local_only'])
    self.assertEqual(row['tinygrad_runtime'], NOVASPARK_REVISION)
    self.assertFalse(row['uses_external_gpu'])

  def test_selected_only_by_the_runtime_it_was_built_for(self):
    path = self.install()
    atomic_json(self.root / 'preferences.json', {'small': 'novaspark'})
    self.assertEqual(preferences(self.root)['small'], 'novaspark')
    default = resolve_runtime(True, root=self.root)
    self.assertEqual(default.small_id, DEFAULT_SMALL)
    self.assertEqual(default.small_path.read_bytes(), self.data)  # the shipped default bytes in this fixture
    selected = resolve_runtime(True, root=self.root, tinygrad_runtime=NOVASPARK_REVISION)
    self.assertEqual(selected.small_id, 'novaspark')
    self.assertEqual(selected.small_path, path)

  def test_f6_runtime_never_loads_a_9d_model(self):
    self.install()
    atomic_json(self.root / 'preferences.json', {'small': DEFAULT_SMALL})
    self.assertEqual(resolve_runtime(True, root=self.root, tinygrad_runtime=NOVASPARK_REVISION).small_id, DEFAULT_SMALL)

  def test_unverified_bytes_fall_back(self):
    path = self.install()
    path.write_bytes(b'x' * len(self.data))
    atomic_json(self.root / 'preferences.json', {'small': 'novaspark'})
    self.assertEqual(resolve_runtime(True, root=self.root, tinygrad_runtime=NOVASPARK_REVISION).small_id, DEFAULT_SMALL)

  def test_randomizer_never_picks_it(self):
    self.install()
    atomic_json(self.root / 'preferences.json', {'small': DEFAULT_SMALL, 'randomizer': True})
    seen = []
    randomize_next_start(False, root=self.root, chooser=lambda c: seen.append(list(c)) or c[0])
    self.assertTrue(seen)
    self.assertNotIn('novaspark', seen[0])

  def test_download_refused(self):
    row = catalog(self.root)['novaspark']
    with self.assertRaisesRegex(ModelError, 'installed locally'):
      ModelManager._download(SimpleNamespace(root=self.root), 'novaspark', row)


class F6RuntimeSwitchTest(unittest.TestCase):
  def setUp(self):
    temp = tempfile.TemporaryDirectory()
    self.addCleanup(temp.cleanup)
    self.markers = Path(temp.name) / 'markers'
    for name, value in (('MARKER_DIR', self.markers), ('_f6_sha', None), ('_selection', None)):
      p = patch.object(runtime, name, value)
      p.start()
      self.addCleanup(p.stop)
    self.prefs = {'small': 'novaspark', 'big': '', 'randomizer': False}
    self.entries = {'novaspark': {'tinygrad_runtime': NOVASPARK_REVISION, 'artifact_sha256': SHA}}
    self.verified = Path('/verified')
    for name, kw in (('preferences', {'side_effect': lambda: self.prefs}),
                     ('catalog', {'side_effect': lambda: self.entries}),
                     ('verified_artifact', {'side_effect': lambda *a: self.verified})):
      p = patch.object(manager, name, **kw)
      p.start()
      self.addCleanup(p.stop)

  def marker(self, kind):
    return self.markers / f'f6-{kind}-{SHA}'

  def test_candidate_when_selected_and_verified(self):
    self.assertEqual(runtime._f6_candidate(), SHA)

  def test_candidate_skipped(self):
    cases = {
      'randomizer': lambda: self.prefs.update(randomizer=True),
      'big model': lambda: self.prefs.update(big='bigmodel'),
      'other runtime': lambda: self.entries['novaspark'].update(tinygrad_runtime=None),
      'unverified': lambda: setattr(self, 'verified', None),
      'not selected': lambda: self.prefs.update(small=DEFAULT_SMALL),
    }
    for name, change in cases.items():
      with self.subTest(name):
        saved = (dict(self.prefs), {k: dict(v) for k, v in self.entries.items()}, self.verified)
        change()
        self.assertIsNone(runtime._f6_candidate())
        self.prefs, self.entries, self.verified = saved

  def test_failed_marker_blocks(self):
    self.markers.mkdir()
    self.marker('failed').write_bytes(b'')
    self.assertIsNone(runtime._f6_candidate())

  def test_unfinished_start_becomes_failed(self):
    self.markers.mkdir()
    self.marker('attempt').write_bytes(b'')
    self.assertIsNone(runtime._f6_candidate())
    self.assertFalse(self.marker('attempt').exists())
    self.assertTrue(self.marker('failed').exists())
    self.assertIsNone(runtime._f6_candidate())

  def test_candidate_errors_mean_default(self):
    self.entries['novaspark'].pop('artifact_sha256')
    self.assertIsNone(runtime._f6_candidate())

  def prepare(self, activate_f6):
    with patch('openpilot.common.hardware.AGNOS', True), patch.object(runtime, '_verify_policy'), \
         patch.object(runtime, '_activate_f6', side_effect=activate_f6), \
         patch.object(runtime, 'activate', side_effect=AssertionError('9d runtime selected')):
      return runtime.prepare('driving')

  def test_setup_failure_exits_77_and_records_failure(self):
    def fail(sha):
      raise RuntimeError('bundle changed')
    with self.assertRaises(SystemExit) as raised:
      self.prepare(fail)
    self.assertEqual(raised.exception.code, runtime.F6_FALLBACK_EXIT)
    self.assertTrue(self.marker('failed').exists())
    self.assertFalse(self.marker('attempt').exists())

  def test_setup_success_returns_selection(self):
    self.assertEqual(self.prepare(lambda sha: {'role': 'driving'}), {'role': 'driving'})

  def test_default_runtime_when_not_a_candidate(self):
    self.prefs['small'] = DEFAULT_SMALL
    with patch('openpilot.common.hardware.AGNOS', True), patch.object(runtime, '_verify_policy'), \
         patch.object(runtime, '_activate_f6', side_effect=AssertionError('f6 selected')), \
         patch.object(runtime, 'activate', return_value={'role': 'driving'}) as activate:
      self.assertEqual(runtime.prepare('driving'), {'role': 'driving'})
    self.assertEqual(activate.call_args.args[0], runtime.BUNDLE)
    self.assertIsNone(runtime.active_revision())

  def test_load_markers(self):
    runtime.f6_failed()
    runtime.f6_loaded()
    self.assertFalse(self.markers.exists())
    self.markers.mkdir()
    runtime._f6_sha = SHA
    self.assertEqual(runtime.active_revision(), NOVASPARK_REVISION)
    self.marker('attempt').write_bytes(b'')
    runtime.f6_loaded()
    self.assertFalse(self.marker('attempt').exists())
    self.assertFalse(self.marker('failed').exists())
    self.marker('attempt').write_bytes(b'')
    runtime.f6_failed()
    self.assertFalse(self.marker('attempt').exists())
    self.assertTrue(self.marker('failed').exists())

  def test_guard_hash_is_pinned(self):
    guard = Path(runtime.__file__).with_name('legacy_hcq.py')
    self.assertEqual(hashlib.sha256(guard.read_bytes()).hexdigest(), runtime.F6_GUARD_SHA256)

  def test_changed_guard_refused_before_any_marker(self):
    with patch.object(runtime, 'F6_GUARD_SHA256', '0' * 64), self.assertRaisesRegex(RuntimeError, 'bounded wait'):
      runtime._activate_f6(SHA)
    self.assertFalse(self.markers.exists())


class BoundedWaitFatalTest(unittest.TestCase):
  def test_fatal_records_then_exits_even_if_hook_fails(self):
    calls = []
    for hook in (lambda: calls.append('hook'), lambda: 1 / 0):
      with self.subTest(hook=hook), patch.object(legacy_hcq, 'on_fatal', hook), \
           patch('os._exit', side_effect=lambda code: calls.append(code) or (_ for _ in ()).throw(SystemExit(code))):
        with self.assertRaises(SystemExit):
          legacy_hcq._fatal(legacy_hcq.EXIT_DEADLINE)
    self.assertEqual(calls, ['hook', legacy_hcq.EXIT_DEADLINE, legacy_hcq.EXIT_DEADLINE])

  def test_deadline_exit_through_fatal(self):
    signal = SimpleNamespace(value=0, _sleep=lambda ms: None)
    with patch.object(legacy_hcq, 'DEADLINE_NS', 0), \
         patch.object(legacy_hcq, '_fatal', side_effect=SystemExit) as fatal, self.assertRaises(SystemExit):
      legacy_hcq._wait(signal, 1)
    fatal.assert_called_once_with(legacy_hcq.EXIT_DEADLINE)



class BlinkerReleaseFlushTest(unittest.TestCase):
  def state(self, enabled):
    from openpilot.starpilot.models.runner import CatalogModelState
    queue = SimpleNamespace(cleared=0)
    queue.assign = lambda value: setattr(queue, 'cleared', queue.cleared + 1)
    model = object.__new__(CatalogModelState)
    model.__dict__.update(flush_desire_on_blinker_release=enabled, blinker_on=False, prev_blinker_on=False,
                          desire_key='desire_q', input_queues={'desire_q': queue})
    return model, queue

  def drive(self, model, blinker):
    for on in blinker:
      model.blinker_on = on
      model._flush_desire_on_blinker_release()

  def test_flush_only_on_falling_edge(self):
    model, queue = self.state(True)
    self.drive(model, [False, True, True, False, False, True, False])
    self.assertEqual(queue.cleared, 2)

  def test_other_models_never_flush(self):
    model, queue = self.state(False)
    self.drive(model, [True, False, True, False])
    self.assertEqual(queue.cleared, 0)


if __name__ == '__main__':
  unittest.main()
