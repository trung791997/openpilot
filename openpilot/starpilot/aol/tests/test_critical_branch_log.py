"""The AOL critical-branch log is diagnostic only: rate limited, never raises, never gates."""
from unittest.mock import patch

from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD


def bare():
  sd = SelfdriveD.__new__(SelfdriveD)
  sd.enabled = True
  sd.aol_fault_reason = 'critical'
  return sd


def test_logs_branch_and_ages_in_ms_once_per_second_per_branch():
  sd = bare()
  with patch('openpilot.selfdrive.selfdrived.selfdrived.cloudlog') as log, \
       patch('openpilot.selfdrive.selfdrived.selfdrived.time.monotonic', side_effect=[10.0, 10.4, 10.5, 11.2]):
    sd._log_aol_critical('output_lease_expired', 0, lambda: dict(lease_overrun_ns=5_000_000, source_age_ns=None))
    sd._log_aol_critical('output_lease_expired', 0, lambda: dict(lease_overrun_ns=6_000_000))  # rate limited
    sd._log_aol_critical('source_expired', 0, lambda: dict(source_age_ns=61_000_000))  # other branch is independent
    sd._log_aol_critical('output_lease_expired', 0, lambda: dict(lease_overrun_ns=7_000_000))  # 1.2 s later
  calls = [c.kwargs for c in log.event.call_args_list]
  assert [c['branch'] for c in calls] == ['output_lease_expired', 'source_expired', 'output_lease_expired']
  assert calls[0]['lease_overrun_ns'] == 5.0 and 'source_age_ns' not in calls[0]
  assert calls[1]['source_age_ns'] == 61.0 and calls[0]['was_enabled'] is True


def test_logging_failure_never_propagates():
  sd = bare()
  with patch('openpilot.selfdrive.selfdrived.selfdrived.cloudlog') as log:
    log.event.side_effect = RuntimeError('swaglog down')
    sd._log_aol_critical('native_missing', 0, lambda: dict(wire_age_ns=1))


def test_age_expression_failure_never_propagates():
  sd = bare()
  with patch('openpilot.selfdrive.selfdrived.selfdrived.cloudlog'):
    sd._log_aol_critical('native_missing', 0, lambda: dict(wire_age_ns=sd.sm.logMonoTime['aolSafetyWire']))  # no sm
