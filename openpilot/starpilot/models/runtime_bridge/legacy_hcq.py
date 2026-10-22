"""Bound every QCOM host wait in the f6fc4e3 tinygrad runtime (NovaSpark).

Same contract as bounded_wait.c for the 9d0446 runtime: a wait that has not
finished 30 s after it started ends the process with exit code 73. f6fc4e3 waits
in Python (HCQSignal.wait), and its QCOM sleep blocks in the kernel for up to
0xffffffff ms, so both are replaced here.
"""
import errno
import os
import time

DEADLINE_NS = 30_000_000_000
SLEEP_MS = 100
EXIT_DEADLINE = 73
EXIT_CLOCK = 74
EXIT_IOCTL = 76

_installed = False
# Called once before a bounded-wait exit, so the caller can record the failure.
on_fatal = None


def _fatal(code):
  try:
    if on_fatal is not None:
      on_fatal()
  except BaseException:
    pass
  os._exit(code)


def _wait(self, value, timeout=None):
  # timeout is ignored: QCOM cannot recover (can_recover=False), so f6 never passes one.
  try:
    deadline = time.monotonic_ns() + DEADLINE_NS
  except Exception:
    _fatal(EXIT_CLOCK)
  while self.value < value:
    try:
      now = time.monotonic_ns()
    except Exception:
      _fatal(EXIT_CLOCK)
    if now >= deadline:
      _fatal(EXIT_DEADLINE)
    self._sleep(min(SLEEP_MS, (deadline - now) // 1_000_000 + 1))


def _qcom_sleep(self, budget_ms):
  if not (self.is_timeline and self.owner is not None):
    return
  from tinygrad.runtime.autogen import kgsl
  try:
    kgsl.IOCTL_KGSL_DEVICE_WAITTIMESTAMP_CTXTID(self.owner.fd, context_id=self.owner.ctx,
                                                timestamp=self.owner.last_cmd, timeout=max(1, min(SLEEP_MS, int(budget_ms))))
  except OSError as e:
    if e.errno not in (errno.ETIMEDOUT, errno.EINTR, errno.EAGAIN):
      _fatal(EXIT_IOCTL)
  except Exception:
    _fatal(EXIT_IOCTL)


def install(bundle_root):
  """Patch the f6 runtime in place. Call before any QCOM device is opened."""
  global _installed
  if _installed:
    return
  if 'HCQDEV_WAIT_TIMEOUT_MS' in os.environ:
    raise RuntimeError('HCQDEV_WAIT_TIMEOUT_MS would change the bounded QCOM wait')
  from tinygrad.runtime.support import hcq
  from tinygrad.runtime import ops_qcom
  tree = os.path.realpath(os.path.join(bundle_root, 'tinygrad_repo', 'tinygrad')) + os.sep
  for module in (hcq, ops_qcom):
    if not os.path.realpath(module.__file__).startswith(tree):
      raise RuntimeError('Bounded wait target is not the f6 runtime: ' + module.__file__)
  if ops_qcom.QCOMSignal.__bases__ != (hcq.HCQSignal,) or 'wait' in ops_qcom.QCOMSignal.__dict__:
    raise RuntimeError('Unexpected QCOM signal layout')
  for name in ('wait', '_sleep'):
    if name not in (hcq.HCQSignal.__dict__ if name == 'wait' else ops_qcom.QCOMSignal.__dict__):
      raise RuntimeError('Missing QCOM signal method: ' + name)
  hcq.HCQSignal.wait = _wait
  ops_qcom.QCOMSignal._sleep = _qcom_sleep
  _installed = True
