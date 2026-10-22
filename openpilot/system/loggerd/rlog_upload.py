# StarPilot: which rlogs UploadRlogs sends, shared by the uploader and the hardwared shutdown hold.
#
# The first drive seen with the toggle on (or with the device parked) is the anchor, stored as an xattr on the log
# root. Every rlog from the anchor route onward goes up oldest first, so an unfinished previous drive completes before
# the one in progress. Routes older than the anchor (the backlog from before rlogs were first sent) are never sent.
# With UploadRlogs off the rlogs still go up while the device is parked, so the anchor is kept and a drive's rlogs
# finish after parking. Ported from origin/jetlink-087 (32201f7197, 42b1e66e5b, 475f054e4b).
#
# Kept free of uploader imports (Api, cereal, the connect provider) so hardwared can use it without an import cycle.
# Upload marks are read uncached here because the uploader sets them from another process. The upload mark name is
# per connect provider (user.upload or user.upload.konik), so callers pass it in.
import errno
import os
from collections.abc import Callable

from openpilot.system.loggerd.xattr_cache import _getxattr, _setxattr

ANCHOR_ATTR_NAME = "user.rlog_upload_from"
UPLOAD_ATTR_VALUE = b"1"
RLOG_NAMES = ("rlog", "rlog.zst")

_NO_ATTR_ERRNOS = {errno.ENOENT, errno.ENODATA, getattr(errno, "ENOATTR", errno.ENODATA)}


def route_of(logdir: str) -> str | None:
  route, sep, seg = logdir.rpartition("--")
  return route if sep and route and seg.isdigit() else None


def route_sort_key(route: str) -> tuple[str, str]:
  # the old date-named format sorts first, as in uploader.get_directory_sort
  return ("0" if route.startswith("2024-") else "1", route)


def segment_dirs(root: str) -> list[str]:
  try:
    dirs = [d for d in os.listdir(root) if route_of(d) is not None and os.path.isdir(os.path.join(root, d))]
  except OSError:
    return []
  return sorted(dirs, key=lambda d: (route_sort_key(route_of(d)), int(d.rpartition("--")[2])))


def current_route(root: str) -> str | None:
  dirs = segment_dirs(root)
  return route_of(dirs[-1]) if dirs else None


def _read_xattr(path: str, attr_name: str) -> bytes | None:
  try:
    return _getxattr(path, attr_name)
  except OSError as e:
    if e.errno in _NO_ATTR_ERRNOS:
      return None
    raise


def get_anchor(root: str) -> str | None:
  try:
    value = _read_xattr(root, ANCHOR_ATTR_NAME)
  except OSError:
    return None
  return value.decode() if value else None


def ensure_anchor(root: str) -> str | None:
  anchor = get_anchor(root)
  if anchor is None:
    anchor = current_route(root)
    if anchor is not None:
      try:
        _setxattr(root, ANCHOR_ATTR_NAME, anchor.encode())
      except OSError:
        pass  # no xattr support: fall back to the drive in progress each time
  return anchor


def in_upload_window(logdir: str, anchor: str | None) -> bool:
  route = route_of(logdir)
  return route is not None and anchor is not None and route_sort_key(route) >= route_sort_key(anchor)


def rlogs_pending(root: str, upload_attr_name: str, owns: Callable[[str], bool] = lambda _: True) -> bool:
  anchor = get_anchor(root)
  if anchor is None:
    return False
  for logdir in segment_dirs(root):
    if not in_upload_window(logdir, anchor):
      continue
    try:
      # the uploader never sends a locked segment, so a stale lock must not hold shutdown
      if any(n.endswith(".lock") for n in os.listdir(os.path.join(root, logdir))):
        continue
    except OSError:
      continue
    for name in RLOG_NAMES:
      fn = os.path.join(root, logdir, name)
      try:
        if os.path.getsize(fn) > 0 and owns(fn) and _read_xattr(fn, upload_attr_name) != UPLOAD_ATTR_VALUE:
          return True
      except OSError:
        continue
  return False
