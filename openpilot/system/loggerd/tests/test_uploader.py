import os
import threading
import logging
import json
from pathlib import Path
from openpilot.common.hardware.hw import Paths

from openpilot.common.swaglog import cloudlog
from openpilot.system.loggerd.uploader import clear_locks, main, Uploader, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE

from openpilot.system.loggerd.rlog_upload import ANCHOR_ATTR_NAME, ensure_anchor, get_anchor, rlogs_pending
from openpilot.system.loggerd.tests.loggerd_tests_common import UploaderTestCase


class FakeLogHandler(logging.Handler):
  def __init__(self):
    logging.Handler.__init__(self)
    self.condition = threading.Condition()
    self.reset()

  def reset(self):
    with self.condition:
      self.upload_order = []
      self.upload_ignored = []

  def emit(self, record):
    try:
      j = json.loads(record.getMessage())
      with self.condition:
        if j["event"] == "upload_success":
          self.upload_order.append(j["key"])
        if j["event"] == "upload_ignored":
          self.upload_ignored.append(j["key"])
        self.condition.notify_all()
    except Exception:
      pass

  def wait_for_uploads(self, count: int, ignored: bool = False):
    uploads = self.upload_ignored if ignored else self.upload_order
    with self.condition:
      assert self.condition.wait_for(lambda: len(uploads) >= count, timeout=1), "Uploader did not process all files"

log_handler = FakeLogHandler()
cloudlog.addHandler(log_handler)


class TestUploader(UploaderTestCase):
  def setup_method(self):
    super().openpilot_setup_method()
    log_handler.reset()
    # StarPilot: rlogs go up while parked or with UploadRlogs on; these upstream tests cover qlog-only uploading
    self.params.put_bool("UploadRlogs", False, block=True)
    self.params.put_bool("IsOffroad", False, block=True)
    try:
      os.removexattr(Paths.log_root(), ANCHOR_ATTR_NAME)
    except OSError:
      pass

  def start_thread(self):
    self.end_event = threading.Event()
    self.up_thread = threading.Thread(target=main, args=[self.end_event])
    self.up_thread.daemon = True
    self.up_thread.start()

  def join_thread(self):
    self.end_event.set()
    self.up_thread.join()

  def gen_files(self, lock=False, xattr: bytes | None = None, boot=True) -> list[Path]:
    f_paths = []
    for t in ["qlog", "rlog", "dcamera.hevc", "fcamera.hevc"]:
      f_paths.append(self.make_file_with_data(self.seg_dir, t, 1, lock=lock, upload_xattr=xattr))

    if boot:
      f_paths.append(self.make_file_with_data("boot", f"{self.seg_dir}", 1, lock=lock, upload_xattr=xattr))
    return f_paths

  def gen_order(self, seg1: list[int], seg2: list[int], boot=True) -> list[str]:
    keys = []
    if boot:
      keys += [f"boot/{self.seg_format.format(i)}.zst" for i in seg1]
      keys += [f"boot/{self.seg_format2.format(i)}.zst" for i in seg2]
    keys += [f"{self.seg_format.format(i)}/qlog.zst" for i in seg1]
    keys += [f"{self.seg_format2.format(i)}/qlog.zst" for i in seg2]
    return keys

  def test_upload(self):
    self.gen_files(lock=False)
    exp_order = self.gen_order([self.seg_num], [])

    self.start_thread()
    log_handler.wait_for_uploads(len(exp_order))
    self.join_thread()

    assert len(log_handler.upload_ignored) == 0, "Some files were ignored"
    assert not len(log_handler.upload_order) < len(exp_order), "Some files failed to upload"
    assert not len(log_handler.upload_order) > len(exp_order), "Some files were uploaded twice"
    for f_path in exp_order:
      assert os.getxattr((Path(Paths.log_root()) / f_path).with_suffix(""), UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE, "All files not uploaded"

    assert log_handler.upload_order == exp_order, "Files uploaded in wrong order"

  def test_upload_with_wrong_xattr(self):
    self.gen_files(lock=False, xattr=b'0')
    exp_order = self.gen_order([self.seg_num], [])

    self.start_thread()
    log_handler.wait_for_uploads(len(exp_order))
    self.join_thread()

    assert len(log_handler.upload_ignored) == 0, "Some files were ignored"
    assert not len(log_handler.upload_order) < len(exp_order), "Some files failed to upload"
    assert not len(log_handler.upload_order) > len(exp_order), "Some files were uploaded twice"
    for f_path in exp_order:
      assert os.getxattr((Path(Paths.log_root()) / f_path).with_suffix(""), UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE, "All files not uploaded"

    assert log_handler.upload_order == exp_order, "Files uploaded in wrong order"

  def test_upload_ignored(self):
    self.set_ignore()
    self.gen_files(lock=False)
    exp_order = self.gen_order([self.seg_num], [])

    self.start_thread()
    log_handler.wait_for_uploads(len(exp_order), ignored=True)
    self.join_thread()

    assert len(log_handler.upload_order) == 0, "Some files were not ignored"
    assert not len(log_handler.upload_ignored) < len(exp_order), "Some files failed to ignore"
    assert not len(log_handler.upload_ignored) > len(exp_order), "Some files were ignored twice"
    for f_path in exp_order:
      assert os.getxattr((Path(Paths.log_root()) / f_path).with_suffix(""), UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE, "All files not ignored"

    assert log_handler.upload_ignored == exp_order, "Files ignored in wrong order"

  def test_upload_files_in_create_order(self):
    seg1_nums = [0, 1, 2, 10, 20]
    for i in seg1_nums:
      self.seg_dir = self.seg_format.format(i)
      self.gen_files(boot=False)
    seg2_nums = [5, 50, 51]
    for i in seg2_nums:
      self.seg_dir = self.seg_format2.format(i)
      self.gen_files(boot=False)

    exp_order = self.gen_order(seg1_nums, seg2_nums, boot=False)

    self.start_thread()
    log_handler.wait_for_uploads(len(exp_order))
    self.join_thread()

    assert len(log_handler.upload_ignored) == 0, "Some files were ignored"
    assert not len(log_handler.upload_order) < len(exp_order), "Some files failed to upload"
    assert not len(log_handler.upload_order) > len(exp_order), "Some files were uploaded twice"
    for f_path in exp_order:
      assert os.getxattr((Path(Paths.log_root()) / f_path).with_suffix(""), UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE, "All files not uploaded"

    assert log_handler.upload_order == exp_order, "Files uploaded in wrong order"

  def test_no_upload_with_lock_file(self):
    f_paths = self.gen_files(lock=True, boot=False)
    uploader = Uploader("0000000000000000", Paths.log_root())

    for f_path in f_paths:
      fn = f_path.with_suffix(f_path.suffix.replace(".zst", ""))
      assert all(candidate[2] != str(fn) for candidate in uploader.list_upload_files(metered=False)), "Locked file selected for upload"

  def test_no_upload_with_xattr(self):
    f_paths = self.gen_files(lock=False, xattr=UPLOAD_ATTR_VALUE)
    uploader = Uploader("0000000000000000", Paths.log_root())
    upload_candidates = {candidate[2] for candidate in uploader.list_upload_files(metered=False)}
    assert upload_candidates.isdisjoint(map(str, f_paths)), "Uploaded file selected again"

  def test_finished_routes_are_not_rescanned(self, mocker):
    self.gen_files(lock=False, xattr=UPLOAD_ATTR_VALUE, boot=False)
    for i in range(3):  # the newest routes are always rescanned, so bury the finished one
      newer = Path(Paths.log_root()) / f"{9000 + i:08x}--aaaaaaaaaa--0"
      newer.mkdir()
    uploader = Uploader("0000000000000000", Paths.log_root())
    assert list(uploader.list_upload_files(metered=False)) == []
    assert self.seg_dir.split("/")[0] in uploader.settled

    listed = []
    real_listdir = os.listdir
    mocker.patch("os.listdir", side_effect=lambda path: listed.append(str(path)) or real_listdir(path))
    assert list(uploader.list_upload_files(metered=False)) == []
    assert not any(self.seg_dir.split("/")[0] in path for path in listed), "Settled route rescanned"

  def test_unsendable_rlogs_do_not_hold_routes_open(self):
    f_paths = self.gen_files(lock=False, boot=False)
    for f_path in f_paths:
      if not f_path.name.startswith("rlog"):
        os.setxattr(f_path, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE)
    assert ensure_anchor(Paths.log_root()) == self.seg_dir.split("/")[0].rpartition("--")[0]  # rlog is in the window
    for i in range(3):  # the newest routes are always rescanned, so bury this one
      (Path(Paths.log_root()) / f"{9000 + i:08x}--aaaaaaaaaa--0").mkdir()
    uploader = Uploader("0000000000000000", Paths.log_root())

    # driving with UploadRlogs off: the rlog cannot be sent, so the route settles
    assert uploader.next_file_to_upload(metered=False) is None
    assert self.seg_dir.split("/")[0] in uploader.settled

    # parked: the route is rescanned and its rlog goes up
    self.params.put_bool("IsOffroad", True, block=True)
    upload = uploader.next_file_to_upload(metered=False)
    assert upload is not None and upload[1].startswith(f"{self.seg_dir}/rlog")

  def test_clear_locks_on_startup(self, mocker):
    f_paths = self.gen_files(lock=True, boot=False)
    locks_cleared = threading.Event()

    def clear_locks_and_signal(root):
      clear_locks(root)
      locks_cleared.set()

    mocker.patch("openpilot.system.loggerd.uploader.clear_locks", side_effect=clear_locks_and_signal)
    self.start_thread()
    assert locks_cleared.wait(timeout=1), "Uploader did not clear locks on startup"
    self.join_thread()

    for f_path in f_paths:
      lock_path = f_path.with_suffix(f_path.suffix + ".lock")
      assert not lock_path.is_file(), "File lock not cleared on startup"

  def test_rlogs_not_uploaded_while_driving_when_off(self):
    self.gen_files(lock=False, boot=False)
    uploader = Uploader("0000000000000000", Paths.log_root())
    keys = []
    while (d := uploader.next_file_to_upload(metered=False)) is not None:
      keys.append(d[1])
      os.setxattr(d[2], UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE)
    assert keys == [f"{self.seg_dir}/qlog"]
    assert get_anchor(Paths.log_root()) is None

  def test_upload_rlogs_after_qlogs(self):
    self.gen_files(lock=False, boot=False)
    self.params.put_bool("UploadRlogs", True, block=True)
    self.start_thread()
    log_handler.wait_for_uploads(2)
    self.join_thread()
    assert log_handler.upload_order == [f"{self.seg_dir}/qlog.zst", f"{self.seg_dir}/rlog.zst"]
    assert not rlogs_pending(Paths.log_root(), UPLOAD_ATTR_NAME)

  def test_rlogs_upload_when_parked_with_toggle_off(self):
    self.gen_files(lock=False, boot=False)
    self.params.put_bool("IsOffroad", True, block=True)
    self.start_thread()
    log_handler.wait_for_uploads(2)
    self.join_thread()
    assert log_handler.upload_order == [f"{self.seg_dir}/qlog.zst", f"{self.seg_dir}/rlog.zst"]
