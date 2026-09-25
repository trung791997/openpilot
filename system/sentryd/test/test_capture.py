import numpy as np

import openpilot.system.sentryd.sentryd as sentryd


def test_capture_includes_driver_camera_regardless_of_record_front(mocker, tmp_path):
  params = mocker.MagicMock()
  params.get_bool.side_effect = lambda key: False  # RecordFront off
  mocker.patch.object(sentryd, "event_root", return_value=tmp_path)
  frame = np.zeros((4, 4, 3), dtype=np.uint8)
  snapshot = mocker.patch.object(sentryd, "snapshot", return_value=(frame, frame))

  mode = sentryd.SentryMode.__new__(sentryd.SentryMode)
  mode.params = params
  paths = mode._capture_images("1-abc")

  assert snapshot.call_args.kwargs["include_front"] is True
  assert [p.rsplit("/", 1)[1] for p in paths] == ["wide.jpg", "driver.jpg"]
