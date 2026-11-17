"""The vendor extension probe opens the channel, logs the answer and anything that arrives, and sends nothing else."""
from openpilot.starpilot.system.starport import session as S
from openpilot.starpilot.system.starport.wire import field


def _session(monkeypatch, tmp_path, flag=True):
  flag_path = tmp_path / "probe"
  if flag:
    flag_path.write_text("")
  monkeypatch.setattr(S, "VENDOR_EXTENSION_PROBE_FLAG", str(flag_path))
  s = S.ProjectionSession.__new__(S.ProjectionSession)
  s.events, s.sent = [], []
  s.event = lambda name, **v: s.events.append((name, v))
  s.ignored = lambda name, *a, **v: s.events.append((name, v))
  s.send = lambda ch, kind, body=b"", **kw: s.sent.append((ch, kind, body))
  s.vendor_channel, s.vendor_messages = None, 0
  s.audio = s.nav = s.mode = None
  s.sensor_channel = s.bluetooth_channel = None
  s.input_channel = None
  return s


def test_discovery_records_the_vendor_extension():
  descriptor = field(1, 13) + field(S.VENDOR_EXTENSION_SERVICE, field(1, "com.honda.mdve.protocol.VendorExtension"))
  fields = S.parse_fields(descriptor)
  vendor = S.parse_fields(S.one(fields, S.VENDOR_EXTENSION_SERVICE))
  assert S.one(vendor, 1) == b"com.honda.mdve.protocol.VendorExtension"


def test_probe_opens_channel_and_logs_messages(monkeypatch, tmp_path):
  s = _session(monkeypatch, tmp_path)
  s.channels = [{"id": 13, "vendor_extension": {"name": "com.honda.mdve.protocol.VendorExtension"}}]
  s.wait_for = lambda ch, kind, timeout=10.0, patient=False: field(1, 0)
  s._probe_vendor_extension()
  assert s.sent == [(13, S.MSG_CHANNEL_OPEN_REQUEST, field(1, 0) + field(2, 13))]
  assert s.vendor_channel == 13
  assert s.events[-1][0] == "vendor_extension_open_response" and s.events[-1][1]["status"] == 0
  s.handle(13, 0x8001, b"\x01\x02\x03")
  assert s.events[-1][0] == "vendor_extension_message" and s.events[-1][1]["data"] == "010203"
  assert len(s.sent) == 1  # nothing sent on the vendor channel


def test_probe_off_without_flag(monkeypatch, tmp_path):
  s = _session(monkeypatch, tmp_path, flag=False)
  s.channels = [{"id": 13, "vendor_extension": {}}]
  s._probe_vendor_extension()
  assert s.sent == [] and s.events == []


def test_rejected_or_silent_open_is_logged(monkeypatch, tmp_path):
  s = _session(monkeypatch, tmp_path)
  s.channels = [{"id": 13, "vendor_extension": {}}]
  def timeout(*a, **k):
    raise TimeoutError("no answer")
  s.wait_for = timeout
  s._probe_vendor_extension()
  assert s.vendor_channel is None and s.events[-1][0] == "vendor_extension_unavailable"
  s.wait_for = lambda *a, **k: field(1, (1 << 64) - 1)  # status -1
  s._probe_vendor_extension()
  assert s.vendor_channel is None and s.events[-1][1]["status"] == -1


def test_vendor_tunnel_opens_without_probe_flag_and_bridges_hu_feed(monkeypatch, tmp_path):
  class FakeFeed:
    def __init__(self):
      self.attached = 0
      self.received = []
      self.outgoing = [b'{"v":1,"t":"b","port":47312}', b'{"v":1,"t":"p"}']

    def attach_aa(self):
      self.attached += 1

    def receive_aa(self, data):
      self.received.append(data)

    def drain_aa(self, limit=8):
      items = self.outgoing[:limit]
      self.outgoing = self.outgoing[limit:]
      return items

  s = _session(monkeypatch, tmp_path, flag=False)
  s.hu_feed = FakeFeed()
  s.channels = [{"id": 13, "vendor_extension": {"name": "com.honda.mdve.protocol.VendorExtension"}}]
  s.wait_for = lambda ch, kind, timeout=10.0, patient=False: field(1, 0)
  s._probe_vendor_extension()
  assert s.vendor_channel == 13
  assert s.hu_feed.attached == 1

  # Tunnel frames (0x5350 = 'SP') route to hu_feed; stock HUVE frames (0x0400) do not.
  s.handle(13, 0x0400, b"\x1e\x08\x00")
  assert s.hu_feed.received == []
  s.handle(13, S.VENDOR_TUNNEL_KIND, b'{"v":1,"t":"h"}')
  assert s.hu_feed.received == [b'{"v":1,"t":"h"}']

  s.sent.clear()
  s.service_vendor()
  assert s.sent == [
    (13, S.VENDOR_TUNNEL_KIND, b'{"v":1,"t":"b","port":47312}'),
    (13, S.VENDOR_TUNNEL_KIND, b'{"v":1,"t":"p"}'),
  ]
