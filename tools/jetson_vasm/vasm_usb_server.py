#!/usr/bin/env python3
"""Jetson side: answer the comma's V-ASM crops with TensorRT, over USB or TCP.

USB: the comma's Jetlink gadget (1209:0001) carries a second vendor interface
next to Jetlink's interface 0, identified by VASM_IFACE_CLASS. This claims only
that interface. It never sets the configuration or resets the device: either
would drop jetlink-server's interface 0 and the driving model with it.

TCP: the same requests over a socket, for testing from the Mac without the comma:
  python3 vasm_usb_server.py --engine run12_fp16_b1.engine --tcp 0.0.0.0:7461

Needs python3-usb and read/write on the comma's /dev/bus/usb node (udev rule).
"""
import argparse
import errno
import socket
import sys
import time

import numpy as np

import vasm_wire as W
from vasm_trt import VasmTrt
from yield_gate import YieldGate

USB_VID, USB_PID = 0x1209, 0x0001         # the comma's Jetlink gadget (jetlink/protocol.py)
VASM_IFACE_CLASS = (0xFF, 0x56, 0x41)     # vendor, 'V', 'A': distinct from Jetlink's FF/FF/FF
READ_TIMEOUT_MS = 1000
RESCAN_S = 1.0


def log(msg: str):
  print(f"{time.strftime('%H:%M:%S')} vasm: {msg}", flush=True)


class UsbLink:
  IFACE_CLASS = VASM_IFACE_CLASS
  REQ_SIZE = W.REQ_SIZE      # the largest request; one read is one transfer

  def __init__(self):
    import usb.core
    import usb.util
    self.core, self.util = usb.core, usb.util
    self.dev = self.intf = self.ep_in = self.ep_out = None

  def open(self) -> bool:
    dev = self.core.find(idVendor=USB_VID, idProduct=USB_PID)
    if dev is None:
      return False
    try:
      cfg = dev.get_active_configuration()   # no set_configuration: Jetlink owns the device
    except self.core.USBError as e:
      log(f"no active configuration yet: {e}")
      return False
    for intf in cfg:
      if (intf.bInterfaceClass, intf.bInterfaceSubClass, intf.bInterfaceProtocol) == self.IFACE_CLASS:
        break
    else:
      return False
    ep_in = self.util.find_descriptor(intf, custom_match=lambda e: self.util.endpoint_direction(e.bEndpointAddress) == self.util.ENDPOINT_IN)
    ep_out = self.util.find_descriptor(intf, custom_match=lambda e: self.util.endpoint_direction(e.bEndpointAddress) == self.util.ENDPOINT_OUT)
    if ep_in is None or ep_out is None:
      log(f"interface {intf.bInterfaceNumber} has no bulk pair")
      return False
    self.util.claim_interface(dev, intf.bInterfaceNumber)
    self.dev, self.intf, self.ep_in, self.ep_out = dev, intf.bInterfaceNumber, ep_in, ep_out
    log(f"claimed interface {self.intf} (in {ep_in.bEndpointAddress:#x} out {ep_out.bEndpointAddress:#x}, "
        f"max packet {ep_in.wMaxPacketSize})")
    return True

  def recv(self):
    """A whole request, None on timeout. Raises OSError when the device is gone."""
    try:
      return self.ep_in.read(self.REQ_SIZE, timeout=READ_TIMEOUT_MS)
    except self.core.USBTimeoutError:
      return None
    except self.core.USBError as e:
      raise OSError(e.errno or errno.EIO, str(e)) from e

  def send(self, b: bytes):
    try:
      self.ep_out.write(b, timeout=READ_TIMEOUT_MS)
    except self.core.USBError as e:
      raise OSError(e.errno or errno.EIO, str(e)) from e

  def hello(self, b: bytes):
    """Best effort: nobody reading on the comma (no client process) times out here."""
    try:
      self.ep_out.write(b, timeout=100)
    except self.core.USBTimeoutError:
      pass
    except self.core.USBError as e:
      raise OSError(e.errno or errno.EIO, str(e)) from e

  def close(self):
    if self.dev is not None:
      try:
        self.util.release_interface(self.dev, self.intf)
      except self.core.USBError:
        pass
      self.util.dispose_resources(self.dev)
    self.dev = None


class TcpLink:
  def __init__(self, addr: str):
    host, port = addr.rsplit(":", 1)
    self.srv = socket.create_server((host, int(port)), reuse_port=True)
    self.conn = None

  def open(self) -> bool:
    self.srv.settimeout(RESCAN_S)
    try:
      self.conn, peer = self.srv.accept()
    except socket.timeout:
      return False
    self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    log(f"tcp client {peer[0]}")
    return True

  def recv(self):
    buf = bytearray(W.REQ_SIZE)
    view, got = memoryview(buf), 0
    while got < W.REQ_SIZE:
      n = self.conn.recv_into(view[got:])
      if n == 0:
        raise OSError(errno.ECONNRESET, "client closed")
      got += n
    return buf

  def send(self, b: bytes):
    self.conn.sendall(b)

  def hello(self, b: bytes):
    pass  # the TCP test client does not expect hellos

  def close(self):
    if self.conn is not None:
      self.conn.close()
    self.conn = None


def serve(link, model: VasmTrt, hello: bytes, gate=None, yield_hello: bytes = b''):
  n, t_log = 0, time.monotonic()
  while True:
    if not link.open():
      time.sleep(0 if isinstance(link, TcpLink) else RESCAN_S)
      continue
    try:
      while True:
        buf = link.recv()
        if buf is None:
          link.hello(yield_hello if gate is not None and gate.yielding() else hello)
          continue
        t0 = time.perf_counter()
        try:
          seq, side, crop = W.unpack_request(buf)
        except ValueError as e:
          log(f"dropped: {e}")
          link.send(W.pack_response(0, W.STATUS_BAD_REQUEST, 0))
          continue
        if gate is not None and gate.yielding():
          link.send(W.pack_response(seq, W.STATUS_YIELD, 0))
          continue
        try:
          probs, status = model.infer(crop), W.STATUS_OK
        except Exception as e:  # report, keep serving
          log(f"infer failed: {e}")
          probs, status = np.zeros(3, np.float32), W.STATUS_INFER_FAILED
        us = int((time.perf_counter() - t0) * 1e6)
        if gate is not None and status == W.STATUS_OK:
          gate.note(0, us)
        link.send(W.pack_response(seq, status, us, tuple(float(p) for p in probs[:3])))
        n += 1
        if time.monotonic() - t_log > 60:
          log(f"{n} crops served")
          t_log = time.monotonic()
    except OSError as e:
      log(f"link lost: {e}")
      link.close()


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--engine", required=True)
  ap.add_argument("--model-sha256", required=True, help="sha256 of the ONNX the engine was built from")
  ap.add_argument("--tcp", help="host:port; USB when omitted")
  ap.add_argument("--no-yield", action="store_true", help="never give the GPU back to the driving model (bench only)")
  args = ap.parse_args()
  model = VasmTrt(args.engine)
  model.infer(np.zeros(W.CROP_SHAPE, np.uint8))  # warm the engine before the first real crop
  link = TcpLink(args.tcp) if args.tcp else UsbLink()
  log(f"ready on {'tcp ' + args.tcp if args.tcp else 'usb %04x:%04x class %02x/%02x/%02x' % (USB_VID, USB_PID, *VASM_IFACE_CLASS)}")
  try:
    gate = None if args.no_yield else YieldGate(log=log)
    serve(link, model, W.pack_hello(args.model_sha256), gate, W.pack_hello(args.model_sha256, W.STATUS_YIELD))
  except KeyboardInterrupt:
    link.close()
    sys.exit(0)


if __name__ == "__main__":
  main()
