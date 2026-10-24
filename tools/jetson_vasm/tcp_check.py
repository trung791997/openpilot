#!/usr/bin/env python3
"""Drive vasm_usb_server.py over TCP and check its answers against a saved run.

  python tcp_check.py --addr <jetson>:7461 --npy /tmp/jvasm/crops.npy --expect /tmp/jvasm/trt_fp16.npy

Tests the server's framing and serving loop without the comma. Round-trip times
here are Wi-Fi's, not the cable's.
"""
import argparse
import socket
import time

import numpy as np

import vasm_wire as W


def recv_exact(s, n):
  buf = bytearray()
  while len(buf) < n:
    chunk = s.recv(n - len(buf))
    if not chunk:
      raise OSError("server closed")
    buf += chunk
  return bytes(buf)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--addr", required=True)
  ap.add_argument("--npy", required=True)
  ap.add_argument("--expect", required=True)
  args = ap.parse_args()
  host, port = args.addr.rsplit(":", 1)
  crops, expect = np.load(args.npy), np.load(args.expect)

  s = socket.create_connection((host, int(port)))
  s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
  got, rtt, srv = [], [], []
  for i, c in enumerate(crops):
    t0 = time.perf_counter()
    s.sendall(W.pack_request(i, i & 1, c))
    seq, status, server_us, probs = W.unpack_response(recv_exact(s, W.RESP_SIZE))
    rtt.append((time.perf_counter() - t0) * 1e3)
    if seq != i or status != W.STATUS_OK:
      raise SystemExit(f"crop {i}: seq {seq} status {status}")
    got.append(probs)
    srv.append(server_us / 1e3)
  bad = s.sendall(b"\0" * W.REQ_SIZE) or W.unpack_response(recv_exact(s, W.RESP_SIZE))
  s.close()

  got = np.array(got, dtype=np.float32)
  d = np.abs(got - expect).max()
  print(f"n {len(got)}  max |served - saved run| {d:.2e}")
  print(f"server time  p50 {np.percentile(srv, 50):.2f}  p99 {np.percentile(srv, 99):.2f} ms")
  print(f"wifi rtt     p50 {np.percentile(rtt, 50):.1f}  p99 {np.percentile(rtt, 99):.1f} ms")
  print(f"malformed request -> status {bad[1]} (expect {W.STATUS_BAD_REQUEST})")


if __name__ == "__main__":
  main()
