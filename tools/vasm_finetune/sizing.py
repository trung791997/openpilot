"""Smaller model input for V-ASM (owner, 2026-10-19: scan faster within the comma's CPU and RAM).

The converted model is trained and exported at 352x352. InputResize downsizes inside the graph, so the exported ONNX still
takes the daemon's 352x352 blob, and training, benchmark and the daemon all see the same resize. Only the PSA attention's
reshape constants depend on the input size (11x11 tokens at 352, stride 32); make_base patches them.
Usage: python sizing.py --base ft/vasm_converted.pt --size 224 --out ft/vasm_converted_224.pt
"""
import argparse

import torch
from torch import nn
import torch.nn.functional as F

STRIDE = 32
SHAPES = {"onnx_initializer_5": lambda n: [-1, 2, 128, n * n], "onnx_initializer_8": lambda n: [-1, 128, n, n],
          "onnx_initializer_9": lambda n: [-1, 128, n, n]}


class InputResize(nn.Module):
  def __init__(self, net, size):
    super().__init__()
    self.net, self.size = net, size

  def forward(self, x):
    return self.net(F.interpolate(x, size=(self.size, self.size), mode="bilinear", align_corners=False))


def make_base(base, size):
  assert size % STRIDE == 0, f"size must be a multiple of {STRIDE}"
  net = torch.load(base, weights_only=False, map_location="cpu")
  for name, shape in SHAPES.items():
    buf = net.initializers.get_buffer(name)
    assert buf.tolist() == shape(352 // STRIDE), f"{name} is {buf.tolist()}, not the 352 shape: different base model"
    net.initializers.register_buffer(name, torch.tensor(shape(size // STRIDE), dtype=buf.dtype))
  return InputResize(net, size).eval()


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--base", required=True)
  ap.add_argument("--size", type=int, required=True)
  ap.add_argument("--out", required=True)
  args = ap.parse_args()
  import sizing  # pickle the class as sizing.InputResize, not __main__.InputResize, so train/soup can load it
  m = sizing.make_base(args.base, args.size)
  with torch.no_grad():
    print("output", m(torch.zeros(1, 3, 352, 352)).shape)
  torch.save(m, args.out)


if __name__ == "__main__":
  main()
