#!/usr/bin/env python3
"""Extract MS136 DenseBuffers from recorded bags for codec training / tests.

Output: raw little-endian uint16 array of shape (N, 14 * 360), one row per scan,
in ms136::redux DenseBuffer layout (bit 15 = reflector, bits 0-14 = range mm).

The bags we record contain the *client-side decoded* clouds (x, y, z,
reflective) rather than the raw driver fields, so the dense index and range are
recovered by inverting ms136::redux::projectPoint(). This is exact for clouds
produced by MS136ScanAdapter::deserializeMsg (the script verifies each point
lands on the dense grid and aborts otherwise).

Usage (with ROS sourced):
  ms136_extract_scans.py BAG_DIR_OR_MCAP OUT.bin [--stride K] [--max N]
"""
import argparse
import glob
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

# must match ms136::redux::ELEVATION_LUT / AZIMUTH_OFFSET_LUT / AZIMUTH_STEP_RAD
ELEVATION = np.array([-0.390975, -0.303366, -0.218469, -0.132131, -0.048453, 0.034470,
                      0.117062, 0.219243, 0.294787, 0.375273, 0.458180, 0.544040,
                      0.634508, 0.731123])
AZ_OFFSET = np.array([2.961626, 2.774730, 2.715218, 3.222567, 2.724633, 2.785353,
                      2.958301, 6.099781, 5.912278, 5.841743, 0.103522, 5.817581,
                      5.873127, 6.094876])
AZ_STEP = 0.0174533
NUM_LAYERS, LAYER_WIDTH = 14, 360
TOPIC = '/multiscan/lidar_scan'


def cloud_to_dense(msg):
    fields = {f.name: f.offset for f in msg.fields}
    a = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    col = lambda name: a[:, fields[name]:fields[name] + 4].copy().view(np.float32)[:, 0]
    x, y, z, rf = col('x'), col('y'), col('z'), col('reflective')
    rng = np.sqrt(x.astype(np.float64) ** 2 + y.astype(np.float64) ** 2 + z.astype(np.float64) ** 2)
    ok = rng > 0
    x, y, z, rf, rng = x[ok], y[ok], z[ok], rf[ok], rng[ok]

    phi = np.arcsin(np.clip(z / rng, -1, 1))
    layer = np.abs(phi[:, None] - ELEVATION[None]).argmin(1)
    frac = np.mod((np.arctan2(y, x) - AZ_OFFSET[layer]) / AZ_STEP, 2 * np.pi / AZ_STEP)
    idx = np.round(frac)
    if np.abs(phi - ELEVATION[layer]).max() > 1e-4 or np.abs(frac - idx).max() > 0.01:
        raise ValueError('points do not lie on the MS136 dense grid; '
                         'is this a decoded net-adapter cloud?')
    dense_i = layer * LAYER_WIDTH + np.mod(idx, LAYER_WIDTH).astype(np.int64)

    out = np.zeros(NUM_LAYERS * LAYER_WIDTH, np.uint16)
    mm = np.clip(np.round(rng * 1000), 0, 0x7FFF).astype(np.uint16)
    out[dense_i] = mm | ((rf != 0).astype(np.uint16) << 15)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='bag directory or .mcap file')
    ap.add_argument('out', help='output .bin path')
    ap.add_argument('--stride', type=int, default=1, help='keep every K-th scan')
    ap.add_argument('--max', type=int, default=0, help='stop after N kept scans (0 = no limit)')
    args = ap.parse_args()

    uri = args.bag
    if os.path.isdir(uri):
        mcaps = sorted(glob.glob(os.path.join(uri, '*.mcap')))
        if not mcaps:
            sys.exit(f'no .mcap in {uri}')
        uri = mcaps[0]  # open the file directly: tolerates missing/empty metadata.yaml

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))

    kept, n = [], 0
    while reader.has_next() and (args.max == 0 or len(kept) < args.max):
        _, data, _ = reader.read_next()
        if n % args.stride == 0:
            kept.append(cloud_to_dense(deserialize_message(data, PointCloud2)))
        n += 1

    arr = np.stack(kept) if kept else np.zeros((0, NUM_LAYERS * LAYER_WIDTH), np.uint16)
    arr.astype('<u2').tofile(args.out)
    print(f'{os.path.basename(uri)}: kept {len(kept)} of {n} scans -> {args.out}')


if __name__ == '__main__':
    main()
