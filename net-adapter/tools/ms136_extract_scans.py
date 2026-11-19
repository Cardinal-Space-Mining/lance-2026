#!/usr/bin/env python3
"""Extract MS136 DenseBuffers from recorded bags for codec training / tests,
and (in GUI mode) run the whole table-retraining pipeline.

Output: raw little-endian uint16 array of shape (N, 14 * 360), one row per scan,
in ms136::redux DenseBuffer layout (bit 15 = reflector, bits 0-14 = range mm).

Two kinds of cloud are handled:
  raw      straight from multiscan_driver (has layer/index/range/reflective).
           Reduced exactly like MS136ScanAdapter::serializeMsg, i.e. the
           codec sees what it would see on the robot. Preferred.
  decoded  client-side net-adapter output (x, y, z, reflective). The dense
           index is recovered by inverting ms136::redux::projectPoint(); exact
           for MS136ScanAdapter::deserializeMsg output (each point is checked
           to land on the dense grid, otherwise the bag is rejected).
Where several echoes land in one cell, the first echo is kept (the lowest echo
index, or for clouds without an echo field the shortest range).

Usage (with ROS sourced):
  ms136_extract_scans.py                 # GUI: pick bags, extract, train, export
  ms136_extract_scans.py BAG_DIR_OR_MCAP OUT.bin [--stride K] [--max N] [--topic T]

GUI mode needs tkinter (Ubuntu: sudo apt install python3-tk) and the trainer
binary (build with: ./src/build.sh --tools). It caches extracted scans, so
re-running with the same bags/settings only re-runs training. Settings and the
bag list are remembered between runs.
"""
import argparse
import glob
import json
import os
import queue
import re
import subprocess
import sys
import threading
from pathlib import Path

import numpy as np

# must match multiscan_spec.hpp: ms136::redux::ELEVATION_LUT / AZIMUTH_OFFSET_LUT,
# LR_LAYER_AZIMUTH_STEP_RAD, DENSE_LAYER_IDX_LUT, isHdLayer(), reducePoint()
ELEVATION = np.array([-0.390975, -0.303366, -0.218469, -0.132131, -0.048453, 0.034470,
                      0.117062, 0.219243, 0.294787, 0.375273, 0.458180, 0.544040,
                      0.634508, 0.731123])
AZ_OFFSET = np.array([2.961626, 2.774730, 2.715218, 3.222567, 2.724633, 2.785353,
                      2.958301, 6.099781, 5.912278, 5.841743, 0.103522, 5.817581,
                      5.873127, 6.094876])
AZ_STEP = 0.0174533
NUM_LAYERS, LAYER_WIDTH = 14, 360
DENSE_LAYER_IDX = np.array([0, 1, 2, 3, 4, -1, 5, 6, 7, 8, 9, 10, 11, -1, 12, 13])  # -1: HD layer
# bump when cloud_to_dense() output changes, to invalidate cached extractions
EXTRACT_VERSION = 2
MIN_RANGE_MM = np.float32(200)
MAX_RANGE_MM = np.float32(0x7FFF + 0.5)
TOPIC = '/multiscan/lidar_scan'

PKG_DIR = Path(__file__).resolve().parents[1]           # src/net-adapter
WS_DIR = PKG_DIR.parents[1]                             # colcon workspace root
DEFAULT_TRAINER = WS_DIR / 'build' / 'net_adapter' / 'ms136_codec_train'
DEFAULT_HEADER = PKG_DIR / 'src' / 'util' / 'ms136_codec' / 'ms136_codec_tables.hpp'
DEFAULT_SCANS_DIR = WS_DIR / 'build' / 'ms136_scans'    # cache; wiped by build.sh --clean
SETTINGS_PATH = Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'ms136_codec_gui.json'


class Cancelled(Exception):
    pass


# --- Extraction ---------------------------------------------------------------

# sensor_msgs/PointField datatype -> numpy dtype
_PF_DTYPES = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
              5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}
_RAW_FIELDS = {'layer': 6, 'index': 6, 'range': 7, 'reflective': 7}  # as serializeMsg requires


def _columns(msg):
    a = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    cols = {}
    for f in msg.fields:
        dt = _PF_DTYPES.get(f.datatype)
        if dt is not None:
            n = np.dtype(dt).itemsize
            cols[f.name] = (f.datatype, a[:, f.offset:f.offset + n].copy().view(dt)[:, 0])
    return cols


def cloud_kind(msg):
    types = {f.name: f.datatype for f in msg.fields}
    return 'raw' if all(types.get(k) == t for k, t in _RAW_FIELDS.items()) else 'decoded'


def reduce_points(range_m, reflective):
    """ms136::redux::reducePoint(): float32 mm, out-of-range -> 0 (null cell)."""
    r = range_m.astype(np.float32) * np.float32(1000)
    ok = (r >= MIN_RANGE_MM) & (r < MAX_RANGE_MM)   # NaN fails both, like !(r >= min)
    mm = np.where(ok, r + np.float32(0.5), 0).astype(np.uint16) & 0x7FFF
    refl = (np.trunc(np.nan_to_num(reflective)) != 0) & ok   # static_cast<uint8_t>(float)
    return mm | (refl.astype(np.uint16) << 15)


def _fill(dense_i, vals, *priority):
    """One value per cell: the point with the smallest priority keys wins
    (last key is the primary one, as in np.lexsort)."""
    out = np.zeros(NUM_LAYERS * LAYER_WIDTH, np.uint16)
    order = np.lexsort((*priority, dense_i))
    cells, first = np.unique(dense_i[order], return_index=True)
    out[cells] = vals[order][first]
    return out


def cloud_to_dense(msg):
    cols = _columns(msg)
    col = lambda name: cols[name][1]
    if cloud_kind(msg) == 'raw':
        layer = col('layer').astype(np.int64)
        index = col('index').astype(np.int64)
        if layer.size and (layer.max() >= len(DENSE_LAYER_IDX) or index.max() >= LAYER_WIDTH):
            raise ValueError('raw cloud has out-of-range layer/index values')
        dl = DENSE_LAYER_IDX[layer]
        lr = dl >= 0                                     # HD layers are dropped
        dense_i = dl[lr] * LAYER_WIDTH + index[lr]
        vals = reduce_points(col('range')[lr], col('reflective')[lr])
        # serializeMsg: later points overwrite earlier ones; prefer the first echo
        later_first = -np.arange(dense_i.size)
        echo = col('echo')[lr].astype(np.int64) if 'echo' in cols else np.zeros_like(dense_i)
        return _fill(dense_i, vals, later_first, echo)

    x, y, z, rf = col('x'), col('y'), col('z'), col('reflective')
    rng = np.sqrt(x.astype(np.float64) ** 2 + y.astype(np.float64) ** 2 + z.astype(np.float64) ** 2)
    ok = rng > 0
    x, y, z, rf, rng = x[ok], y[ok], z[ok], rf[ok], rng[ok]

    phi = np.arcsin(np.clip(z / rng, -1, 1))
    layer = np.abs(phi[:, None] - ELEVATION[None]).argmin(1)
    frac = np.mod((np.arctan2(y, x) - AZ_OFFSET[layer]) / AZ_STEP, 2 * np.pi / AZ_STEP)
    idx = np.round(frac)
    if rng.size and (np.abs(phi - ELEVATION[layer]).max() > 1e-4 or np.abs(frac - idx).max() > 0.01):
        raise ValueError('points do not lie on the MS136 dense grid')
    dense_i = layer * LAYER_WIDTH + np.mod(idx, LAYER_WIDTH).astype(np.int64)
    # round to whole mm first: decoded ranges are exact mm up to float error
    vals = reduce_points(np.round(rng * 1000) / 1000, rf)
    return _fill(dense_i, vals, rng)                     # first echo = shortest range


def bag_mcaps(bag):
    """All .mcap files making up a bag (a bag directory or a single .mcap)."""
    bag = Path(bag)
    if bag.is_dir():
        return sorted(bag.glob('*.mcap'))
    return [bag] if bag.suffix == '.mcap' and bag.exists() else []


def bag_name(bag):
    bag = Path(bag)
    return bag.stem if bag.suffix == '.mcap' else bag.name


def count_messages(mcaps, topic):
    """Messages on topic across the bag, or None if the metadata can't be read."""
    import rosbag2_py
    total = 0
    try:
        for m in mcaps:
            meta = rosbag2_py.Info().read_metadata(str(m), 'mcap')
            total += sum(t.message_count for t in meta.topics_with_message_count
                         if t.topic_metadata.name == topic)
    except Exception:
        return None
    return total


def read_data(reader):
    # read_next() is deprecated from Lyrical on; older distros lack read_next_ext()
    ext = getattr(reader, 'read_next_ext', None)
    return (ext() if ext else reader.read_next())[1]


def bag_span(mcaps):
    """(start, end) recording time of the bag in ns, or None."""
    import rosbag2_py
    try:
        metas = [rosbag2_py.Info().read_metadata(str(m), 'mcap') for m in mcaps]
        return (min(md.starting_time.nanoseconds for md in metas),
                max(md.starting_time.nanoseconds + md.duration.nanoseconds for md in metas))
    except Exception:
        return None


def probe_bag(bag, topic=TOPIC):
    """(scan count, problem, info). problem is None if the first scan decodes;
    info is {'kind': 'raw'|'decoded', 'span': (t0, t1) or None} when it does."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    mcaps = bag_mcaps(bag)
    if not mcaps:
        return 0, 'no .mcap files', None
    n = count_messages(mcaps, topic)
    if n == 0:
        return 0, f'no {topic} messages', None
    try:
        for m in mcaps:
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(m), storage_id='mcap'),
                        rosbag2_py.ConverterOptions('', ''))
            reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
            if reader.has_next():
                msg = deserialize_message(read_data(reader), PointCloud2)
                cloud_to_dense(msg)
                return n, None, {'kind': cloud_kind(msg), 'span': bag_span(mcaps)}
        return 0, f'no {topic} messages', None
    except KeyError as e:
        return n, f'cloud has no {e} field', None
    except Exception as e:
        return n, str(e), None


def overlaps(a, b, frac=0.5):
    """True if two (t0, t1) spans share more than frac of the shorter one."""
    if not a or not b:
        return False
    shared = min(a[1], b[1]) - max(a[0], b[0])
    return shared > frac * max(min(a[1] - a[0], b[1] - b[0]), 1)


def extract(bag, out, stride=1, max_scans=0, topic=TOPIC, progress=None, cancel=None):
    """Write bag's scans to out (atomically). Returns (kept, seen)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    mcaps = bag_mcaps(bag)
    if not mcaps:
        raise FileNotFoundError(f'no .mcap in {bag}')
    total = count_messages(mcaps, topic) if progress else None
    if total is not None:
        expect = -(-total // stride)
        total = min(expect, max_scans) if max_scans else expect

    kept, n = [], 0
    for m in mcaps:  # open each file directly: tolerates missing/empty metadata.yaml
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(m), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
        while reader.has_next() and (max_scans == 0 or len(kept) < max_scans):
            if cancel is not None and cancel.is_set():
                raise Cancelled()
            data = read_data(reader)
            if n % stride == 0:
                kept.append(cloud_to_dense(deserialize_message(data, PointCloud2)))
                if progress and len(kept) % 20 == 0:
                    progress(len(kept), total)
            n += 1
        del reader
        if max_scans and len(kept) >= max_scans:
            break

    arr = np.stack(kept) if kept else np.zeros((0, NUM_LAYERS * LAYER_WIDTH), np.uint16)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_name(out.name + '.part')
    arr.astype('<u2').tofile(tmp)
    os.replace(tmp, out)
    return len(kept), n


# --- Pipeline (GUI backend) ---------------------------------------------------

def read_table_version(header):
    try:
        m = re.search(r'TABLE_VERSION\s*=\s*(\d+)', Path(header).read_text())
        return int(m.group(1)) if m else None
    except OSError:
        return None


def assign_cache_paths(bags, scans_dir, stride, max_scans, topic):
    """Cache file per bag. Files keep the bag's name (it is recorded in the
    generated header); settings go in the directory name."""
    tag = re.sub(r'[^A-Za-z0-9]+', '_', topic).strip('_')
    d = Path(scans_dir) / f'{tag}_stride{stride}_max{max_scans}'
    used, out = set(), []
    for b in bags:
        name, i = bag_name(b), 2
        while name in used:
            name, i = f'{bag_name(b)}-{i}', i + 1
        used.add(name)
        out.append(d / f'{name}.bin')
    return out


def cache_stamp(bag):
    return {'extractor': EXTRACT_VERSION, 'source': str(Path(bag).resolve()),
            'mcaps': {m.name: [m.stat().st_size, m.stat().st_mtime_ns] for m in bag_mcaps(bag)}}


def cache_valid(bag, path):
    side = path.with_suffix('.json')
    try:
        return path.exists() and json.loads(side.read_text()) == cache_stamp(bag)
    except (OSError, ValueError):
        return False


def run_pipeline(cfg, log, progress, cancel, procs):
    """cfg: dict from the GUI. log(str), progress(frac or None). Raises on failure."""
    bags = [b for b, role in cfg['bags'] if role in ('train', 'eval')]
    roles = {b: role for b, role in cfg['bags']}
    paths = assign_cache_paths(bags, cfg['scans_dir'], cfg['stride'], cfg['max'], cfg['topic'])

    # 1. extract (or reuse cached) scans
    failed = set()
    log(f'== Extracting {len(bags)} bag(s) -> {paths[0].parent if paths else "-"}')
    for i, (bag, path) in enumerate(zip(bags, paths)):
        if not cfg['force'] and cache_valid(bag, path):
            log(f'[{i + 1}/{len(bags)}] {bag_name(bag)}: cached ({path.stat().st_size // (2 * NUM_LAYERS * LAYER_WIDTH)} scans)')
            continue
        log(f'[{i + 1}/{len(bags)}] {bag_name(bag)}: extracting...')
        def prog(k, total, i=i):
            progress((i + (min(k / total, 1) if total else 0)) / len(bags))
        try:
            kept, seen = extract(bag, path, cfg['stride'], cfg['max'], cfg['topic'], prog, cancel)
        except (ValueError, KeyError) as e:
            why = f'cloud has no {e} field' if isinstance(e, KeyError) else e
            log(f'    WARNING: skipping bag: {why}')
            failed.add(bag)
            continue
        if kept == 0:
            log(f'    WARNING: skipping bag: no {cfg["topic"]} messages')
            failed.add(bag)
            continue
        path.with_suffix('.json').write_text(json.dumps(cache_stamp(bag)))
        log(f'    kept {kept} of {seen} scans')
    progress(1.0)
    if failed:
        log(f'{len(failed)} bag(s) skipped: ' + ', '.join(bag_name(b) for b in failed))

    # 2. train
    train = [str(p) for b, p in zip(bags, paths) if roles[b] == 'train' and b not in failed]
    evals = [str(p) for b, p in zip(bags, paths) if roles[b] == 'eval' and b not in failed]
    if not train:
        raise RuntimeError('no usable training bags')
    cmd = [cfg['trainer']]
    if cfg['write_header']:
        cmd += ['-o', cfg['header'], '--version', str(cfg['version'])]
    cmd += ['--train', *train] + (['--eval', *evals] if evals else [])
    log('== Training' + ('' if cfg['write_header'] else ' (evaluate only, header not written)'))
    log(f'{len(train)} train / {len(evals)} eval file(s) in {paths[0].parent}')
    run_cmd(cmd, log, cancel, procs, shown=[os.path.basename(c) if c.endswith('.bin') else c for c in cmd])

    # 3. rebuild + test
    if cfg['write_header'] and cfg['rebuild']:
        log('== Rebuilding net_adapter')
        progress(None)
        run_cmd(['colcon', 'build', '--symlink-install', '--packages-select', 'net_adapter',
                 '--event-handlers', 'console_direct+'], log, cancel, procs, cwd=WS_DIR)
        log('== Running test_ms136_codec')
        run_cmd(['colcon', 'test', '--packages-select', 'net_adapter',
                 '--event-handlers', 'console_direct+', '--ctest-args', '-R', 'test_ms136_codec'],
                log, cancel, procs, cwd=WS_DIR)
        run_cmd(['colcon', 'test-result', '--verbose', '--test-result-base',
                 str(WS_DIR / 'build' / 'net_adapter')], log, cancel, procs, cwd=WS_DIR)
        progress(1.0)


def run_cmd(cmd, log, cancel, procs, cwd=None, shown=None):
    log('$ ' + ' '.join(shown or cmd))
    p = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, bufsize=1)
    procs.append(p)
    try:
        for line in p.stdout:
            log(line.rstrip('\n'))
        p.wait()
    finally:
        procs.remove(p)
    if cancel.is_set():
        raise Cancelled()
    if p.returncode != 0:
        raise RuntimeError(f'{os.path.basename(cmd[0])} {cmd[1] if len(cmd) > 1 else ""} '
                           f'exited with code {p.returncode}')


# --- GUI ----------------------------------------------------------------------

def run_gui():
    try:
        import tkinter as tk
        from tkinter import ttk, filedialog, messagebox
        from tkinter.scrolledtext import ScrolledText
    except ImportError:
        sys.exit('GUI mode needs tkinter: sudo apt install python3-tk\n'
                 '(or pass BAG OUT.bin for command-line extraction; see --help)')

    root = tk.Tk()
    root.title('MS136 codec table trainer')
    root.geometry('980x760')

    saved = {}
    try:
        saved = json.loads(SETTINGS_PATH.read_text())
    except (OSError, ValueError):
        pass

    cur_version = read_table_version(saved.get('header', DEFAULT_HEADER))
    v = {
        'topic': tk.StringVar(value=saved.get('topic', TOPIC)),
        'stride': tk.IntVar(value=saved.get('stride', 5)),
        'max': tk.IntVar(value=saved.get('max', 0)),
        'scans_dir': tk.StringVar(value=saved.get('scans_dir', str(DEFAULT_SCANS_DIR))),
        'force': tk.BooleanVar(value=False),
        'trainer': tk.StringVar(value=saved.get('trainer', str(DEFAULT_TRAINER))),
        'header': tk.StringVar(value=saved.get('header', str(DEFAULT_HEADER))),
        'write_header': tk.BooleanVar(value=saved.get('write_header', True)),
        'version': tk.IntVar(value=(cur_version or 0) + 1),
        'rebuild': tk.BooleanVar(value=saved.get('rebuild', True)),
    }
    events = queue.Queue()
    cancel = threading.Event()
    procs = []
    state = {'worker': None, 'last_dir': saved.get('last_dir', str(Path.home()))}

    # bags ---------------------------------------------------------------------
    bf = ttk.LabelFrame(root, text='Bags  (double-click a row to cycle train / eval / skip)')
    bf.pack(fill='both', expand=True, padx=8, pady=(8, 4))
    tree = ttk.Treeview(bf, columns=('role', 'scans', 'status', 'name', 'path'), show='headings',
                        selectmode='extended', height=10)
    for c, w, label in (('role', 55, 'Role'), ('scans', 65, 'Scans'), ('status', 200, 'Status'),
                        ('name', 250, 'Bag'), ('path', 330, 'Path')):
        tree.heading(c, text=label)
        tree.column(c, width=w, stretch=(c == 'path'))
    sb = ttk.Scrollbar(bf, orient='vertical', command=tree.yview)
    tree.configure(yscrollcommand=sb.set)
    tree.tag_configure('eval', foreground='#1f5fbf')
    tree.tag_configure('skip', foreground='#999999')
    btns = ttk.Frame(bf)
    btns.pack(side='right', fill='y', padx=4, pady=4)
    sb.pack(side='right', fill='y')
    tree.pack(side='left', fill='both', expand=True, padx=(4, 0), pady=4)

    # Bags are probed (scan count + first-scan decode check) on a background
    # thread; results come back through `events`.
    # Bags recorded over the same time (a raw bag and its client-side decoded
    # twin, or a bag and its split parts) hold the same scans, so only the best
    # of each overlapping set is kept: raw over decoded, then the larger one.
    probe_q = queue.Queue()
    pending_drop = set()   # paths added by folder scan: removed if they have no scans
    probed = {}            # path -> (kind, span, n) for bags that decode
    auto_skip = set()      # paths the GUI (not the user) set to skip

    def prober():
        while True:
            path, topic = probe_q.get()
            try:
                n, problem, info = probe_bag(path, topic)
            except Exception as e:
                n, problem, info = 0, str(e), None
            events.put(('probe', (path, topic, n, problem, info)))

    threading.Thread(target=prober, daemon=True).start()

    def find(path):
        return next((i for i in tree.get_children() if tree.set(i, 'path') == path), None)

    def add_bag(path, role='train', drop_if_empty=False):
        path = str(Path(path).resolve())
        if find(path):
            return
        tree.insert('', 'end', values=(role, '...', 'checking', bag_name(path), path), tags=(role,))
        if drop_if_empty:
            pending_drop.add(path)
        probe_q.put((path, v['topic'].get()))

    def auto_skip_row(i, why):
        if tree.set(i, 'role') != 'skip':
            set_role([i], 'skip')
            auto_skip.add(tree.set(i, 'path'))
        tree.set(i, 'status', why)

    def on_probe(path, topic, n, problem, info):
        i = find(path)
        if i is None or topic != v['topic'].get():
            return
        drop = path in pending_drop
        pending_drop.discard(path)
        if drop and n == 0:
            tree.delete(i)
            return
        tree.set(i, 'scans', n)
        if path in auto_skip:          # re-probed: let it compete again
            auto_skip.discard(path)
            set_role([i], 'train')
        if problem:
            probed.pop(path, None)
            auto_skip_row(i, problem)
            return
        probed[path] = (info['kind'], info['span'], n)
        tree.set(i, 'status', f'ok ({info["kind"]})')
        rank = lambda p: (probed[p][0] == 'raw', probed[p][2])
        for other in list(probed):
            j = find(other)
            if other == path or j is None or not overlaps(probed[path][1], probed[other][1]):
                continue
            win, lose = (path, j) if rank(path) > rank(other) else (other, i)
            auto_skip_row(lose, f'same run as {bag_name(win)}')

    def recheck():
        probed.clear()
        for i in tree.get_children():
            tree.set(i, 'scans', '...')
            tree.set(i, 'status', 'checking')
            probe_q.put((tree.set(i, 'path'), v['topic'].get()))

    def set_role(items, role, manual=False):
        for i in items:
            tree.set(i, 'role', role)
            tree.item(i, tags=(role,))
            if manual:
                auto_skip.discard(tree.set(i, 'path'))

    def add_folder():
        d = filedialog.askdirectory(title='Bag folder, or a folder containing bags', mustexist=True,
                                    initialdir=state['last_dir'])
        if not d:
            return
        state['last_dir'] = d
        if bag_mcaps(d):
            add_bag(d)
            return
        found = sorted({os.path.dirname(m) for m in glob.glob(os.path.join(glob.escape(d), '**', '*.mcap'),
                                                               recursive=True)})
        if not found:
            messagebox.showwarning('No bags', f'No .mcap files found under\n{d}')
        for f in found:
            add_bag(f, drop_if_empty=True)

    def add_files():
        files = filedialog.askopenfilenames(title='MCAP files', initialdir=state['last_dir'],
                                            filetypes=[('MCAP', '*.mcap'), ('All', '*')])
        for f in files:
            add_bag(f)
        if files:
            state['last_dir'] = os.path.dirname(files[0])

    def cycle(_evt):
        i = tree.focus()
        if i:
            order = ['train', 'eval', 'skip']
            set_role([i], order[(order.index(tree.set(i, 'role')) + 1) % 3], manual=True)

    tree.bind('<Double-1>', cycle)
    tree.bind('<Delete>', lambda e: tree.delete(*tree.selection()))
    for text, cmd in (('Add folder...', add_folder), ('Add .mcap files...', add_files), (None, None),
                      ('Set train', lambda: set_role(tree.selection(), 'train', manual=True)),
                      ('Set eval', lambda: set_role(tree.selection(), 'eval', manual=True)),
                      ('Set skip', lambda: set_role(tree.selection(), 'skip', manual=True)), (None, None),
                      ('Re-check', recheck),
                      ('Remove', lambda: tree.delete(*tree.selection())),
                      ('Clear', lambda: tree.delete(*tree.get_children()))):
        if text is None:
            ttk.Separator(btns).pack(fill='x', pady=4)
        else:
            ttk.Button(btns, text=text, command=cmd, width=16).pack(fill='x', pady=1)
    for b, role in saved.get('bags', []):   # unmounted ones show up as 'no .mcap files'
        add_bag(b, role)

    # settings -----------------------------------------------------------------
    sf = ttk.LabelFrame(root, text='Settings')
    sf.pack(fill='x', padx=8, pady=4)
    sf.columnconfigure(1, weight=1)

    def browse(var, kind):
        if kind == 'dir':
            p = filedialog.askdirectory(initialdir=var.get())
        elif kind == 'save':
            p = filedialog.asksaveasfilename(initialfile=os.path.basename(var.get()),
                                             initialdir=os.path.dirname(var.get()))
        else:
            p = filedialog.askopenfilename(initialdir=os.path.dirname(var.get()))
        if p:
            var.set(p)

    r = 0
    def row(label, widget, browse_kind=None, var=None, note=None):
        nonlocal r
        ttk.Label(sf, text=label).grid(row=r, column=0, sticky='w', padx=6, pady=2)
        widget.grid(row=r, column=1, sticky='we', padx=4, pady=2)
        if browse_kind:
            ttk.Button(sf, text='...', width=3, command=lambda: browse(var, browse_kind)).grid(row=r, column=2, padx=4)
        if note is not None:
            note.grid(row=r, column=3, sticky='w', padx=6)
        r += 1

    nums = ttk.Frame(sf)
    ttk.Label(nums, text='keep every').pack(side='left')
    ttk.Spinbox(nums, from_=1, to=1000, width=5, textvariable=v['stride']).pack(side='left', padx=4)
    ttk.Label(nums, text='th scan,   max per bag').pack(side='left')
    ttk.Spinbox(nums, from_=0, to=1000000, width=8, textvariable=v['max']).pack(side='left', padx=4)
    ttk.Label(nums, text='(0 = no limit)').pack(side='left')
    ttk.Checkbutton(nums, text='Re-extract even if cached', variable=v['force']).pack(side='left', padx=16)

    trainer_note = ttk.Label(sf, foreground='#b00020')
    version_note = ttk.Label(sf)
    row('Scan topic', ttk.Entry(sf, textvariable=v['topic']))
    row('Sampling', nums)
    row('Scan cache dir', ttk.Entry(sf, textvariable=v['scans_dir']), 'dir', v['scans_dir'])
    row('Trainer binary', ttk.Entry(sf, textvariable=v['trainer']), 'file', v['trainer'], trainer_note)
    out_row = ttk.Frame(sf)
    ttk.Checkbutton(out_row, text='Write header', variable=v['write_header']).pack(side='left')
    ttk.Entry(out_row, textvariable=v['header']).pack(side='left', fill='x', expand=True, padx=4)
    row('Output tables', out_row, 'save', v['header'])
    ver_row = ttk.Frame(sf)
    ttk.Spinbox(ver_row, from_=1, to=65535, width=6, textvariable=v['version']).pack(side='left')
    ttk.Checkbutton(ver_row, text='Rebuild net_adapter and run test_ms136_codec afterward',
                    variable=v['rebuild']).pack(side='left', padx=16)
    row('TABLE_VERSION', ver_row, note=version_note)

    def refresh_notes(*_):
        t = v['trainer'].get()
        trainer_note.config(text='' if os.access(t, os.X_OK) else 'not built: ./src/build.sh --tools')
        cv = read_table_version(v['header'].get())
        version_note.config(text=f'current: {cv}' if cv is not None else 'current: (new file)')
    v['trainer'].trace_add('write', refresh_notes)
    v['header'].trace_add('write', refresh_notes)
    refresh_notes()
    topic_timer = [None]
    def topic_changed(*_):   # re-probe once typing pauses
        if topic_timer[0]:
            root.after_cancel(topic_timer[0])
        topic_timer[0] = root.after(800, recheck)
    v['topic'].trace_add('write', topic_changed)

    # run / log ----------------------------------------------------------------
    rf = ttk.Frame(root)
    rf.pack(fill='x', padx=8, pady=4)
    run_btn = ttk.Button(rf, text='Run')
    stop_btn = ttk.Button(rf, text='Cancel', state='disabled')
    run_btn.pack(side='left')
    stop_btn.pack(side='left', padx=4)
    pbar = ttk.Progressbar(rf, maximum=1.0)
    pbar.pack(side='left', fill='x', expand=True, padx=8)
    status = ttk.Label(rf, text='idle', width=24)
    status.pack(side='left')
    logbox = ScrolledText(root, height=14, font=('monospace', 9), state='disabled')
    logbox.pack(fill='both', expand=True, padx=8, pady=(0, 8))

    def collect():
        bags = [(tree.set(i, 'path'), tree.set(i, 'role')) for i in tree.get_children()]
        cfg = {k: var.get() for k, var in v.items()}
        cfg['bags'] = bags
        return cfg

    def save_settings(cfg):
        keep = {k: cfg[k] for k in ('topic', 'stride', 'max', 'scans_dir', 'trainer',
                                    'header', 'write_header', 'rebuild', 'bags')}
        keep['last_dir'] = state['last_dir']
        try:
            SETTINGS_PATH.parent.mkdir(parents=True, exist_ok=True)
            SETTINGS_PATH.write_text(json.dumps(keep, indent=1))
        except OSError:
            pass

    def start():
        try:
            cfg = collect()
        except tk.TclError:
            messagebox.showerror('Invalid setting', 'Stride, max and version must be integers.')
            return
        save_settings(cfg)
        if not any(role == 'train' for _, role in cfg['bags']):
            messagebox.showerror('No training bags', 'Mark at least one bag as train.')
            return
        if cfg['stride'] < 1 or cfg['max'] < 0:
            messagebox.showerror('Invalid setting', 'Stride must be >= 1 and max >= 0.')
            return
        if not os.access(cfg['trainer'], os.X_OK):
            messagebox.showerror('Trainer not found', f'{cfg["trainer"]}\n\nBuild it with:  ./src/build.sh --tools')
            return
        cv = read_table_version(cfg['header'])
        if cfg['write_header'] and cv is not None and cfg['version'] <= cv and not messagebox.askyesno(
                'TABLE_VERSION not bumped',
                f'The header is at TABLE_VERSION {cv}. Writing new tables as version {cfg["version"]} '
                'means peers running the old tables will not detect the mismatch.\n\nContinue anyway?'):
            return
        logbox.config(state='normal')
        logbox.delete('1.0', 'end')
        logbox.config(state='disabled')
        cancel.clear()
        run_btn.config(state='disabled')
        stop_btn.config(state='normal')
        status.config(text='running...')

        def work():
            try:
                run_pipeline(cfg, lambda s: events.put(('log', s)),
                             lambda f: events.put(('progress', f)), cancel, procs)
                events.put(('done', 'finished'))
            except Cancelled:
                events.put(('done', 'cancelled'))
            except Exception as e:
                events.put(('log', f'ERROR: {e}'))
                events.put(('done', 'failed'))
        state['worker'] = threading.Thread(target=work, daemon=True)
        state['worker'].start()

    def stop():
        cancel.set()
        for p in list(procs):
            p.terminate()
        status.config(text='cancelling...')

    run_btn.config(command=start)
    stop_btn.config(command=stop)

    def pump():
        try:
            while True:
                kind, val = events.get_nowait()
                if kind == 'log':
                    logbox.config(state='normal')
                    logbox.insert('end', val + '\n')
                    logbox.see('end')
                    logbox.config(state='disabled')
                elif kind == 'probe':
                    on_probe(*val)
                elif kind == 'progress':
                    if val is None:
                        pbar.config(mode='indeterminate')
                        pbar.start(15)
                    else:
                        pbar.stop()
                        pbar.config(mode='determinate', value=val)
                elif kind == 'done':
                    pbar.stop()
                    pbar.config(mode='determinate')
                    run_btn.config(state='normal')
                    stop_btn.config(state='disabled')
                    status.config(text=val)
                    refresh_notes()
                    cv = read_table_version(v['header'].get())
                    if val == 'finished' and cv is not None:
                        v['version'].set(cv + 1)
        except queue.Empty:
            pass
        root.after(50, pump)

    def on_close():
        try:
            save_settings(collect())
        except tk.TclError:
            pass
        stop()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    pump()
    root.mainloop()


# --- CLI ----------------------------------------------------------------------

def main():
    if len(sys.argv) == 1 or sys.argv[1:] == ['--gui']:
        run_gui()
        return
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='bag directory or .mcap file')
    ap.add_argument('out', help='output .bin path')
    ap.add_argument('--stride', type=int, default=1, help='keep every K-th scan')
    ap.add_argument('--max', type=int, default=0, help='stop after N kept scans (0 = no limit)')
    ap.add_argument('--topic', default=TOPIC, help=f'PointCloud2 topic (default {TOPIC})')
    args = ap.parse_args()
    if args.stride < 1:
        ap.error('--stride must be >= 1')
    try:
        kept, n = extract(args.bag, args.out, args.stride, args.max, args.topic)
    except (FileNotFoundError, ValueError) as e:
        sys.exit(str(e))
    print(f'{bag_name(args.bag)}: kept {kept} of {n} scans -> {args.out}')


if __name__ == '__main__':
    main()
