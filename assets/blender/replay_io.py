"""Read the simulator's replay recordings (phase2/record_replay.py, version 2) into numpy, frame by frame.

Mirrors the browser's decoder (phase1/web/soil.js: startReplay, handleFrame, readTail): all poses are in the
viewer's world frame (three.js, Y up), quaternions xyzw; heightmaps are rows along -z from z0.

    hello, frames = load("assets/replay/site_cycle.bin.gz")
    f = decode(frames[k])   # dict: sim_t, links, parts, carriers, dozer, roller, hm, truck_hm, spoil_hm, grains
"""

import gzip
import json
import struct

import numpy as np

MAGIC = {"SOIL": 0x4C494F53, "EXC2": 0x32435845, "EXC3": 0x33435845, "CAR1": 0x31524143, "CARS": 0x53524143, "DOZ1": 0x315A4F44, "ROL1": 0x314C4F52, "ROL2": 0x324C4F52}
HM_HEADER, P_HEADER = 16, 20


def load(path):
    """Header JSON and the list of raw frames (bytes), un-XORed."""
    raw = open(path, "rb").read()
    if raw[:2] == b"\x1f\x8b":
        raw = gzip.decompress(raw)
    magic, version, hl = struct.unpack_from("<III", raw, 0)
    if magic != 0x594C5052:
        raise ValueError("not a replay file")
    hello = json.loads(raw[12 : 12 + hl])
    o = 12 + hl
    (n,) = struct.unpack_from("<I", raw, o)
    o += 4
    frames, prev_h, prev_t = [], None, None
    for _ in range(n):
        if version >= 2:
            lh, lg, lt = struct.unpack_from("<III", raw, o)
            o += 12
            head = np.frombuffer(raw, np.uint8, lh, o).copy()
            tail = np.frombuffer(raw, np.uint8, lt, o + lh + lg).copy()
            if prev_h is not None and len(prev_h) == lh:
                head ^= prev_h
            if prev_t is not None and len(prev_t) == lt:
                tail ^= prev_t
            frames.append(head.tobytes() + raw[o + lh : o + lh + lg] + tail.tobytes())
            prev_h, prev_t = head, tail
            o += lh + lg + lt
        else:
            (ln,) = struct.unpack_from("<I", raw, o)
            frames.append(raw[o + 4 : o + 4 + ln])
            o += 4 + ln
    return hello, frames


def _hm(b, o):
    nx, ny = struct.unpack_from("<HH", b, o)
    x0, z0, cell = struct.unpack_from("<fff", b, o + 4)
    o += HM_HEADER
    h = np.frombuffer(b, np.uint16, nx * ny, o).astype(np.float32) / 1000 - 1 - 2.5
    return {"nx": nx, "ny": ny, "x0": x0, "z0": z0, "cell": cell, "h": h.reshape(ny, nx)}, o + nx * ny * 2


def _f(b, o, n):
    return np.array(struct.unpack_from(f"<{n}f", b, o))


def decode(b):
    magic, frame = struct.unpack_from("<II", b, 0)
    sim_t, step_ms, rtf = struct.unpack_from("<fff", b, 8)
    f = {"frame": frame, "sim_t": sim_t, "kind": "excavator" if magic in (MAGIC["EXC2"], MAGIC["EXC3"]) else "bucket"}
    o = 24
    if f["kind"] == "bucket":
        f["bucket"] = (_f(b, o, 3), _f(b, o + 12, 4))
        o += 52
    else:
        f["links"] = {}
        for name in ("house", "boom", "stick", "bucket"):
            f["links"][name] = (_f(b, o, 3), _f(b, o + 12, 4))
            o += 28
        f["q"] = _f(b, o, 4)
        o += 16 * 3 + 12 + 24
        f["stage"] = b[o]
        o += 1
        o += (4 - o % 4) % 4
        if magic == MAGIC["EXC3"]:
            (n,) = struct.unpack_from("<I", b, o)
            o += 4
            f["parts"] = [(_f(b, o + 28 * k, 3), _f(b, o + 28 * k + 12, 4)) for k in range(n)]
            o += 28 * n + 12
    f["hm"], o = _hm(b, o)
    if f["kind"] == "excavator":
        f["truck_hm"], o = _hm(b, o)
    (count,) = struct.unpack_from("<I", b, o)
    lo = _f(b, o + 4, 3)
    (span,) = struct.unpack_from("<f", b, o + 16)
    o += P_HEADER
    q = np.frombuffer(b, np.uint16, count * 3, o).astype(np.float32).reshape(count, 3)
    f["grains"] = lo + q / 65535 * span
    o += count * 6
    if o + 4 <= len(b) and struct.unpack_from("<I", b, o)[0] in (MAGIC["CAR1"], MAGIC["CARS"]):
        m = struct.unpack_from("<I", b, o)[0]
        o += 4
        n, owner = 1, 0
        if m == MAGIC["CARS"]:
            n, owner = struct.unpack_from("<Ii", b, o)
            o += 8
        f["carriers"] = []
        for _ in range(n):
            g = _f(b, o, 17)
            f["carriers"].append({"base": (g[0:3], g[3:7]), "pin": (g[7:10], g[10:14]), "tip": g[14], "payload": g[15], "stage": struct.unpack_from("<I", b, o + 68)[0]})
            o += 72
        f["load_owner"] = owner
        f["spoil_hm"], o = _hm(b, o)
    if o + 4 <= len(b) and struct.unpack_from("<I", b, o)[0] == MAGIC["DOZ1"]:
        g = _f(b, o + 4, 9)
        f["dozer"] = {"base": (g[0:3], g[3:7]), "lift": g[7], "stage": struct.unpack_from("<I", b, o + 40)[0]}
        o += 44
    f["rollers"] = []
    for tag in ("ROL1", "ROL2"):  # a second roller follows the first in "ROL2", same layout
        if not (o + 4 <= len(b) and struct.unpack_from("<I", b, o)[0] == MAGIC[tag]):
            break
        g = _f(b, o + 4, 7)
        f["rollers"].append({"base": (g[0:3], g[3:7]), "stage": struct.unpack_from("<I", b, o + 32)[0]})
        o += 36
    if f["rollers"]:
        f["roller"] = f["rollers"][0]
    return f
