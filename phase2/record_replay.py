"""Record the autopilot for the static browser demo (GitHub Pages has no GPU server).

Runs excavator_server.py headless and stores the hello message and the stream frames the browser
would have received (machine, cylinder, carrier and dozer poses, cylinder pressures, the pit, truck and
spoil heightmaps, grains), with the grains thinned, gzip-compressed. soil.js plays it back when no server
is connected, `speed` times faster than recorded.

Two recordings:
    cycle   one dig-swing-load pass (the original demo)
    site    a whole haul cycle with two carriers: the first is loaded pass by pass and hauls the load to
            the spoil area while the second takes its place at the excavator; the first dumps, the dozer
            spreads the load, until the dozer parks again (about 5 minutes, played as a time-lapse)

File, version 2: "RPLY", u32 version, u32 hello length, hello JSON (with "replay": {"speed", "what"}),
u32 frame count, then per frame u32 head, grains and tail lengths and the three pieces of the frame as the
WebSocket sends it (head + grains + tail). Head and tail are stored XOR the previous frame's head and tail
when those have the same length (mostly zeros: the heightmaps barely change from frame to frame), which
lets gzip shrink a long recording about tenfold. Version 1 stored u32 length + frame.

    ../phase0/.venv-newton/bin/python record_replay.py site                # assets/replay/site_cycle.bin.gz
    ../phase0/.venv-newton/bin/python record_replay.py cycle --passes 1    # assets/replay/zx200_cycle.bin.gz
"""

import argparse
import gzip
import json
import pathlib
import struct

import numpy as np

import excavator_server as ex

REPLAY_DIR = pathlib.Path(__file__).resolve().parent.parent / "assets" / "replay"
OUT = {"cycle": REPLAY_DIR / "zx200_cycle.bin.gz", "site": REPLAY_DIR / "site_cycle.bin.gz"}


def xor(a, b):
    if b is None or len(a) != len(b):
        return a
    return (np.frombuffer(a, np.uint8) ^ np.frombuffer(b, np.uint8)).tobytes()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", nargs="?", choices=("cycle", "site"), default="site")
    ap.add_argument("--passes", type=int, default=1, help="cycle: full dig-swing-load passes to record")
    ap.add_argument("--fps", type=float, default=None, help="frames per simulated second (cycle 15, site 6)")
    ap.add_argument("--speed", type=float, default=None, help="playback speed (cycle 1, site 4)")
    ap.add_argument("--grains", type=int, default=None, help="grains kept per frame (cycle 3000, site 700)")
    ap.add_argument("--max-s", type=float, default=None)
    ap.add_argument("--soil", default=ex.soils.DEFAULT_SOIL)
    ap.add_argument("--carriers", type=int, default=None, help="site: carriers (default 2)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    site = args.what == "site"
    fps = args.fps or (6.0 if site else 15.0)
    speed = args.speed or (4.0 if site else 1.0)
    n_grains = args.grains or (700 if site else 3000)
    max_s = args.max_s or (420.0 if site else 90.0)

    sim = ex.ExcavatorSim(soil=args.soil, dozer=site, haul=site, carriers=(args.carriers or 2) if site else 1)
    sim.auto = True
    every = max(1, round(1 / (fps * sim.frame_dt)))
    frames, n, spread = [], 0, False
    prev = (None, None)
    while sim.sim_time < max_s:
        sim.step()
        n += 1
        if site:
            spread |= sim.dozer.stage == "push"
            if spread and sim.dozer.parked:
                break  # the first load is spread and the dozer is back at its park
        elif sim.passes >= args.passes:
            break
        if n % every:
            continue
        moving = sim.measure()
        if len(moving) > n_grains:
            moving = moving[np.linspace(0, len(moving) - 1, n_grains).astype(int)]
        head, grains, tail = sim.frame_bytes(len(frames), moving, parts=True)
        frames.append(struct.pack("<III", len(head), len(grains), len(tail)) + xor(head, prev[0]) + grains + xor(tail, prev[1]))
        prev = (head, tail)
    hello = sim.hello()
    hello["replay"] = {"speed": speed, "what": args.what, "soil": args.soil, "carriers": len(sim.fleet)}
    hello = json.dumps(hello).encode()
    blob = b"RPLY" + struct.pack("<II", 2, len(hello)) + hello + struct.pack("<I", len(frames)) + b"".join(frames)
    out = pathlib.Path(args.out or OUT[args.what])
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(gzip.compress(blob, compresslevel=9))
    print(json.dumps({
        "file": str(out), "sim_s": round(sim.sim_time, 1), "frames": len(frames), "fps": round(1 / (every * sim.frame_dt), 1), "speed": speed,
        "raw_mb": round(len(blob) / 1e6, 2), "gz_mb": round(out.stat().st_size / 1e6, 2), "delivered_kg": round(sim.fleet.delivered_kg),
    }))


if __name__ == "__main__":
    main()
