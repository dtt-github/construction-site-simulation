"""Record an auto dig-swing-load cycle for the static browser demo (GitHub Pages has no GPU server).

Runs excavator_server.py headless and stores the hello message and the stream frames the browser
would have received (machine and cylinder poses, cylinder pressures, pit and truck heightmaps, grains),
at 15 Hz with the grains thinned, gzip-compressed. soil.js plays it back when no server is connected.

File: "RPLY", u32 version, u32 hello length, hello JSON, u32 frame count, then per frame u32 length +
the frame bytes exactly as the WebSocket sends them.

    ../phase0/.venv-newton/bin/python record_replay.py                   # one full pass
    ../phase0/.venv-newton/bin/python record_replay.py --passes 2 --fps 12
"""

import argparse
import gzip
import json
import pathlib
import struct

import numpy as np

import excavator_server as ex

OUT = pathlib.Path(__file__).resolve().parent.parent / "assets" / "replay" / "zx200_cycle.bin.gz"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--passes", type=int, default=1, help="full dig-swing-load passes to record")
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--grains", type=int, default=3000, help="grains kept per frame")
    ap.add_argument("--max-s", type=float, default=90.0)
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args()

    sim = ex.ExcavatorSim()
    sim.auto = True
    every = max(1, round(1 / (args.fps * sim.frame_dt)))
    frames, n = [], 0
    while sim.passes < args.passes and sim.sim_time < args.max_s:
        sim.step()
        n += 1
        if n % every:
            continue
        moving = sim.measure()
        if len(moving) > args.grains:
            moving = moving[np.linspace(0, len(moving) - 1, args.grains).astype(int)]
        frames.append(sim.frame_bytes(len(frames), moving))
    hello = json.dumps(sim.hello()).encode()
    blob = b"RPLY" + struct.pack("<II", 1, len(hello)) + hello + struct.pack("<I", len(frames))
    blob += b"".join(struct.pack("<I", len(f)) + f for f in frames)
    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(gzip.compress(blob, compresslevel=9))
    print(json.dumps({
        "file": str(out), "sim_s": round(sim.sim_time, 1), "frames": len(frames), "fps": round(1 / (every * sim.frame_dt), 1),
        "raw_mb": round(len(blob) / 1e6, 2), "gz_mb": round(out.stat().st_size / 1e6, 2), "truck_kg": round(sim.truck_kg),
    }))


if __name__ == "__main__":
    main()
