"""Compare bounds of override hulls (web/hulls/<asset>.json) with the GLB's own hulls per body."""
import json, struct, sys
import numpy as np

asset = sys.argv[1]
data = open(f"glb/{asset}.glb", "rb").read()
jl = struct.unpack_from("<I", data, 12)[0]
g = json.loads(data[20:20 + jl])
binc = data[20 + jl + 8:]
over = json.load(open(f"web/hulls/{asset}.json"))
for n in g["nodes"]:
    ch = n.get("extras", {}).get("collisionHulls")
    if not ch or n["name"] not in over:
        continue
    o = g["bufferViews"][ch["bufferView"]].get("byteOffset", 0) + ch.get("byteOffset", 0)
    pts = np.frombuffer(binc, dtype="<f4", count=sum(ch["counts"]) * 3, offset=o).reshape(-1, 3)
    op = np.concatenate([np.array(h).reshape(-1, 3) for h in over[n["name"]]])
    print(n["name"][:28].ljust(28), "glb", pts.min(0).round(2), pts.max(0).round(2), "| new", op.min(0).round(2), op.max(0).round(2), len(over[n["name"]]), "hulls")
