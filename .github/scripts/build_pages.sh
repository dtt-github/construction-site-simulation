#!/usr/bin/env bash
# Assemble the static browser demo (phase1 site) into _site/ for GitHub Pages.
# Layout mirrors phase1/ so the relative paths in web/*.js keep working:
#   _site/web/        index.html, site.js, showcase.js, soil.js, site.json, hulls/
#   _site/glb/        NVIDIA SimReady props (CC BY 4.0), downloaded here by assets/fetch_assets.py
#   _site/assets/     open machines and site items from git (machinery/, site/) plus licenses, and
#                     replay/: a haul cycle and a dig pass recorded from the GPU server (phase2/record_replay.py)
# The GPU soil server is not part of the demo; the soil panel plays the recorded cycle instead.
set -euo pipefail
cd "$(dirname "$0")/../.."
out="${1:-_site}"

python3 assets/fetch_assets.py   # validates the registry, downloads the fetched (SimReady) GLBs

rm -rf "$out"
mkdir -p "$out/assets"
cp -r phase1/web "$out/web"
cp -r phase1/glb "$out/glb"
cp -r assets/machinery assets/site assets/licenses assets/replay assets/registry.json "$out/assets/"
cp NOTICE LICENSE "$out/"

# every model the site asks for must be present
python3 - "$out" <<'EOF'
import json, pathlib, sys
out = pathlib.Path(sys.argv[1])
site = json.loads((out / "web/site.json").read_text())
need = {f"glb/{p['asset']}.glb" for p in site.get("placements", [])}
need |= {i["file"] for i in site.get("showcase", [])}
need.add("assets/replay/zx200_cycle.bin.gz")  # soil panel replays
need.add("assets/replay/site_cycle.bin.gz")
missing = sorted(n for n in need if not (out / n).is_file())
if missing:
    sys.exit("missing from the Pages build: " + ", ".join(missing))
print(f"pages build ok: {len(need)} models referenced, all present")
EOF

cat > "$out/index.html" <<'EOF'
<!doctype html>
<meta charset="utf-8">
<title>Construction Site Simulation</title>
<meta http-equiv="refresh" content="0; url=web/index.html">
<p><a href="web/index.html">Open the construction site demo</a></p>
EOF
touch "$out/.nojekyll"
du -sh "$out"
