#!/usr/bin/env bash
# Download the SimReady assets used by the Phase 1 site (not stored in git):
#   usd/<asset>/...      USD layers from nvidia/simready-assets (for extract_physics.py, decompose_colliders.py)
#   glb/<asset>.glb      web GLBs from codepawl/simready-glb (loaded by web/index.html)
# Both datasets are CC BY 4.0: credit NVIDIA (and codepawl for the GLB conversion).
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p usd glb
while read -r asset; do
  [ -z "$asset" ] && continue
  echo "== $asset"
  curl -s "https://huggingface.co/api/datasets/nvidia/simready-assets/tree/main/$asset?recursive=true" |
    python3 -c "import sys, json; [print(x['path']) for x in json.load(sys.stdin) if x['type'] == 'file' and x['path'].endswith(('.usd', '.usda', '.usdc'))]" |
    while read -r f; do
      mkdir -p "usd/$(dirname "$f")"
      [ -f "usd/$f" ] || curl -sL -o "usd/$f" "https://huggingface.co/datasets/nvidia/simready-assets/resolve/main/$f"
    done
  [ -f "glb/$asset.glb" ] || curl -sL -o "glb/$asset.glb" "https://huggingface.co/datasets/codepawl/simready-glb/resolve/main/glb/$asset.glb"
done < data/site_assets.txt
echo "done"
