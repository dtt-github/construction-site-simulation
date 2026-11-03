# Construction site simulation

A physics-based construction site built on NVIDIA's free SimReady OpenUSD assets. Rigid-body site physics runs in the browser (three.js + Jolt); soil digging runs on an NVIDIA DGX Spark (Newton MPM) and streams into the same page.

| Folder | What's in it |
|---|---|
| `feasibility.html` | Team feasibility page (browser vs Omniverse decision, research summary, build plan) |
| `reports/`, `research_notes/` | Full cited research report on soil, machine and crane simulation |
| `phase0/` | Platform tests on the DGX Spark: Newton vs Chrono soil benchmark, Chrome WebGPU, Isaac Sim 6.2 container. See `phase0/RESULTS.md` |
| `phase1/` | SimReady → web pipeline (USD physics extractor, GLB checks, CoACD colliders) and the three.js + Jolt site with terrain and machine motors. See `phase1/RESULTS.md` |
| `phase2/` | Newton soil server (MPM sand + bucket) streaming to the site over WebSocket. See `phase2/RESULTS.md` |

## Quick start (on the DGX Spark)

```bash
# environments (see phase0/RESULTS.md): uv venv phase0/.venv-newton && uv pip install -p phase0/.venv-newton "newton[examples]" playwright
phase1/fetch_assets.sh                                         # SimReady USD + GLBs (not in git)
phase0/.venv-newton/bin/python phase2/soil_server.py &         # soil server, ws://127.0.0.1:8765
python3 -m http.server 8731 --bind 127.0.0.1 --directory phase1
# open http://127.0.0.1:8731/web/index.html in Chrome (on the Spark add --enable-unsafe-webgpu --enable-features=Vulkan)
```

## Credits and licences

- 3D models: [NVIDIA SimReady assets](https://huggingface.co/datasets/nvidia/simready-assets), CC BY 4.0. GLB conversion by [codepawl/simready-glb](https://huggingface.co/datasets/codepawl/simready-glb), CC BY 4.0. Masses and colliders modified in this project. Assets are downloaded by `phase1/fetch_assets.sh`, not stored here.
- Physics: [NVIDIA Newton](https://github.com/newton-physics/newton) and Warp (Apache-2.0), [Jolt Physics](https://github.com/jrouwe/JoltPhysics.js) (MIT), [Project Chrono](https://projectchrono.org/) (BSD-3), [CoACD](https://github.com/SarahWeiii/CoACD).
- Rendering: [three.js](https://threejs.org/) (MIT).
