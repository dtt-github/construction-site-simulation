# Asset policy

3D assets are the part of this project most likely to cause legal problems, so every asset is tracked in [`registry.json`](registry.json) with its source and license. Please read this before adding one.

## Allowed licenses

The license must allow **redistribution, modification and commercial use**:

| License | Allowed | Notes |
|---|---|---|
| CC0 / public domain | Yes | Preferred |
| CC-BY 3.0 / 4.0 | Yes | Credit the author in the registry; it is shown in NOTICE and in the app |
| CC-BY-SA 3.0 / 4.0 | Yes, flagged | Modified versions must stay CC-BY-SA; mark `"share_alike": true` |
| MIT, BSD, Apache-2.0 | Yes | Common for robotics description packages (URDF + meshes) |
| CC-BY-NC, CC-BY-ND, any NC or ND variant | **No** | |
| "Royalty-free", "standard", "editorial" store licenses | **No** | They usually forbid redistributing the model file |
| Manufacturer CAD/BIM downloads | **No**, unless the manufacturer grants an open license in writing | Typically licensed for design use only |

If in doubt, open an issue before submitting.

## Technical requirements

- **Format**: glTF 2.0 binary (`.glb`). Source files (`.blend`, `.usd`, `.step`) are welcome alongside.
- **Units and axes**: metres, kilograms, seconds; Y-up (glTF convention); the machine faces −Z or +X, stated in the registry.
- **Scale**: checked against the real machine's published dimensions (length, width, height, track gauge). Record them under `real_world`.
- **Moving parts**: separate nodes, each with its origin at its joint pivot (boom foot pin, stick pin, bucket pin, swing bearing, wheel axles). Record the joints (type, axis, limits) in the registry.
- **Detail**: aim for under ~200k triangles per machine for real-time use. Textures up to 2048 px, PBR metallic-roughness.
- **Physics**: give masses per part (and their source: spec sheet, estimate) and, if you can, simplified collision shapes. The tools in `phase1/` can derive convex colliders.

## Registry entry

Each asset in `registry.json` has:

```json
{
  "id": "excavator_example_20t",
  "name": "20 t hydraulic excavator",
  "category": "excavator",
  "status": "in-repo | fetched | candidate",
  "files": ["assets/machinery/excavator_example_20t.glb"],
  "source_url": "page where the model is published",
  "download_url": "direct download link, if any",
  "license": "CC-BY-4.0",
  "license_url": "https://creativecommons.org/licenses/by/4.0/",
  "share_alike": false,
  "author": "Name (link)",
  "attribution": "\"Model name\" by Author, licensed under CC BY 4.0",
  "modifications": "Converted to GLB, split into parts, rescaled to metres",
  "realism": "low | medium | high",
  "articulated": true,
  "real_world": {"reference": "make/model if any", "length_m": 9.5, "width_m": 2.8, "height_m": 3.0, "operating_mass_kg": 20000},
  "physics": {"mass_kg": null, "mass_source": "", "joints": []},
  "added": "2026-11-03"
}
```

`status`: **in-repo** means the file is committed here; **fetched** means `assets/fetch_assets.py` downloads it; **candidate** means it is redistributable but needs a manual download (e.g. a login), and someone still has to add it.

## Folder layout

```
assets/
  registry.json      every asset, its source and license
  fetch_assets.py    downloads "fetched" assets and checks licenses are recorded
  machinery/         machines (excavators, trucks, loaders, cranes, ...)
  site/              site equipment, people, materials
```
