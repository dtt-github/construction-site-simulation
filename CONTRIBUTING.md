# Contributing

This project aims to be an open, physically realistic construction-site simulator that universities and companies build together. It was initiated by [DigitalTwin Technology GmbH](https://www.digitaltwin.technology), which maintains it and is its main contributor, as part of its work on [ConstructionWare](https://www.constructionware.app). Contributions of every kind help: code, 3D assets, measured data, validation studies, and bug reports.

By contributing you agree that your contribution is licensed under the same terms as the part of the project it goes into (see [Licensing](#licensing)), and you agree to follow the [Code of Conduct](CODE_OF_CONDUCT.md).

## What we need most

| Area | Examples | Where |
|---|---|---|
| **Real-world validation data** | Bucket/blade force measurements with soil properties; excavator joint pressures and cycle logs; wheel/track–soil tests; crane load and sway measurements | Open an issue with the [*Validation data*](https://github.com/dtt-github/construction-site-simulation/issues/new?template=validation_data.yml) template |
| **3D assets of real machinery** | Excavators, dump trucks, loaders, dozers, cranes, mixers with separate moving parts, correct dimensions and masses | Follow the [asset policy](assets/README.md), then open a PR or the *Asset submission* issue |
| **Physics models** | Soil models (cohesive soils, moisture, compaction), hydraulics (pumps, valves, cylinders), cable/rope models for cranes, tyre–soil contact | PRs to `phase2/` (server) or new modules |
| **Calibration** | Fitting soil parameters to lab tests (triaxial, direct shear), fitting hydraulic models to machine data | PRs plus a short write-up in the relevant `RESULTS.md` |
| **Web client** | Rendering, operator controls, UX, performance | `phase1/web/` |
| **Benchmarks** | Reproducible scenes with reference results (e.g. a standard bucket pass) | `benchmarks/` (to be created) |

If you're unsure where something fits, open an issue first.

### Validation data

The weakest part of the simulator is validation against real machines. If you have measured data, even partial, please use the [*Validation data* issue form](https://github.com/dtt-github/construction-site-simulation/issues/new?template=validation_data.yml). It asks for the machine, the data type (bucket/blade forces, cylinder pressures, swing torque, cycle times, soil tests), soil properties with test methods, the sensor setup and sample rate, and the licence. CC-BY-4.0 or CC0 let us publish the data with the benchmarks; private sharing with the maintainers is also possible.

## How to contribute code

1. Fork the repository and create a branch from `main`.
2. Keep changes focused. Match the style of the surrounding code; Python follows PEP 8 with long lines allowed, JavaScript uses ES modules with no build step.
3. If you change physics, include measured numbers: a benchmark run, before/after timings, or comparison against data. Results go into the relevant `RESULTS.md`.
4. Sign off your commits (`git commit -s`) to certify the [Developer Certificate of Origin](https://developercertificate.org/): you wrote the change or have the right to submit it under the project license.
5. Open a pull request using the template. Describe what changed, how you tested it, and on which hardware.

### Development setup

See the quick start in the [README](README.md). In short: a Linux machine with an NVIDIA GPU (developed on an NVIDIA DGX Spark, aarch64), Python 3.12 with `newton[examples]`, and Chrome with WebGPU for the browser client. Third-party assets are downloaded with `phase1/fetch_assets.sh` and `assets/fetch_assets.py`; they are not stored in git unless their license allows it and they are small.

## How to contribute 3D assets

Read the [asset policy](assets/README.md). The short version:

- Only licenses that allow redistribution, modification and commercial use: CC0, CC-BY, CC-BY-SA (flagged), MIT, BSD, Apache-2.0, public domain. No NC, ND or "editorial" licenses, and no manufacturer CAD you aren't allowed to redistribute.
- Every asset needs a registry entry in [`assets/registry.json`](assets/registry.json): source, license, author, what you changed, real-world reference (make/model, dimensions) and, for machines, masses and joints.
- Metres, kilograms, Y-up glTF 2.0 (`.glb`). Moving parts as separate nodes with their pivot at the joint.

Manufacturers and dealers: if you can release models of your machines under an open license, that is the single biggest improvement to this project's realism. Open an issue and we'll help with the format.

## Licensing

- **Code**: Apache License 2.0 ([LICENSE](LICENSE)).
- **3D assets and data**: each keeps its own license, recorded per file in [`assets/registry.json`](assets/registry.json) and summarised in [NOTICE](NOTICE). Attribution is mandatory where the license requires it.
- **Documentation and research notes**: Apache-2.0 unless a file says otherwise. Quoted or cited third-party material remains with its owners.

## Reporting bugs and asking questions

Use GitHub issues. For bugs include the hardware (GPU, driver, OS), versions (Newton/Warp, browser), the command you ran and the output.
