// Phase 2 soil client: shows the Newton MPM sand streamed from the Spark in the site.
// Works with two servers:
//   soil_server.py       ("SOIL" frames) - a kinematic bucket steered by position sliders
//   excavator_server.py  ("EXC3" frames) - the ZX200 (swing, boom, stick, bucket) driven by its
//                                           hydraulic cylinders, steered with levers, loading a
//                                           dump truck parked beside it
// With no server (e.g. the static GitHub Pages demo) it plays a recorded excavator pass instead.
// Draws the sand surface from the streamed heightmap, the moving grains as points and the
// bucket / arm, and mirrors the surface into a Jolt heightfield so site props rest on the dug ground.

const HM_HEADER_BYTES = 16;
const P_HEADER_BYTES = 20;
const MAGIC_SOIL = 0x4c494f53; // "SOIL"
const MAGIC_EXC = 0x32435845; // "EXC2"
const MAGIC_EXC3 = 0x33435845; // "EXC3": EXC2 + hydraulic cylinder and linkage poses, cylinder pressures

export function startSoil({ THREE, scene, loader, Jolt, jolt, bodyInterface, layerStatic, panel, view }) {
  const params = new URLSearchParams(location.search);
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  const url = params.get("soil") ? `${scheme}://${params.get("soil")}` : `ws://${location.hostname || "127.0.0.1"}:8765`;
  // A static HTTPS host (e.g. the GitHub Pages demo) cannot open ws:// and has no GPU server behind it:
  // stay offline unless ?soil=host:port names a server explicitly.
  const tryConnect = Boolean(params.get("soil")) || location.protocol !== "https:";
  const QUICK_START = "https://github.com/dtt-github/construction-site-simulation#quick-start";
  // one auto dig-swing-load pass recorded from excavator_server.py (phase2/record_replay.py), played
  // back when no server is connected, so the static demo shows the real simulation
  const REPLAY_URL = "../assets/replay/zx200_cycle.bin.gz";

  // ------------------------------------------------------------ panel (controls depend on the server kind)
  const box = document.createElement("fieldset");
  box.className = "machine";
  box.innerHTML = `
    <legend>Excavation · Newton soil</legend>
    <div id="soil-status" class="hint">Connecting to ${url}…</div>
    <dl class="stats" id="soil-stats"></dl>
    <div class="buttons" id="soil-buttons" hidden>
      <button id="soil-auto">Auto dig cycle</button>
      <button id="soil-reset" class="secondary">Reset soil</button>
      <button id="soil-view" class="secondary">View pit</button>
    </div>
    <div id="soil-controls"></div>`;
  panel.insertBefore(box, panel.querySelector("p.hint"));
  const statusEl = box.querySelector("#soil-status");
  const statsEl = box.querySelector("#soil-stats");
  const controlsEl = box.querySelector("#soil-controls");
  const buttonsEl = box.querySelector("#soil-buttons");
  const showOffline = (detail) => {
    if (replay) return; // the recorded cycle is playing
    buttonsEl.hidden = controlsEl.hidden = true;
    statusEl.innerHTML = `<strong>Soil server not connected.</strong> The excavator and soil need the GPU server (NVIDIA Newton); see the <a href="${QUICK_START}" target="_blank" rel="noopener">README quick start</a>. ${detail} <button id="soil-replay" class="secondary">Play recorded dig cycle</button>`;
    statusEl.querySelector("#soil-replay").addEventListener("click", startReplay);
  };

  let ws = null;
  let kind = null; // "bucket" | "excavator"
  const send = (obj) => ws && ws.readyState === 1 && ws.send(JSON.stringify(obj));
  let auto = false;
  const autoBtn = box.querySelector("#soil-auto");
  const setAuto = (on, notify = true) => {
    auto = on;
    autoBtn.textContent = auto ? "Stop auto dig" : "Auto dig cycle";
    if (notify) send({ type: "auto", on: auto });
  };
  autoBtn.addEventListener("click", () => setAuto(!auto));
  box.querySelector("#soil-reset").addEventListener("click", () => send({ type: "reset" }));
  box.querySelector("#soil-view").addEventListener("click", () =>
    kind === "excavator" ? view(17.5, 10, 18.5, 27.5, -0.5, 9.5) : view(29.5, 3.5, 17.5, 24, -2.2, 12)
  );

  function bucketControls() {
    controlsEl.innerHTML = `
      <label for="soil-x">Bucket position <output id="soil-x-out">-2.20 m</output></label>
      <input id="soil-x" type="range" min="-2.6" max="2.6" step="0.05" value="-2.2">
      <label for="soil-z">Bucket height <output id="soil-z-out">1.20 m</output></label>
      <input id="soil-z" type="range" min="0.25" max="2.2" step="0.05" value="1.2">
      <label for="soil-curl">Bucket curl <output id="soil-curl-out">0°</output></label>
      <input id="soil-curl" type="range" min="-110" max="80" step="1" value="0">`;
    const sliders = ["x", "z", "curl"].map((k) => controlsEl.querySelector(`#soil-${k}`));
    const units = { x: " m", z: " m", curl: "°" };
    for (const s of sliders) {
      s.addEventListener("input", () => {
        const k = s.id.slice(5);
        controlsEl.querySelector(`#soil-${k}-out`).textContent = `${Number(s.value).toFixed(k === "curl" ? 0 : 2)}${units[k]}`;
        if (auto) setAuto(false, false);
        send({ type: "target", x: Number(sliders[0].value), z: Number(sliders[1].value), curl: Number(sliders[2].value) });
      });
    }
  }

  // Excavator: three spring-return levers (or keys), relief pressure, cylinder pressure gauges.
  const levers = { swing: 0, boom: 0, stick: 0, bucket: 0 };
  const LEVERS = [
    { key: "swing", label: "Swing", neg: "right", pos: "left", keys: ["arrowright", "arrowleft"] },
    { key: "boom", label: "Boom", neg: "lower", pos: "raise", keys: ["s", "w"] },
    { key: "stick", label: "Stick", neg: "in", pos: "out", keys: ["a", "d"] },
    { key: "bucket", label: "Bucket", neg: "curl", pos: "dump", keys: ["q", "e"] },
  ];
  function sendLevers() {
    if (auto) setAuto(false, false);
    send({ type: "levers", ...levers });
  }
  function excavatorControls() {
    controlsEl.innerHTML =
      LEVERS.map(
        (l) => `
      <label for="lever-${l.key}">${l.label} <output id="lever-${l.key}-out">${l.neg} ◂ 0 ▸ ${l.pos}</output></label>
      <input id="lever-${l.key}" type="range" min="-1" max="1" step="0.05" value="0">
      <div class="gauge" aria-label="${l.label} cylinder pressure"><div id="gauge-${l.key}"></div></div>`
      ).join("") +
      `<label for="relief">Relief pressure <output id="relief-out">100%</output></label>
      <input id="relief" type="range" min="30" max="100" step="5" value="100">
      <p class="hint">Keys: ←/→ swing, W/S boom, A/D stick, Q/E bucket (hold). Levers spring back when released.</p>`;
    for (const l of LEVERS) {
      const s = controlsEl.querySelector(`#lever-${l.key}`);
      const out = controlsEl.querySelector(`#lever-${l.key}-out`);
      const show = () => (out.textContent = `${l.neg} ◂ ${Number(s.value).toFixed(2)} ▸ ${l.pos}`);
      s.addEventListener("input", () => {
        levers[l.key] = Number(s.value);
        show();
        sendLevers();
      });
      const release = () => {
        if (s.value === "0") return;
        s.value = "0";
        levers[l.key] = 0;
        show();
        sendLevers();
      };
      s.addEventListener("pointerup", release);
      s.addEventListener("pointercancel", release);
      l.slider = s;
      l.show = show;
    }
    const relief = controlsEl.querySelector("#relief");
    relief.addEventListener("input", () => {
      controlsEl.querySelector("#relief-out").textContent = `${relief.value}%`;
      send({ type: "relief", value: Number(relief.value) / 100 });
    });
    const held = new Set();
    const keyLever = (key) => LEVERS.find((l) => l.keys.includes(key));
    const applyKeys = () => {
      for (const l of LEVERS) {
        const v = (held.has(l.keys[1]) ? 1 : 0) - (held.has(l.keys[0]) ? 1 : 0);
        if (levers[l.key] !== v) {
          levers[l.key] = v;
          l.slider.value = String(v);
          l.show();
        }
      }
      sendLevers();
    };
    window.addEventListener("keydown", (e) => {
      const k = e.key.toLowerCase();
      if (!keyLever(k) || (e.target.tagName === "INPUT" && e.target.type !== "range")) return;
      e.preventDefault(); // arrow keys would otherwise move the focused slider or scroll the panel
      if (held.has(k)) return;
      held.add(k);
      applyKeys();
    });
    window.addEventListener("keyup", (e) => {
      const k = e.key.toLowerCase();
      if (!held.delete(k)) return;
      applyKeys();
    });
  }

  // ------------------------------------------------------------ scene objects
  const sandColor = new THREE.Color("#c9a774");
  let surface = null; // created on first frame (size comes from the server)
  let hmInfo = null;

  const grains = new THREE.Points(
    new THREE.BufferGeometry(),
    new THREE.PointsMaterial({ color: "#b08a55", size: 0.07, sizeAttenuation: true })
  );
  const grainPos = new Float32Array(40000 * 3);
  grains.geometry.setAttribute("position", new THREE.BufferAttribute(grainPos, 3).setUsage(THREE.DynamicDrawUsage));
  grains.geometry.setDrawRange(0, 0);
  grains.frustumCulled = false;
  scene.add(grains);

  const paint = new THREE.MeshStandardMaterial({ color: "#e0a417", metalness: 0.3, roughness: 0.55 });
  const steel = new THREE.MeshStandardMaterial({ color: "#5b5a56", metalness: 0.6, roughness: 0.5 });
  const dark = new THREE.MeshStandardMaterial({ color: "#2b2b2b", roughness: 0.9 });
  const glass = new THREE.MeshStandardMaterial({ color: "#2f4552", metalness: 0.2, roughness: 0.1, transparent: true, opacity: 0.75 });
  const addBox = (group, size, center, mat, rotZ = 0) => {
    const m = new THREE.Mesh(new THREE.BoxGeometry(...size), mat);
    m.position.set(...center);
    m.rotation.z = rotZ;
    m.castShadow = true;
    m.receiveShadow = true;
    group.add(m);
    return m;
  };

  // kinematic bucket (soil_server.py)
  const bucket = new THREE.Group();
  {
    const L = 0.7, W = 0.9, H = 0.6, T = 0.03;
    addBox(bucket, [T, H, W], [-L / 2, 0, 0], paint);
    addBox(bucket, [L, T, W], [0, -H / 2, 0], paint);
    addBox(bucket, [L, H, T], [0, 0, W / 2], paint);
    addBox(bucket, [L, H, T], [0, 0, -W / 2], paint);
  }
  bucket.visible = false;
  scene.add(bucket);

  // excavator (excavator_server.py): links built from the hello geometry (boxes), replaced by the real
  // machine's meshes when the server names a glTF model whose link frames match its physics links
  const arm = { links: {}, parts: {}, base: null, truckSurface: null, tracks: null, model: false };
  function buildExcavator(geom) {
    for (const name of ["house", "boom", "stick", "bucket"]) {
      const g = new THREE.Group();
      geom[name].forEach((b, i) => addBox(g, b.size, b.center, name === "bucket" ? steel : name === "house" && i === 2 ? glass : paint, b.rot_z ?? 0));
      g.visible = false;
      scene.add(g);
      arm.links[name] = g;
    }
    // undercarriage and dump truck (static, world coordinates)
    const base = new THREE.Group();
    arm.tracks = new THREE.Group();
    geom.tracks.forEach((b) => addBox(arm.tracks, b.size, b.center, dark));
    base.add(arm.tracks);
    if (geom.model && loader) loadMachineModel(geom.model);
    const truckPaint = new THREE.MeshStandardMaterial({ color: "#b5452f", metalness: 0.3, roughness: 0.6 });
    geom.truck_bed.forEach((b) => addBox(base, b.size, b.center, truckPaint));
    geom.truck_body.forEach((b, i) => addBox(base, b.size, b.center, i === 1 ? truckPaint : dark));
    const tire = new THREE.CylinderGeometry(1, 1, 1, 24);
    for (const w of geom.truck_wheels) {
      const m = new THREE.Mesh(tire, dark);
      m.scale.set(w.radius, w.width, w.radius);
      m.rotation.x = Math.PI / 2; // axle across the truck (three z)
      m.position.set(...w.center);
      m.castShadow = true;
      base.add(m);
    }
    scene.add(base);
    arm.base = base;
    arm.capacity = geom.truck_capacity_kg;
  }

  async function loadMachineModel(model) {
    let gltf;
    try {
      gltf = await loader.loadAsync(`../${model.file}`);
    } catch (err) {
      console.warn(`excavator: could not load ${model.file}, drawing boxes`, err);
      return;
    }
    // a link's own meshes (not its child links), in its URDF frame (Z up) -> three.js (Y up)
    const linkMeshes = (nodeName) => {
      const node = gltf.scene.getObjectByName(nodeName);
      if (!node) throw new Error(`node ${nodeName} missing in ${model.file}`);
      const wrap = new THREE.Group();
      wrap.rotation.x = -Math.PI / 2;
      for (const child of node.children) {
        if (child.name.endsWith("_link")) continue;
        wrap.add(child.clone(true));
      }
      wrap.traverse((o) => {
        if (o.isMesh) o.castShadow = o.receiveShadow = true;
      });
      return wrap;
    };
    try {
      const parts = Object.fromEntries(Object.entries(model.links).map(([name, node]) => [name, linkMeshes(node)]));
      const crawler = new THREE.Group();
      crawler.position.set(...model.crawler.pos);
      crawler.quaternion.set(...model.crawler.quat);
      crawler.add(linkMeshes(model.crawler.node));
      for (const [name, wrap] of Object.entries(parts)) {
        const g = arm.links[name];
        g.clear();
        g.add(wrap);
      }
      arm.tracks.clear();
      arm.tracks.add(crawler);
      // hydraulic cylinders (barrel, rod) and bucket linkage plates, placed by pose each frame
      for (const name of model.parts ?? []) {
        const g = new THREE.Group();
        g.add(linkMeshes(name));
        g.visible = false;
        scene.add(g);
        arm.parts[name] = g;
      }
      arm.model = true;
    } catch (err) {
      console.warn("excavator: model has unexpected structure, drawing boxes", err);
    }
  }

  function makeSurface(nx, ny, x0, z0, cell, color = sandColor) {
    const geo = new THREE.BufferGeometry();
    const pos = new Float32Array(nx * ny * 3);
    const col = new Float32Array(nx * ny * 3);
    const c = new THREE.Color();
    for (let iy = 0; iy < ny; iy++) {
      for (let ix = 0; ix < nx; ix++) {
        const v = iy * nx + ix;
        pos[v * 3] = x0 + ix * cell;
        pos[v * 3 + 2] = z0 - iy * cell;
        c.copy(color).multiplyScalar(0.9 + 0.2 * Math.random());
        col.set([c.r, c.g, c.b], v * 3);
      }
    }
    const idx = [];
    for (let iy = 0; iy < ny - 1; iy++) {
      for (let ix = 0; ix < nx - 1; ix++) {
        const a = iy * nx + ix;
        idx.push(a, a + 1, a + nx, a + 1, a + nx + 1, a + nx);
      }
    }
    geo.setIndex(idx);
    geo.setAttribute("position", new THREE.BufferAttribute(pos, 3).setUsage(THREE.DynamicDrawUsage));
    geo.setAttribute("color", new THREE.BufferAttribute(col, 3));
    const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 1, side: THREE.DoubleSide }));
    mesh.receiveShadow = true;
    mesh.castShadow = true;
    scene.add(mesh);
    return mesh;
  }

  // ------------------------------------------------------------ Jolt mirror of the sand surface
  const J = { N: 64, scale: 0.1, hfShape: null, buf: 0, x0: 0, z0: 0, lastUpdate: 0 };
  function makeJoltSurface(info) {
    J.x0 = info.x0 - info.cell / 2 - 0.1;
    J.z0 = info.z0 - (info.ny - 1) * info.cell - info.cell / 2 - 0.1;
    const hs = new Jolt.HeightFieldShapeSettings();
    hs.mSampleCount = J.N;
    hs.mBlockSize = 4;
    hs.mOffset.Set(J.x0, 0, J.z0);
    hs.mScale.Set(J.scale, 1, J.scale);
    hs.mHeightSamples.resize(J.N * J.N);
    const s = new Float32Array(Jolt.HEAPF32.buffer, Jolt.getPointer(hs.mHeightSamples.data()), J.N * J.N);
    s.fill(-2.5);
    const shape = hs.Create().Get();
    J.hfShape = Jolt.castObject(shape, Jolt.HeightFieldShape);
    Jolt.destroy(hs);
    const bcs = new Jolt.BodyCreationSettings(shape, new Jolt.RVec3(0, 0, 0), new Jolt.Quat(0, 0, 0, 1), Jolt.EMotionType_Static, layerStatic);
    bcs.mFriction = 0.75;
    const body = bodyInterface.CreateBody(bcs);
    bodyInterface.AddBody(body.GetID(), Jolt.EActivation_DontActivate);
    Jolt.destroy(bcs);
    J.buf = Jolt._webidl_malloc(J.N * J.N * 4);
  }
  function updateJoltSurface(heights, info) {
    const s = new Float32Array(Jolt.HEAPF32.buffer, J.buf, J.N * J.N);
    for (let sy = 0; sy < J.N; sy++) {
      for (let sx = 0; sx < J.N; sx++) {
        const x = J.x0 + sx * J.scale;
        const z = J.z0 + sy * J.scale;
        const ix = Math.round((x - info.x0) / info.cell);
        const iy = Math.round((info.z0 - z) / info.cell);
        if (ix < 0 || iy < 0 || ix >= info.nx || iy >= info.ny) s[sy * J.N + sx] = Jolt.HeightFieldShapeConstantValues.prototype.cNoCollisionValue;
        else s[sy * J.N + sx] = heights[iy * info.nx + ix];
      }
    }
    J.hfShape.SetHeights(0, 0, J.N, J.N, J.buf, J.N, jolt.GetTempAllocator());
  }

  // ------------------------------------------------------------ frames
  const latest = { frame: null };
  const stats = { frames: 0, bytes: 0, lastT: performance.now(), fps: 0, mbps: 0 };
  const debug = (window.__soilDebug = { messages: 0, frames: 0, badMagic: 0, lastError: null });
  let stages = [];
  let zoneMode = false;

  function readHeightmap(dv, buf, o) {
    const nx = dv.getUint16(o, true);
    const ny = dv.getUint16(o + 2, true);
    const x0 = dv.getFloat32(o + 4, true);
    const z0 = dv.getFloat32(o + 8, true);
    const cell = dv.getFloat32(o + 12, true);
    o += HM_HEADER_BYTES;
    const raw = new Uint16Array(buf.slice(o, o + nx * ny * 2));
    o += nx * ny * 2;
    const heights = new Float32Array(nx * ny);
    for (let k = 0; k < raw.length; k++) heights[k] = raw[k] / 1000 - 1 - 2.5;
    return [{ nx, ny, x0, z0, cell, heights }, o];
  }

  function readTail(dv, buf, o, f) {
    // pit heightmap (+ truck-bed heightmap for the excavator) + moving grains
    [f.hm, o] = readHeightmap(dv, buf, o);
    if (f.kind === "excavator") [f.truckHm, o] = readHeightmap(dv, buf, o);
    const count = dv.getUint32(o, true);
    const lo = [dv.getFloat32(o + 4, true), dv.getFloat32(o + 8, true), dv.getFloat32(o + 12, true)];
    const span = dv.getFloat32(o + 16, true);
    o += P_HEADER_BYTES;
    f.grains = { count, lo, span, q: new Uint16Array(buf.slice(o, o + count * 6)) };
  }

  function handleFrame(buf) {
    debug.messages++;
    const dv = new DataView(buf);
    const magic = dv.getUint32(0, true);
    const f32 = (o, n) => Array.from({ length: n }, (_, k) => dv.getFloat32(o + 4 * k, true));
    const f = { kind: magic === MAGIC_EXC || magic === MAGIC_EXC3 ? "excavator" : "bucket" };
    if (magic !== MAGIC_SOIL && magic !== MAGIC_EXC && magic !== MAGIC_EXC3) {
      debug.badMagic++;
      return;
    }
    f.frame = dv.getUint32(4, true);
    f.simTime = dv.getFloat32(8, true);
    f.stepMs = dv.getFloat32(12, true);
    f.rtf = dv.getFloat32(16, true);
    f.particles = dv.getUint32(20, true);
    let o = 24;
    if (f.kind === "bucket") {
      f.pos = f32(o, 3); o += 12;
      f.quat = f32(o, 4); o += 16;
      f.force = f32(o, 3); o += 12;
      [f.fill, f.cut, f.fee] = f32(o, 3); o += 12;
    } else {
      f.links = {};
      for (const name of ["house", "boom", "stick", "bucket"]) {
        f.links[name] = { pos: f32(o, 3), quat: f32(o + 12, 4) };
        o += 28;
      }
      f.q = f32(o, 4); o += 16;
      f.qd = f32(o, 4); o += 16;
      f.press = f32(o, 4); o += 16;
      f.force = f32(o, 3); o += 12;
      [f.fill, f.cut, f.fee, f.relief, f.truck, f.capacity] = f32(o, 6); o += 24;
      f.stage = dv.getUint8(o); o += 1;
      o += (4 - (o % 4)) % 4;
      if (magic === MAGIC_EXC3) {
        const n = dv.getUint32(o, true); o += 4;
        f.parts = [];
        for (let k = 0; k < n; k++) {
          f.parts.push({ pos: f32(o, 3), quat: f32(o + 12, 4) });
          o += 28;
        }
        f.cylBar = f32(o, 3); o += 12; // + head side, - rod side
      }
    }
    readTail(dv, buf, o, f);
    latest.frame = f;
    debug.frames++;
    stats.frames++;
    stats.bytes += buf.byteLength;
  }

  function onHello(hello) {
    const newKind = hello.kind === "excavator" ? "excavator" : "bucket";
    if (newKind !== kind) {
      kind = newKind;
      if (kind === "excavator") {
        if (!arm.base) buildExcavator(hello.geometry);
        excavatorControls();
      } else bucketControls();
    }
    stages = hello.stages ?? [];
    zoneMode = !!hello.zone;
  }

  // ------------------------------------------------------------ recorded replay (no server)
  let replay = null; // { frames: [{t, buf}], t0, k }
  async function startReplay() {
    if (replay) return;
    statusEl.textContent = "Loading the recorded dig cycle…";
    try {
      const res = await fetch(REPLAY_URL);
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      let buf = await res.arrayBuffer();
      const head = new Uint8Array(buf, 0, 2);
      if (head[0] === 0x1f && head[1] === 0x8b) {
        // served as a plain .gz file: decompress here (a server that set Content-Encoding already did)
        buf = await new Response(new Blob([buf]).stream().pipeThrough(new DecompressionStream("gzip"))).arrayBuffer();
      }
      const dv = new DataView(buf);
      if (dv.getUint32(0, true) !== 0x594c5052) throw new Error("not a replay file"); // "RPLY"
      const helloLen = dv.getUint32(8, true);
      const hello = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 12, helloLen)));
      let o = 12 + helloLen;
      const n = dv.getUint32(o, true);
      o += 4;
      const frames = [];
      for (let k = 0; k < n; k++) {
        const len = dv.getUint32(o, true);
        const frame = buf.slice(o + 4, o + 4 + len);
        frames.push({ t: new DataView(frame).getFloat32(8, true), buf: frame });
        o += 4 + len;
      }
      onHello(hello);
      replay = { frames, t0: performance.now() - frames[0].t * 1000, k: 0 };
      buttonsEl.hidden = controlsEl.hidden = false;
      autoBtn.hidden = box.querySelector("#soil-reset").hidden = true;
      controlsEl.querySelectorAll("input").forEach((el) => (el.disabled = true));
      const secs = frames[frames.length - 1].t.toFixed(0);
      statusEl.innerHTML = `<strong>Recorded replay</strong> (${secs} s, loops): one auto dig–swing–load pass of the ZX200 with its hydraulic cylinders, in Newton MPM sand, recorded from the GPU server. For live control, run the server (<a href="${QUICK_START}" target="_blank" rel="noopener">quick start</a>).`;
    } catch (err) {
      replay = null;
      statusEl.textContent = `Could not load the recorded cycle (${err.message}).`;
    }
  }
  function tickReplay() {
    const r = replay;
    const now = performance.now();
    let t = (now - r.t0) / 1000;
    if (t > r.frames[r.frames.length - 1].t + 1.0) {
      // loop: start over from the untouched pit
      r.t0 = now - r.frames[0].t * 1000;
      r.k = 0;
      t = r.frames[0].t;
    }
    let next = null;
    while (r.k < r.frames.length && r.frames[r.k].t <= t) next = r.frames[r.k++];
    if (next) handleFrame(next.buf);
  }

  let failures = 0; // back off retries so a missing local server doesn't flood the console
  function connect() {
    try {
      ws = new WebSocket(url);
    } catch (err) {
      showOffline(`Could not open ${url} (${err.message}).`);
      return;
    }
    ws.binaryType = "arraybuffer";
    ws.onopen = () => {
      if (replay) location.reload(); // a live server appeared: start clean with live control
      buttonsEl.hidden = controlsEl.hidden = false;
      failures = 0;
      statusEl.textContent = `Connected to ${url}`;
    };
    ws.onmessage = (ev) => {
      if (typeof ev.data === "string") {
        const hello = JSON.parse(ev.data);
        onHello(hello);
        const soilText = hello.zone
          ? `moving soil zone (${hello.particles.toLocaleString()}-particle pool)`
          : `${hello.particles.toLocaleString()} particles`;
        statusEl.textContent = `Connected · ${kind === "excavator" ? "excavator arm" : "kinematic bucket"} · ${soilText}, ${Math.round(hello.voxel * 100)} cm grid`;
        return;
      }
      try {
        handleFrame(ev.data);
      } catch (err) {
        debug.lastError = String(err);
      }
    };
    ws.onclose = () => {
      showOffline(`Start phase2/excavator_server.py (or soil_server.py) for ${url}; retrying…`);
      bucket.visible = false;
      for (const g of Object.values(arm.links)) g.visible = false;
      setTimeout(connect, Math.min(30000, 2000 * 2 ** failures++));
    };
  }
  if (params.has("replay") || !tryConnect) startReplay(); // ?replay forces the recording
  else connect();

  // ------------------------------------------------------------ per render frame
  function update() {
    if (replay) tickReplay();
    const f = latest.frame;
    if (!f) return;
    latest.frame = null;
    const h = f.hm;
    if (!surface) {
      hmInfo = { nx: h.nx, ny: h.ny, x0: h.x0, z0: h.z0, cell: h.cell };
      surface = makeSurface(h.nx, h.ny, h.x0, h.z0, h.cell);
      makeJoltSurface(hmInfo);
    }
    const pos = surface.geometry.attributes.position;
    for (let k = 0; k < h.heights.length; k++) pos.array[k * 3 + 1] = h.heights[k];
    pos.needsUpdate = true;
    surface.geometry.computeVertexNormals();

    const g = f.grains;
    for (let k = 0; k < g.count; k++) {
      grainPos[k * 3] = g.lo[0] + (g.q[k * 3] / 65535) * g.span;
      grainPos[k * 3 + 1] = g.lo[1] + (g.q[k * 3 + 1] / 65535) * g.span;
      grainPos[k * 3 + 2] = g.lo[2] + (g.q[k * 3 + 2] / 65535) * g.span;
    }
    grains.geometry.attributes.position.needsUpdate = true;
    grains.geometry.setDrawRange(0, g.count);

    if (f.truckHm) {
      const t = f.truckHm;
      if (!arm.truckSurface) arm.truckSurface = makeSurface(t.nx, t.ny, t.x0, t.z0, t.cell, new THREE.Color("#b8925f"));
      const tp = arm.truckSurface.geometry.attributes.position;
      for (let k = 0; k < t.heights.length; k++) tp.array[k * 3 + 1] = t.heights[k];
      tp.needsUpdate = true;
      arm.truckSurface.geometry.computeVertexNormals();
    }
    if (f.kind === "bucket") {
      bucket.visible = true;
      bucket.position.set(...f.pos);
      bucket.quaternion.set(...f.quat);
    } else {
      for (const [name, link] of Object.entries(f.links)) {
        const obj = arm.links[name];
        if (!obj) continue;
        obj.visible = true;
        obj.position.set(...link.pos);
        obj.quaternion.set(...link.quat);
      }
      const names = Object.keys(arm.parts);
      (f.parts ?? []).forEach((p, k) => {
        const obj = arm.parts[names[k]];
        if (!obj) return;
        obj.visible = true;
        obj.position.set(...p.pos);
        obj.quaternion.set(...p.quat);
      });
    }

    const now = performance.now();
    if (now - J.lastUpdate > 500) {
      updateJoltSurface(h.heights, hmInfo);
      J.lastUpdate = now;
    }
    if (now - stats.lastT > 1000) {
      stats.fps = (stats.frames * 1000) / (now - stats.lastT);
      stats.mbps = (stats.bytes * 8) / 1e6 / ((now - stats.lastT) / 1000);
      stats.frames = 0;
      stats.bytes = 0;
      stats.lastT = now;
    }
    const kN = (v) => `${(v / 1000).toFixed(1)} kN`;
    const rows = [
      ["Server step", `${f.stepMs.toFixed(1)} ms (${f.rtf.toFixed(2)}× real time)`],
      ["Stream", `${stats.fps.toFixed(0)} frames/s · ${stats.mbps.toFixed(1)} Mbit/s`],
    ];
    if (zoneMode) rows.push(["Active soil", `${f.particles.toLocaleString()} particles near the bucket`]);
    const stalled = [];
    if (f.kind === "excavator") {
      rows.push(["Auto stage", f.stage === 255 ? "manual" : stages[(f.stage + 1) % stages.length] ?? f.stage]);
      LEVERS.forEach((l, k) => {
        const p = f.press[k];
        const el = controlsEl.querySelector(`#gauge-${l.key}`);
        if (el) {
          el.style.width = `${Math.min(p, 1) * 100}%`;
          const atRelief = p >= 0.97 * f.relief;
          el.className = atRelief ? "relief" : p > 0.7 * f.relief ? "high" : "";
          // stalled: at relief pressure, commanded to move, but not moving
          if (atRelief && Math.abs(f.qd[k]) < 0.02 && (Math.abs(levers[l.key]) > 0.1 || f.stage !== 255)) stalled.push(l.label);
        }
      });
      rows.push(["Joints", f.q.map((v) => `${((v * 180) / Math.PI).toFixed(0)}°`).join(" · ")]);
      if (f.cylBar) {
        const side = (v) => `${Math.abs(v).toFixed(0)} bar ${v >= 0 ? "head" : "rod"}`;
        rows.push(["Cylinders", ["boom", "arm", "bucket"].map((n, k) => `${n} ${side(f.cylBar[k])}`).join(" · ")]);
      }
      rows.push(["Stall", stalled.length ? `<strong class="stall">${stalled.join(", ")} at relief</strong>` : "–"]);
    } else {
      rows.push(["Moving grains", g.count.toLocaleString()]);
    }
    rows.push(["Soil force on bucket", `${kN(Math.hypot(f.force[0], f.force[2]))} horiz · ${kN(f.force[1])} vert`]);
    rows.push(["McKyes reference", f.cut > 0.01 ? `${kN(f.fee)} at ${(f.cut * 100).toFixed(0)} cm cut` : "–"]);
    rows.push(["Bucket load", `${f.fill.toFixed(0)} kg`]);
    if (f.kind === "excavator") rows.push(["Truck load", `${(f.truck / 1000).toFixed(2)} t of ${(f.capacity / 1000).toFixed(0)} t (${Math.round((100 * f.truck) / f.capacity)}%)`]);
    statsEl.innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join("");
    window.__soil = { lastFrame: f, stats: { ...stats }, stalled, model: arm.model };
  }

  let reported = false;
  function safeUpdate() {
    try {
      update();
    } catch (err) {
      if (!reported) {
        console.error("soil update failed", err);
        statusEl.textContent = `Soil display error: ${err.message}`;
        reported = true;
      }
    }
  }

  return { update: safeUpdate };
}
