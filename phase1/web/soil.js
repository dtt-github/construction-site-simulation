// Phase 2 soil client: shows the Newton MPM sand streamed from the Spark in the site.
// Works with two servers:
//   soil_server.py       ("SOIL" frames) - a kinematic bucket steered by position sliders
//   excavator_server.py  ("EXC1" frames) - a joint-driven excavator arm with hydraulic-style
//                                           actuators, steered with levers like a real machine
// Draws the sand surface from the streamed heightmap, the moving grains as points and the
// bucket / arm, and mirrors the surface into a Jolt heightfield so site props rest on the dug ground.

const HM_HEADER_BYTES = 16;
const P_HEADER_BYTES = 20;
const MAGIC_SOIL = 0x4c494f53; // "SOIL"
const MAGIC_EXC = 0x31435845; // "EXC1"

export function startSoil({ THREE, scene, Jolt, jolt, bodyInterface, layerStatic, panel, view }) {
  const params = new URLSearchParams(location.search);
  const url = params.get("soil") ? `ws://${params.get("soil")}` : `ws://${location.hostname || "127.0.0.1"}:8765`;

  // ------------------------------------------------------------ panel (controls depend on the server kind)
  const box = document.createElement("fieldset");
  box.className = "machine";
  box.innerHTML = `
    <legend>Excavation · Newton soil</legend>
    <div id="soil-status" class="hint">Connecting to ${url}…</div>
    <dl class="stats" id="soil-stats"></dl>
    <div class="buttons">
      <button id="soil-auto">Auto dig cycle</button>
      <button id="soil-reset" class="secondary">Reset soil</button>
      <button id="soil-view" class="secondary">View pit</button>
    </div>
    <div id="soil-controls"></div>`;
  panel.insertBefore(box, panel.querySelector("p.hint"));
  const statusEl = box.querySelector("#soil-status");
  const statsEl = box.querySelector("#soil-stats");
  const controlsEl = box.querySelector("#soil-controls");

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
    kind === "excavator" ? view(22.5, 6.5, 22, 27, -1.2, 12) : view(29.5, 3.5, 17.5, 24, -2.2, 12)
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
  const levers = { boom: 0, stick: 0, bucket: 0 };
  const LEVERS = [
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
      <p class="hint">Keys: W/S boom, A/D stick, Q/E bucket (hold). Levers spring back when released.</p>`;
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
      if (!keyLever(k) || held.has(k) || e.target.tagName === "INPUT" && e.target.type !== "range") return;
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
  const addBox = (group, size, center, mat) => {
    const m = new THREE.Mesh(new THREE.BoxGeometry(...size), mat);
    m.position.set(...center);
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

  // excavator (excavator_server.py): links built from the hello geometry
  const arm = { links: {}, base: null };
  function buildExcavator(geom) {
    for (const name of ["boom", "stick", "bucket"]) {
      const g = new THREE.Group();
      for (const b of geom[name]) addBox(g, b.size, b.center, name === "bucket" ? steel : paint);
      g.visible = false;
      scene.add(g);
      arm.links[name] = g;
    }
    // machine body: tracks, house, cab, counterweight (cosmetic; faces the pit along -x)
    const base = new THREE.Group();
    const [px, py, pz] = geom.base.pin;
    const gy = geom.base.ground_y;
    for (const side of [-1, 1]) addBox(base, [4.6, 0.9, 0.7], [px + 1.9, gy + 0.45, pz + side * 1.2], dark);
    addBox(base, [3.6, 1.1, 2.6], [px + 2.1, gy + 1.55, pz], paint);
    addBox(base, [1.0, 1.0, 2.6], [px + 3.8, gy + 1.6, pz], steel);
    addBox(base, [1.3, 1.5, 1.0], [px + 0.9, gy + 2.85, pz - 0.75], paint);
    addBox(base, [1.32, 0.9, 0.9], [px + 0.9, gy + 2.9, pz - 0.75], glass);
    addBox(base, [0.8, 0.9, 0.8], [px + 0.2, py - 0.2, pz], paint); // boom foot
    scene.add(base);
    arm.base = base;
  }

  function makeSurface(nx, ny, x0, z0, cell) {
    const geo = new THREE.BufferGeometry();
    const pos = new Float32Array(nx * ny * 3);
    const col = new Float32Array(nx * ny * 3);
    const c = new THREE.Color();
    for (let iy = 0; iy < ny; iy++) {
      for (let ix = 0; ix < nx; ix++) {
        const v = iy * nx + ix;
        pos[v * 3] = x0 + ix * cell;
        pos[v * 3 + 2] = z0 - iy * cell;
        c.copy(sandColor).multiplyScalar(0.9 + 0.2 * Math.random());
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

  function readTail(dv, buf, o, f) {
    // heightmap + moving grains, shared by both frame kinds
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
    f.hm = { nx, ny, x0, z0, cell, heights };
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
    const f = { kind: magic === MAGIC_EXC ? "excavator" : "bucket" };
    if (magic !== MAGIC_SOIL && magic !== MAGIC_EXC) {
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
      for (const name of ["boom", "stick", "bucket"]) {
        f.links[name] = { pos: f32(o, 3), quat: f32(o + 12, 4) };
        o += 28;
      }
      f.q = f32(o, 3); o += 12;
      f.qd = f32(o, 3); o += 12;
      f.press = f32(o, 3); o += 12;
      f.force = f32(o, 3); o += 12;
      [f.fill, f.cut, f.fee, f.relief] = f32(o, 4); o += 16;
      f.stage = dv.getUint8(o); o += 1;
      o += (4 - (o % 4)) % 4;
    }
    readTail(dv, buf, o, f);
    latest.frame = f;
    debug.frames++;
    stats.frames++;
    stats.bytes += buf.byteLength;
  }

  function connect() {
    ws = new WebSocket(url);
    ws.binaryType = "arraybuffer";
    ws.onopen = () => (statusEl.textContent = `Connected to ${url}`);
    ws.onmessage = (ev) => {
      if (typeof ev.data === "string") {
        const hello = JSON.parse(ev.data);
        const newKind = hello.kind === "excavator" ? "excavator" : "bucket";
        if (newKind !== kind) {
          kind = newKind;
          if (kind === "excavator") {
            if (!arm.base) buildExcavator(hello.geometry);
            excavatorControls();
          } else bucketControls();
        }
        stages = hello.stages ?? [];
        statusEl.textContent = `Connected · ${kind === "excavator" ? "excavator arm" : "kinematic bucket"} · ${hello.particles.toLocaleString()} particles, ${Math.round(hello.voxel * 100)} cm grid`;
        return;
      }
      try {
        handleFrame(ev.data);
      } catch (err) {
        debug.lastError = String(err);
      }
    };
    ws.onclose = () => {
      statusEl.textContent = `Soil server not reachable (${url}). Start phase2/excavator_server.py (or soil_server.py); retrying…`;
      bucket.visible = false;
      for (const g of Object.values(arm.links)) g.visible = false;
      setTimeout(connect, 2000);
    };
  }
  connect();

  // ------------------------------------------------------------ per render frame
  function update() {
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
      rows.push(["Stall", stalled.length ? `<strong class="stall">${stalled.join(", ")} at relief</strong>` : "–"]);
    } else {
      rows.push(["Moving grains", g.count.toLocaleString()]);
    }
    rows.push(["Soil force on bucket", `${kN(Math.hypot(f.force[0], f.force[2]))} horiz · ${kN(f.force[1])} vert`]);
    rows.push(["McKyes reference", f.cut > 0.01 ? `${kN(f.fee)} at ${(f.cut * 100).toFixed(0)} cm cut` : "–"]);
    rows.push(["Bucket load", `${f.fill.toFixed(0)} kg`]);
    statsEl.innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join("");
    window.__soil = { lastFrame: f, stats: { ...stats }, stalled };
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
