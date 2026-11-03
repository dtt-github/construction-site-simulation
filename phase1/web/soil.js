// Phase 2 soil client: shows the Newton MPM sand streamed from soil_server.py (on the Spark) in the site.
// Draws the sand surface from the streamed heightmap, the moving grains as points and the bucket,
// and mirrors the surface into a Jolt heightfield so site props rest on the dug ground.

const HEADER_BYTES = 76;
const HM_HEADER_BYTES = 16;
const P_HEADER_BYTES = 20;

export function startSoil({ THREE, scene, Jolt, jolt, bodyInterface, physicsSystem, layerStatic, panel, view }) {
  const params = new URLSearchParams(location.search);
  const url = params.get("soil") ? `ws://${params.get("soil")}` : `ws://${location.hostname || "127.0.0.1"}:8765`;

  // ------------------------------------------------------------ panel
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
    <label for="soil-x">Bucket position <output id="soil-x-out">-2.20 m</output></label>
    <input id="soil-x" type="range" min="-2.6" max="2.6" step="0.05" value="-2.2">
    <label for="soil-z">Bucket height <output id="soil-z-out">1.20 m</output></label>
    <input id="soil-z" type="range" min="0.25" max="2.2" step="0.05" value="1.2">
    <label for="soil-curl">Bucket curl <output id="soil-curl-out">0°</output></label>
    <input id="soil-curl" type="range" min="-110" max="80" step="1" value="0">`;
  panel.insertBefore(box, panel.querySelector("p.hint"));
  const statusEl = box.querySelector("#soil-status");
  const statsEl = box.querySelector("#soil-stats");

  let ws = null;
  const send = (obj) => ws && ws.readyState === 1 && ws.send(JSON.stringify(obj));
  let auto = false;
  const autoBtn = box.querySelector("#soil-auto");
  autoBtn.addEventListener("click", () => {
    auto = !auto;
    autoBtn.textContent = auto ? "Stop auto dig" : "Auto dig cycle";
    send({ type: "auto", on: auto });
  });
  box.querySelector("#soil-reset").addEventListener("click", () => send({ type: "reset" }));
  box.querySelector("#soil-view").addEventListener("click", () => view(29.5, 3.5, 17.5, 24, -2.2, 12));
  const sliders = ["x", "z", "curl"].map((k) => box.querySelector(`#soil-${k}`));
  const units = { x: " m", z: " m", curl: "°" };
  for (const s of sliders) {
    s.addEventListener("input", () => {
      const k = s.id.slice(5);
      box.querySelector(`#soil-${k}-out`).textContent = `${Number(s.value).toFixed(k === "curl" ? 0 : 2)}${units[k]}`;
      if (auto) {
        auto = false;
        autoBtn.textContent = "Auto dig cycle";
      }
      send({ type: "target", x: Number(sliders[0].value), z: Number(sliders[1].value), curl: Number(sliders[2].value) });
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

  const bucket = new THREE.Group();
  const steel = new THREE.MeshStandardMaterial({ color: "#d9a21b", metalness: 0.4, roughness: 0.55 });
  const L = 0.7, W = 0.9, H = 0.6, T = 0.03;
  for (const [sx, sy, sz, px, py, pz] of [
    [T, H, W, -L / 2, 0, 0], // back (bucket x = zone x, y = up, z = across)
    [L, T, W, 0, -H / 2, 0], // bottom
    [L, H, T, 0, 0, W / 2],
    [L, H, T, 0, 0, -W / 2],
  ]) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), steel);
    m.position.set(px, py, pz);
    m.castShadow = true;
    bucket.add(m);
  }
  bucket.visible = false;
  scene.add(bucket);

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
  const J = { N: 64, scale: 0.1, body: null, shape: null, buf: 0, x0: 0, z0: 0, lastUpdate: 0 };
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
    J.shape = hs.Create().Get();
    J.hfShape = Jolt.castObject(J.shape, Jolt.HeightFieldShape);
    Jolt.destroy(hs);
    const bcs = new Jolt.BodyCreationSettings(J.shape, new Jolt.RVec3(0, 0, 0), new Jolt.Quat(0, 0, 0, 1), Jolt.EMotionType_Static, layerStatic);
    bcs.mFriction = 0.75;
    J.body = bodyInterface.CreateBody(bcs);
    bodyInterface.AddBody(J.body.GetID(), Jolt.EActivation_DontActivate);
    Jolt.destroy(bcs);
    J.buf = Jolt._webidl_malloc(J.N * J.N * 4);
  }
  function updateJoltSurface(heights, info, oy) {
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
  function handleFrame(buf) {
    debug.messages++;
    const dv = new DataView(buf);
    if (dv.getUint32(0, true) !== 0x4c494f53) {
      debug.badMagic++;
      return; // not "SOIL"
    }
    let o = 4;
    const f = {};
    f.frame = dv.getUint32(o, true); o += 4;
    f.simTime = dv.getFloat32(o, true); o += 4;
    f.stepMs = dv.getFloat32(o, true); o += 4;
    f.rtf = dv.getFloat32(o, true); o += 4;
    f.particles = dv.getUint32(o, true); o += 4;
    f.pos = [0, 1, 2].map((k) => dv.getFloat32(o + 4 * k, true)); o += 12;
    f.quat = [0, 1, 2, 3].map((k) => dv.getFloat32(o + 4 * k, true)); o += 16;
    f.force = [0, 1, 2].map((k) => dv.getFloat32(o + 4 * k, true)); o += 12;
    f.fill = dv.getFloat32(o, true); o += 4;
    f.cut = dv.getFloat32(o, true); o += 4;
    f.fee = dv.getFloat32(o, true); o += 4;
    const nx = dv.getUint16(o, true);
    const ny = dv.getUint16(o + 2, true);
    const x0 = dv.getFloat32(o + 4, true);
    const z0 = dv.getFloat32(o + 8, true);
    const cell = dv.getFloat32(o + 12, true);
    o += HM_HEADER_BYTES;
    const raw = new Uint16Array(buf.slice(o, o + nx * ny * 2));
    o += nx * ny * 2;
    const oy = -2.5;
    const heights = new Float32Array(nx * ny);
    for (let k = 0; k < raw.length; k++) heights[k] = raw[k] / 1000 - 1 + oy;
    f.hm = { nx, ny, x0, z0, cell, heights };
    const count = dv.getUint32(o, true);
    const lo = [dv.getFloat32(o + 4, true), dv.getFloat32(o + 8, true), dv.getFloat32(o + 12, true)];
    const span = dv.getFloat32(o + 16, true);
    o += P_HEADER_BYTES;
    f.grains = { count, lo, span, q: new Uint16Array(buf.slice(o, o + count * 6)) };
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
        statusEl.textContent = `Connected · ${hello.particles.toLocaleString()} particles, ${hello.voxel * 100} cm grid`;
        return;
      }
      try {
        handleFrame(ev.data);
      } catch (err) {
        debug.lastError = String(err);
      }
    };
    ws.onclose = () => {
      statusEl.textContent = `Soil server not reachable (${url}). Start phase2/soil_server.py; retrying…`;
      bucket.visible = false;
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

    bucket.visible = true;
    bucket.position.set(...f.pos);
    bucket.quaternion.set(...f.quat);

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
      ["Moving grains", g.count.toLocaleString()],
      ["Digging force", `${kN(Math.hypot(f.force[0], f.force[2]))} horiz · ${kN(f.force[1])} vert`],
      ["McKyes reference", f.cut > 0.01 ? `${kN(f.fee)} at ${(f.cut * 100).toFixed(0)} cm cut` : "–"],
      ["Bucket load", `${f.fill.toFixed(0)} kg`],
    ];
    statsEl.innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join("");
    window.__soil = { lastFrame: f, stats: { ...stats } };
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
