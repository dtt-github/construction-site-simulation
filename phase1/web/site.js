// Phase 1 prop site: SimReady GLBs (codepawl conversion) simulated with Jolt in the browser.
// Each GLB body node becomes a Jolt body built from its stored convex hulls; joints from the
// node extras become hinge, slider or fixed constraints. Masses come from site.json.

import * as THREE from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { MeshoptDecoder } from "three/addons/libs/meshopt_decoder.module.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { ConvexGeometry } from "three/addons/geometries/ConvexGeometry.js";
import initJolt from "https://cdn.jsdelivr.net/npm/jolt-physics@1.1.1/dist/jolt-physics.wasm-compat.js";

const LAYER_STATIC = 0;
const LAYER_MOVING = 1;
const FIXED_DT = 1 / 60;
const MAX_RESTITUTION = 0.3;

const statusEl = document.getElementById("status");
const statsEl = document.getElementById("stats");
const setStatus = (text, isError = false) => {
  statusEl.textContent = text;
  statusEl.classList.toggle("error", isError);
};

// ---------------------------------------------------------------- graphics
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.outputColorSpace = THREE.SRGBColorSpace;
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color("#b9c4cc");
scene.fog = new THREE.Fog("#b9c4cc", 60, 140);

const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 400);
camera.position.set(-34, 24, 40);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(-2, 0, 0);
controls.maxPolarAngle = Math.PI * 0.49;
controls.update();

scene.add(new THREE.HemisphereLight("#dfe8ef", "#6b5d4a", 1.1));
const sun = new THREE.DirectionalLight("#fff4e0", 2.4);
sun.position.set(-20, 35, 15);
sun.castShadow = true;
sun.shadow.mapSize.set(4096, 4096);
Object.assign(sun.shadow.camera, { left: -35, right: 35, top: 30, bottom: -30, near: 1, far: 100 });
sun.shadow.bias = -0.0004;
scene.add(sun);

function groundTexture() {
  const c = document.createElement("canvas");
  c.width = c.height = 512;
  const g = c.getContext("2d");
  g.fillStyle = "#d8d2c4";
  g.fillRect(0, 0, 512, 512);
  for (let i = 0; i < 9000; i++) {
    const v = 90 + Math.random() * 70;
    g.fillStyle = `rgba(${v + 20},${v + 8},${v - 10},${0.25 + Math.random() * 0.4})`;
    const r = Math.random() * 2.2 + 0.4;
    g.beginPath();
    g.arc(Math.random() * 512, Math.random() * 512, r, 0, Math.PI * 2);
    g.fill();
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(40, 40);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

// ---------------------------------------------------------------- physics
const Jolt = await initJolt();
let jolt, physicsSystem, bodyInterface;

function initPhysics() {
  const settings = new Jolt.JoltSettings();
  const objectFilter = new Jolt.ObjectLayerPairFilterTable(2);
  objectFilter.EnableCollision(LAYER_STATIC, LAYER_MOVING);
  objectFilter.EnableCollision(LAYER_MOVING, LAYER_MOVING);
  const bpStatic = new Jolt.BroadPhaseLayer(0);
  const bpMoving = new Jolt.BroadPhaseLayer(1);
  const bpInterface = new Jolt.BroadPhaseLayerInterfaceTable(2, 2);
  bpInterface.MapObjectToBroadPhaseLayer(LAYER_STATIC, bpStatic);
  bpInterface.MapObjectToBroadPhaseLayer(LAYER_MOVING, bpMoving);
  settings.mObjectLayerPairFilter = objectFilter;
  settings.mBroadPhaseLayerInterface = bpInterface;
  settings.mObjectVsBroadPhaseLayerFilter = new Jolt.ObjectVsBroadPhaseLayerFilterTable(bpInterface, 2, objectFilter, 2);
  jolt = new Jolt.JoltInterface(settings);
  Jolt.destroy(settings);
  physicsSystem = jolt.GetPhysicsSystem();
  bodyInterface = physicsSystem.GetBodyInterface();
}

const v3 = (v) => new Jolt.Vec3(v.x, v.y, v.z);
const rv3 = (v) => new Jolt.RVec3(v.x, v.y, v.z);
const jq = (q) => new Jolt.Quat(q.x, q.y, q.z, q.w);

// ---------------------------------------------------------------- assets
const loader = new GLTFLoader();
loader.setMeshoptDecoder(MeshoptDecoder);
const assetCache = new Map();

async function loadAsset(name) {
  if (assetCache.has(name)) return assetCache.get(name);
  const gltf = await loader.loadAsync(`../glb/${name}.glb`);
  const json = gltf.parser.json;
  const nodeObject = new Map();
  gltf.scene.traverse((o) => {
    const assoc = gltf.parser.associations.get(o);
    if (assoc && assoc.nodes !== undefined) nodeObject.set(assoc.nodes, o);
  });
  const materials = json.extensions?.KHR_physics_rigid_bodies?.physicsMaterials ?? [];
  const bodies = [];
  for (let i = 0; i < json.nodes.length; i++) {
    const node = json.nodes[i];
    const motion = node.extensions?.KHR_physics_rigid_bodies?.motion;
    if (!motion) continue;
    const extras = node.extras ?? {};
    const hulls = [];
    const ch = extras.collisionHulls;
    if (ch) {
      const buf = await gltf.parser.getDependency("bufferView", ch.bufferView);
      let offset = ch.byteOffset ?? 0;
      for (const count of ch.counts) {
        hulls.push(new Float32Array(buf.slice(offset, offset + count * 12)));
        offset += count * 12;
      }
    }
    // physics material: first collider below this body
    let friction = 0.5;
    let restitution = 0.1;
    nodeObject.get(i)?.traverse((o) => {
      const a = gltf.parser.associations.get(o);
      const col = a && json.nodes[a.nodes]?.extensions?.KHR_physics_rigid_bodies?.collider;
      if (col && col.physicsMaterial !== undefined && materials[col.physicsMaterial]) {
        friction = materials[col.physicsMaterial].dynamicFriction ?? friction;
        restitution = materials[col.physicsMaterial].restitution ?? restitution;
      }
    });
    bodies.push({ name: node.name, mass: motion.mass ?? 1, hulls, joints: extras.joints ?? [], friction, restitution, hullSource: "glb" });
  }
  // Finer convex decompositions (decompose_colliders.py) replace the GLB hulls where present
  try {
    const res = (site.hull_overrides ?? []).includes(name) ? await fetch(`hulls/${name}.json`) : null;
    if (res?.ok) {
      const override = await res.json();
      for (const b of bodies) {
        if (override[b.name]) {
          b.hulls = override[b.name].map((h) => new Float32Array(h));
          b.hullSource = "coacd";
        }
      }
    }
  } catch {
    /* no override */
  }
  // optional ground clearance: lift hull points of a part above a minimum height (body frame)
  const minY = site.assets[name]?.hull_min_y ?? {};
  for (const b of bodies) {
    if (minY[b.name] === undefined) continue;
    for (const h of b.hulls) for (let k = 1; k < h.length; k += 3) h[k] = Math.max(h[k], minY[b.name]);
  }
  const data = { gltf, bodies, glbMass: bodies.reduce((s, b) => s + b.mass, 0) };
  assetCache.set(name, data);
  return data;
}

function hullShapeSettings(points) {
  const s = new Jolt.ConvexHullShapeSettings();
  for (let k = 0; k < points.length; k += 3) s.mPoints.push_back(new Jolt.Vec3(points[k], points[k + 1], points[k + 2]));
  s.mMaxConvexRadius = 0.01;
  return s;
}

function bodyShape(body, obj) {
  if (body.hulls.length === 1) return hullShapeSettings(body.hulls[0]).Create().Get();
  if (body.hulls.length > 1) {
    const c = new Jolt.StaticCompoundShapeSettings();
    for (const h of body.hulls) c.AddShapeShapeSettings(new Jolt.Vec3(0, 0, 0), new Jolt.Quat(0, 0, 0, 1), hullShapeSettings(h), 0);
    return c.Create().Get();
  }
  // fallback: box around the visual mesh, in body space
  const box = new THREE.Box3().setFromObject(obj);
  const size = box.getSize(new THREE.Vector3()).multiplyScalar(0.5).max(new THREE.Vector3(0.02, 0.02, 0.02));
  return new Jolt.BoxShape(new Jolt.Vec3(size.x, size.y, size.z), 0.01, null);
}

// ---------------------------------------------------------------- world state
const live = []; // { obj, id, mass, debug }
const constraints = [];
let soil = null; // Phase 2 soil stream (soil.js)
const instances = []; // { asset, joints: { name: { c, type, motor } }, ids }
const pickable = new Map(); // mesh uuid -> live entry
let showHulls = false;
let site;
const stats = { assets: 0, bodies: 0, constraints: 0, motors: 0, totalMass: 0, loadMs: 0, stepMs: 0, fps: 0, skippedJoints: 0 };

function addStatic(size, pos, color, friction = 0.7) {
  const shape = new Jolt.BoxShape(new Jolt.Vec3(size[0] / 2, size[1] / 2, size[2] / 2), 0.02, null);
  const bcs = new Jolt.BodyCreationSettings(shape, new Jolt.RVec3(...pos), new Jolt.Quat(0, 0, 0, 1), Jolt.EMotionType_Static, LAYER_STATIC);
  bcs.mFriction = friction;
  const body = bodyInterface.CreateBody(bcs);
  bodyInterface.AddBody(body.GetID(), Jolt.EActivation_DontActivate);
  Jolt.destroy(bcs);
  if (color) {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(...size), new THREE.MeshStandardMaterial({ color, roughness: 0.95 }));
    mesh.position.set(...pos);
    mesh.castShadow = mesh.receiveShadow = true;
    scene.add(mesh);
  }
}

function hullDebug(body) {
  const group = new THREE.Group();
  const mat = new THREE.MeshBasicMaterial({ color: "#41d1ff", wireframe: true, transparent: true, opacity: 0.6 });
  for (const h of body.hulls) {
    const pts = [];
    for (let k = 0; k < h.length; k += 3) pts.push(new THREE.Vector3(h[k], h[k + 1], h[k + 2]));
    try {
      group.add(new THREE.Mesh(new ConvexGeometry(pts), mat));
    } catch {
      /* degenerate hull */
    }
  }
  group.visible = showHulls;
  return group;
}

let groupCounter = 0;

async function spawn(assetName, pos, yawDeg, massOverride) {
  const data = await loadAsset(assetName);
  const root = data.gltf.scene.clone(true);
  root.position.set(...pos);
  root.rotation.y = THREE.MathUtils.degToRad(yawDeg ?? 0);
  scene.add(root);
  root.updateMatrixWorld(true);
  root.traverse((o) => {
    if (o.isMesh) o.castShadow = o.receiveShadow = true;
  });

  const scale = massOverride ? massOverride / data.glbMass : 1;
  const filter = new Jolt.GroupFilterTable(Math.max(data.bodies.length, 1));
  for (let a = 0; a < data.bodies.length; a++) for (let b = a + 1; b < data.bodies.length; b++) filter.DisableCollision(a, b);
  const group = ++groupCounter;

  const byName = new Map();
  const tmpP = new THREE.Vector3();
  const tmpQ = new THREE.Quaternion();
  const tmpS = new THREE.Vector3();
  data.bodies.forEach((b, idx) => {
    const obj = root.getObjectByName(b.name);
    if (!obj) return;
    obj.matrixWorld.decompose(tmpP, tmpQ, tmpS);
    const shape = bodyShape(b, obj);
    const bcs = new Jolt.BodyCreationSettings(shape, rv3(tmpP), jq(tmpQ), Jolt.EMotionType_Dynamic, LAYER_MOVING);
    const mass = Math.max(b.mass * scale, 0.05);
    bcs.mOverrideMassProperties = Jolt.EOverrideMassProperties_CalculateInertia;
    const mp = new Jolt.MassProperties();
    mp.mMass = mass;
    bcs.mMassPropertiesOverride = mp;
    Jolt.destroy(mp);
    bcs.mFriction = b.friction;
    bcs.mRestitution = Math.min(b.restitution, MAX_RESTITUTION);
    bcs.mCollisionGroup.SetGroupFilter(filter);
    bcs.mCollisionGroup.SetGroupID(group);
    bcs.mCollisionGroup.SetSubGroupID(idx);
    const body = bodyInterface.CreateBody(bcs);
    Jolt.destroy(bcs);
    bodyInterface.AddBody(body.GetID(), Jolt.EActivation_Activate);
    scene.attach(obj);
    const debug = hullDebug(b);
    obj.add(debug);
    const entry = { obj, id: body.GetID(), mass, debug, asset: assetName };
    live.push(entry);
    byName.set(b.name, { entry, body });
    obj.traverse((o) => o.isMesh && pickable.set(o.uuid, entry));
    stats.bodies++;
    stats.totalMass += mass;
  });

  // joints (pivot and axis are in model space at rest)
  const rootQ = root.getWorldQuaternion(new THREE.Quaternion());
  const motorCfg = site.assets[assetName]?.motors ?? {};
  const instance = { asset: assetName, joints: {}, ids: [...byName.values()].map((v) => v.entry.id) };
  for (const b of data.bodies) {
    for (const j of b.joints) {
      const child = byName.get(b.name);
      const parent = byName.get(j.parent);
      if (!child || !parent) {
        stats.skippedJoints++;
        continue;
      }
      const pivot = new THREE.Vector3(...(j.pivot ?? [0, 0, 0])).applyMatrix4(root.matrixWorld);
      const axis = new THREE.Vector3(...(j.axis ?? [1, 0, 0])).applyQuaternion(rootQ).normalize();
      const normal = Math.abs(axis.y) < 0.9 ? new THREE.Vector3(0, 1, 0) : new THREE.Vector3(1, 0, 0);
      normal.sub(axis.clone().multiplyScalar(normal.dot(axis))).normalize();
      let settings;
      if (j.type === "revolute") {
        settings = new Jolt.HingeConstraintSettings();
        settings.mPoint1 = settings.mPoint2 = rv3(pivot);
        settings.mHingeAxis1 = settings.mHingeAxis2 = v3(axis);
        settings.mNormalAxis1 = settings.mNormalAxis2 = v3(normal);
        if (j.min !== undefined && j.max !== undefined) {
          settings.mLimitsMin = THREE.MathUtils.degToRad(j.min);
          settings.mLimitsMax = THREE.MathUtils.degToRad(j.max);
        }
        settings.mMaxFrictionTorque = 0.5;
      } else if (j.type === "prismatic") {
        settings = new Jolt.SliderConstraintSettings();
        settings.mAutoDetectPoint = false;
        settings.mPoint1 = settings.mPoint2 = rv3(pivot);
        settings.mSliderAxis1 = settings.mSliderAxis2 = v3(axis);
        settings.mNormalAxis1 = settings.mNormalAxis2 = v3(normal);
        if (j.min !== undefined && j.max !== undefined) {
          settings.mLimitsMin = j.min;
          settings.mLimitsMax = j.max;
        }
        settings.mMaxFrictionForce = 20;
      } else {
        settings = new Jolt.FixedConstraintSettings();
        settings.mAutoDetectPoint = true;
      }
      const motor = motorCfg[j.name];
      if (motor && (j.type === "revolute" || j.type === "prismatic")) {
        const ms = new Jolt.MotorSettings();
        const spring = new Jolt.SpringSettings();
        spring.mFrequency = motor.frequency ?? 2;
        spring.mDamping = 1;
        ms.mSpringSettings = spring;
        const limit = motor.max_force ?? motor.max_torque ?? 1000;
        ms.mMinForceLimit = ms.mMinTorqueLimit = -limit;
        ms.mMaxForceLimit = ms.mMaxTorqueLimit = limit;
        settings.mMotorSettings = ms;
        Jolt.destroy(ms);
        Jolt.destroy(spring);
      }
      const c = settings.Create(parent.body, child.body);
      physicsSystem.AddConstraint(c);
      constraints.push(c);
      stats.constraints++;
      if (motor) {
        const typed = j.type === "revolute" ? Jolt.castObject(c, Jolt.HingeConstraint) : Jolt.castObject(c, Jolt.SliderConstraint);
        typed.SetMotorState(motor.mode === "velocity" ? Jolt.EMotorState_Velocity : Jolt.EMotorState_Position);
        instance.joints[j.name] = { c: typed, type: j.type, motor };
        stats.motors++;
      }
    }
  }
  stats.assets++;
  instances.push(instance);
  return data;
}

// Terrain: flat working pad, rolling ground around it, a stockpile and an excavation pit.
const TERRAIN = { size: 128, samples: 256 };
const smooth = (e0, e1, x) => {
  const t = Math.min(Math.max((x - e0) / (e1 - e0), 0), 1);
  return t * t * (3 - 2 * t);
};
function terrainHeight(x, z) {
  // outside the pad (|x| < 30, |z| < 22) the ground rises into low rolling hills
  const dx = Math.max(Math.abs(x) - 30, 0);
  const dz = Math.max(Math.abs(z) - 22, 0);
  const out = Math.hypot(dx, dz);
  const hills = 0.9 * Math.sin(x * 0.11) * Math.cos(z * 0.13) + 0.6 * Math.sin(x * 0.27 + z * 0.19) + 0.35 * Math.cos(x * 0.53 - z * 0.41);
  let h = smooth(0, 14, out) * (3.5 + hills + 1.2);
  // haul road leaving the pad through the gate (x ~ -11), graded at about 1:12
  const road = 1 - smooth(4, 7, Math.abs(x + 11));
  if (z > 22) h = h * (1 - road) + road * Math.min((z - 22) / 12, 4);
  // spoil stockpile: rounded cone, ~3 m high, flank ~30 degrees (below sand's angle of repose)
  const sr = Math.hypot(x + 22, z + 15);
  h += 3.2 * (1 - smooth(1.5, 6.5, sr));
  // excavation pit: 6 m x 5 m floor, 2.5 m deep, sides 1V:1.5H
  const px = Math.max(Math.abs(x - 24) - 3, 0);
  const pz = Math.max(Math.abs(z - 12) - 2.5, 0);
  const pd = Math.hypot(px, pz);
  h -= Math.max(2.5 - pd / 1.5, 0);
  return h;
}

function buildTerrain() {
  const N = TERRAIN.samples;
  const step = TERRAIN.size / (N - 1);
  const half = TERRAIN.size / 2;
  const hs = new Jolt.HeightFieldShapeSettings();
  hs.mSampleCount = N;
  hs.mBlockSize = 4;
  const off = new Jolt.Vec3(-half, 0, -half);
  const sc = new Jolt.Vec3(step, 1, step);
  hs.mOffset = off;
  hs.mScale = sc;
  hs.mHeightSamples.resize(N * N);
  const samples = new Float32Array(Jolt.HEAPF32.buffer, Jolt.getPointer(hs.mHeightSamples.data()), N * N);
  const geo = new THREE.PlaneGeometry(TERRAIN.size, TERRAIN.size, N - 1, N - 1);
  geo.rotateX(-Math.PI / 2);
  const pos = geo.attributes.position;
  const colors = new Float32Array(pos.count * 3);
  const pad = new THREE.Color("#b3a68f");
  const dirt = new THREE.Color("#a3815c");
  const pit = new THREE.Color("#7d5f44");
  const grass = new THREE.Color("#8d9a5e");
  const c = new THREE.Color();
  for (let iz = 0; iz < N; iz++) {
    for (let ix = 0; ix < N; ix++) {
      const x = -half + ix * step;
      const z = -half + iz * step;
      const h = terrainHeight(x, z);
      samples[iz * N + ix] = h; // Jolt sample (x, y) -> world (x, z)
      const v = iz * N + ix; // PlaneGeometry rotated: rows run along +z, columns along +x
      pos.setY(v, h);
      const rough = 0.92 + 0.16 * Math.random();
      if (h < -0.15) c.copy(pit);
      else if (Math.abs(h) < 0.05) c.copy(pad);
      else if (Math.hypot(Math.max(Math.abs(x) - 30, 0), Math.max(Math.abs(z) - 22, 0)) > 9) c.copy(grass);
      else c.copy(dirt);
      c.multiplyScalar(rough);
      colors.set([c.r, c.g, c.b], v * 3);
    }
  }
  geo.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  geo.computeVertexNormals();
  const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 1, map: groundTexture() }));
  mesh.receiveShadow = true;
  scene.add(mesh);

  const shape = hs.Create().Get();
  Jolt.destroy(off);
  Jolt.destroy(sc);
  Jolt.destroy(hs);
  const bcs = new Jolt.BodyCreationSettings(shape, new Jolt.RVec3(0, 0, 0), new Jolt.Quat(0, 0, 0, 1), Jolt.EMotionType_Static, LAYER_STATIC);
  bcs.mFriction = site.ground.friction;
  const body = bodyInterface.CreateBody(bcs);
  bodyInterface.AddBody(body.GetID(), Jolt.EActivation_DontActivate);
  Jolt.destroy(bcs);
}

// ---------------------------------------------------------------- build / reset
async function build() {
  const t0 = performance.now();
  initPhysics();
  buildTerrain();
  // placement heights are relative to the terrain under the object
  const onGround = ([x, y, z]) => [x, terrainHeight(x, z) + y, z];
  for (const b of site.static_blocks) addStatic(b.size, onGround(b.pos), b.color);

  let n = 0;
  for (const p of site.placements) {
    const a = site.assets[p.asset] ?? {};
    setStatus(`Loading ${p.asset} (${++n}/${site.placements.length})`);
    await spawn(p.asset, onGround(p.pos), p.yaw, a.mass_kg ?? null);
  }
  setupControls();
  try {
    const { startSoil } = await import(`./soil.js?v=${Date.now()}`);
    soil = startSoil({
      THREE,
      scene,
      Jolt,
      jolt,
      bodyInterface,
      physicsSystem,
      layerStatic: LAYER_STATIC,
      panel: document.getElementById("panel"),
      view: (...a) => window.__site.view(...a),
    });
  } catch (err) {
    console.error("soil client failed", err);
  }
  stats.loadMs = performance.now() - t0;
  setStatus(`Ready. ${stats.assets} assets loaded in ${(stats.loadMs / 1000).toFixed(1)} s`);
  fillMassTable();
}

async function fillMassTable() {
  let usd = {};
  try {
    usd = await (await fetch("mass_table.json")).json();
  } catch {
    /* optional */
  }
  const tbody = document.querySelector("#masses tbody");
  tbody.innerHTML = "";
  for (const [name, cfg] of Object.entries(site.assets)) {
    const data = assetCache.get(name);
    const used = cfg.mass_kg ?? data?.glbMass;
    const row = document.createElement("tr");
    const fmt = (x) => (x === undefined || x === null ? "–" : x >= 100 ? Math.round(x) : x.toFixed(1));
    row.innerHTML = `<td title="${cfg.mass_note ?? ""}">${name.replace(/_a0\d$|_b0\d$|_c0\d$/, "")}</td><td class="num">${fmt(usd[name])}</td><td class="num">${fmt(data?.glbMass)}</td><td class="num">${fmt(used)}</td>`;
    tbody.appendChild(row);
  }
}

// ---------------------------------------------------------------- machine controls
function wake(inst) {
  for (const id of inst.ids) bodyInterface.ActivateBody(id);
}

function setJoint(inst, name, value) {
  const j = inst?.joints[name];
  if (!j) return;
  if (j.motor.rate && j.motor.mode !== "velocity") {
    // hydraulic actuators move at a limited speed: the tick loop walks the target there
    j.goal = value;
    j.command ??= 0;
    ramping.add(j);
    wake(inst);
    j.inst = inst;
    return;
  }
  applyTarget(j, value);
  wake(inst);
}

const ramping = new Set();
function applyTarget(j, value) {
  const mode = j.motor.mode;
  if (j.type === "prismatic") mode === "velocity" ? j.c.SetTargetVelocity(value) : j.c.SetTargetPosition(value);
  else mode === "velocity" ? j.c.SetTargetAngularVelocity(value) : j.c.SetTargetAngle(value);
}

function stepRamps(dt) {
  for (const j of ramping) {
    const d = j.goal - j.command;
    const maxStep = j.motor.rate * dt;
    j.command += Math.abs(d) <= maxStep ? d : Math.sign(d) * maxStep;
    applyTarget(j, j.command);
    wake(j.inst);
    if (j.command === j.goal) ramping.delete(j);
  }
}

function setupControls() {
  const forklift = instances.find((i) => i.asset === "vehicle_forklift_orange_a01");
  const lift = instances.find((i) => i.asset === "vehicleSupport_scissor_lift_c01");
  const F = site.assets.vehicle_forklift_orange_a01?.controls ?? {};
  const L = site.assets.vehicleSupport_scissor_lift_c01?.controls ?? {};

  const fork = document.getElementById("ctl-fork");
  const forkOut = document.getElementById("ctl-fork-out");
  fork.addEventListener("input", () => {
    forkOut.textContent = `${Number(fork.value).toFixed(2)} m`;
    setJoint(forklift, F.lift, Number(fork.value));
  });
  const steer = document.getElementById("ctl-steer");
  const steerOut = document.getElementById("ctl-steer-out");
  steer.addEventListener("input", () => {
    steerOut.textContent = `${steer.value}°`;
    setJoint(forklift, F.steer, THREE.MathUtils.degToRad(Number(steer.value)));
  });
  const drive = (dir) => {
    for (const w of F.drive_wheels ?? []) setJoint(forklift, w, dir * (F.drive_rad_s ?? 10) * (F.drive_sign ?? 1));
  };
  for (const [id, dir] of [["ctl-fwd", 1], ["ctl-rev", -1]]) {
    const b = document.getElementById(id);
    b.addEventListener("pointerdown", () => drive(dir));
    for (const ev of ["pointerup", "pointerleave"]) b.addEventListener(ev, () => drive(0));
  }
  const ramp = document.getElementById("ctl-ramp");
  const rampOut = document.getElementById("ctl-ramp-out");
  ramp.addEventListener("input", () => {
    rampOut.textContent = `${ramp.value}°`;
    for (const r of L.ramps ?? []) setJoint(lift, r, THREE.MathUtils.degToRad(Number(ramp.value)) * (L.ramp_sign ?? 1));
  });
  // start with drive wheels stopped (velocity motors hold zero speed = parking brake)
  drive(0);
}

// ---------------------------------------------------------------- interaction
const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
let downAt = null;
renderer.domElement.addEventListener("pointerdown", (e) => (downAt = [e.clientX, e.clientY]));
renderer.domElement.addEventListener("pointerup", (e) => {
  if (!downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 4) return;
  pointer.set((e.clientX / window.innerWidth) * 2 - 1, -(e.clientY / window.innerHeight) * 2 + 1);
  raycaster.setFromCamera(pointer, camera);
  const hit = raycaster.intersectObjects(scene.children, true).find((h) => pickable.has(h.object.uuid));
  if (!hit) return;
  const entry = pickable.get(hit.object.uuid);
  const dir = raycaster.ray.direction.clone().setY(0).normalize().add(new THREE.Vector3(0, 0.25, 0));
  const impulse = dir.multiplyScalar(entry.mass * 3);
  bodyInterface.AddImpulse(entry.id, v3(impulse), rv3(hit.point));
  bodyInterface.ActivateBody(entry.id);
});

document.getElementById("btn-drop").addEventListener("click", async () => {
  for (let i = 0; i < 6; i++) {
    await spawn("crate_plywood_b01", [2 + Math.random() * 8, 4 + i * 1.3, -8 + Math.random() * 4], Math.random() * 90, site.assets.crate_plywood_b01.mass_kg);
  }
});
document.getElementById("btn-beam").addEventListener("click", () => spawn("structure_beam_a01", [6, 7, -3], 20, null));
document.getElementById("btn-hulls").addEventListener("click", (e) => {
  showHulls = !showHulls;
  e.target.textContent = showHulls ? "Hide colliders" : "Show colliders";
  for (const l of live) l.debug.visible = showHulls;
});
document.getElementById("btn-reset").addEventListener("click", () => location.reload());

// ---------------------------------------------------------------- loop
const clock = new THREE.Clock();
let acc = 0;
let frames = 0;
let fpsT = performance.now();
const tmpPos = new THREE.Vector3();

function renderStats() {
  const rows = [
    ["Assets", stats.assets],
    ["Rigid bodies", stats.bodies],
    ["Joints", stats.constraints + (stats.skippedJoints ? ` (+${stats.skippedJoints} skipped)` : "")],
    ["Motors", stats.motors],
    ["Total mass", `${(stats.totalMass / 1000).toFixed(2)} t`],
    ["Physics step", stats.stepMs < 0.005 ? "all asleep" : `${stats.stepMs.toFixed(2)} ms`],
    ["Frame rate", `${stats.fps.toFixed(0)} fps`],
  ];
  statsEl.innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join("");
}

function tick() {
  requestAnimationFrame(tick);
  const dt = Math.min(clock.getDelta(), 0.1);
  if (jolt) {
    acc += dt;
    let steps = 0;
    while (acc >= FIXED_DT && steps < 4) {
      const t = performance.now();
      stepRamps(FIXED_DT);
      jolt.Step(FIXED_DT, 1);
      stats.stepMs = stats.stepMs * 0.9 + (performance.now() - t) * 0.1;
      acc -= FIXED_DT;
      steps++;
    }
    if (steps === 4) acc = 0;
    for (const l of live) {
      const p = bodyInterface.GetPosition(l.id);
      const q = bodyInterface.GetRotation(l.id);
      l.obj.position.set(p.GetX(), p.GetY(), p.GetZ());
      l.obj.quaternion.set(q.GetX(), q.GetY(), q.GetZ(), q.GetW());
    }
  }
  soil?.update();
  controls.update();
  renderer.render(scene, camera);
  frames++;
  const now = performance.now();
  if (now - fpsT > 500) {
    stats.fps = (frames * 1000) / (now - fpsT);
    frames = 0;
    fpsT = now;
    renderStats();
  }
}

window.addEventListener("resize", () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// Test hook: positions and speeds of every body
window.__site = {
  stats,
  view(px, py, pz, tx, ty, tz) {
    camera.position.set(px, py, pz);
    controls.target.set(tx, ty, tz);
    controls.update();
  },
  snapshot() {
    return live.map((l) => {
      const p = bodyInterface.GetPosition(l.id);
      const v = bodyInterface.GetLinearVelocity(l.id);
      return {
        asset: l.asset,
        name: l.obj.name,
        x: p.GetX(),
        y: p.GetY(),
        z: p.GetZ(),
        ground: terrainHeight(p.GetX(), p.GetZ()),
        speed: Math.hypot(v.GetX(), v.GetY(), v.GetZ()),
      };
    });
  },
  massCheck() {
    // mass Jolt actually uses, per body (1 / inverse mass)
    const lock = physicsSystem.GetBodyLockInterfaceNoLock();
    return live.map((l) => ({ asset: l.asset, name: l.obj.name, intended: l.mass, jolt: 1 / lock.TryGetBody(l.id).GetMotionProperties().GetInverseMass() }));
  },
  motorInfo() {
    return instances.flatMap((i) =>
      Object.entries(i.joints).map(([n, j]) => {
        const ms = j.c.GetMotorSettings();
        return { asset: i.asset, joint: n, state: j.c.GetMotorState(), maxF: ms.mMaxForceLimit, maxT: ms.mMaxTorqueLimit, freq: ms.mSpringSettings.mFrequency };
      })
    );
  },
  jointValue(asset, name) {
    const j = instances.find((i) => i.asset === asset)?.joints[name];
    if (!j) return null;
    return j.type === "prismatic" ? j.c.GetCurrentPosition() : j.c.GetCurrentAngle();
  },
};

try {
  site = await (await fetch("site.json")).json();
  renderStats();
  tick();
  await build();
} catch (err) {
  console.error(err);
  setStatus(`Error: ${err.message}`, true);
}
