// Hydraulic cylinders and linkages, posed from the machine's joints so every part moves as it would on the
// machine: barrels turn about their head pins, rods slide out of them, the bucket four-bar folds.
//
// ZX200: pins and mesh frames from phase2/hydraulics.py (CYLINDERS, GUIDE_PIN, SIDE_PIN, CYLINDER_MESH,
// LINK_MESH); keep the two in step. Pins are (link, x, z) in the URDF link frame, which is the GLB node's own
// frame. The free parts (barrels, rods, link plates) are children of the model root.
//
// Cylinders built as URDF joints (D37PXi blade, IC120 vessel): a revolute head joint and a prismatic rod
// joint below it. The rod's far eye is found at rest from the rod mesh and pinned to the link it pushes.

const ZX200 = {
  links: { house: "body_link", boom: "boom_link", stick: "arm_link", bucket: "bucket_link" },
  cylinders: {
    boom: { head: ["house", 0.686, 0.247], rod: ["boom", 1.776, 2.249], mesh: [0.3425, 0.9395], length: 2.73 },
    arm: { head: ["boom", 2.276, 2.789], rod: ["stick", -0.778, -0.367], mesh: [0.9486, 0.3165], length: 1.7 },
    bucket: { head: ["stick", -0.248, 0.733], rod: null, mesh: [0.6761, 0.7368], length: 2.16 },
  },
  guide: ["stick", 1.577, 1.878],
  side: ["bucket", -0.328, 0.548],
  guideLen: 0.561,
  sideLen: 0.583,
  linkMesh: { side: [[0, 0], [-0.328, 0.482]], guide: [[0, 0], [-0.338, -0.448]] },
};

export function makeLinkage(THREE, root) {
  const byName = {};
  root.traverse((o) => (byName[o.name] ??= o));
  const solvers = [];
  if (byName.boom_barrel && byName.body_link) solvers.push(zx200Solver(THREE, root, byName));
  const pairs = cylinderPairs(THREE, root, byName);
  if (pairs) solvers.push(pairs);
  if (!solvers.length) return null;
  for (const name of ["boom_barrel", "boom_rod", "arm_barrel", "arm_rod", "bucket_barrel", "bucket_rod", "side_link", "guide_link"]) {
    if (byName[name]) byName[name].visible = true;
  }
  return { update: () => solvers.forEach((s) => s()), drivenJoints: pairs?.driven ?? [] };
}

function zx200Solver(THREE, root, byName) {
  const L = ZX200;
  const body = byName[L.links.house];
  const toHouse = {};
  const inv = new THREE.Matrix4();
  const tmp = new THREE.Matrix4();
  const point = ([link, x, z]) => new THREE.Vector3(x, 0, z).applyMatrix4(toHouse[link]);
  const Y = new THREE.Vector3(0, 1, 0); // pin axes: the house's +y
  const R = new THREE.Matrix4();
  const world = new THREE.Matrix4();
  const local = new THREE.Matrix4();

  // rotation taking the mesh frame (pin direction `dir` in its x-z plane, pin axis y) onto pins p0 -> p1
  function frame(p0, p1, dir) {
    const u = p1.clone().sub(p0).normalize();
    const w = Y.clone().addScaledVector(u, -Y.dot(u)).normalize();
    world.makeBasis(u, w, u.clone().cross(w));
    const d = new THREE.Vector3(dir[0], 0, dir[1]).normalize();
    local.makeBasis(d, Y, d.clone().cross(Y));
    return R.multiplyMatrices(world, local.transpose()).clone();
  }

  // place a part: rotation Rh and position ph in the house frame -> the part node's parent frame
  const houseToParent = new THREE.Matrix4();
  function place(name, Rh, ph) {
    const node = byName[name];
    if (!node) return;
    tmp.copy(Rh).setPosition(ph);
    houseToParent.copy(node.parent.matrixWorld).invert().multiply(body.matrixWorld);
    node.matrix.multiplyMatrices(houseToParent, tmp);
    node.matrix.decompose(node.position, node.quaternion, node.scale);
  }

  return function update() {
    root.updateMatrixWorld(true);
    inv.copy(body.matrixWorld).invert();
    for (const [k, n] of Object.entries(L.links)) toHouse[k] = new THREE.Matrix4().multiplyMatrices(inv, byName[n].matrixWorld);

    const pts = {};
    for (const [name, c] of Object.entries(L.cylinders)) {
      pts[name + "_head"] = point(c.head);
      if (c.rod) pts[name + "_rod"] = point(c.rod);
    }
    // bucket four-bar: the guide link (on the stick) and side link (on the bucket) meet at the rod eye
    const g = point(L.guide);
    const k = point(L.side);
    const d = Math.hypot(k.x - g.x, k.z - g.z);
    const a = (L.guideLen ** 2 - L.sideLen ** 2 + d ** 2) / (2 * d);
    const h = Math.sqrt(Math.max(L.guideLen ** 2 - a ** 2, 0));
    const ux = (k.x - g.x) / d;
    const uz = (k.z - g.z) / d;
    pts.bucket_rod = new THREE.Vector3(g.x + a * ux - h * uz, g.y, g.z + a * uz + h * ux);

    for (const [name, c] of Object.entries(L.cylinders)) {
      const head = pts[name + "_head"];
      const rod = pts[name + "_rod"];
      const Rc = frame(head, rod, c.mesh);
      const len = rod.distanceTo(head);
      place(name + "_barrel", Rc, head);
      place(name + "_rod", Rc, head.clone().lerp(rod, (len - c.length) / len)); // the rod slides out of the barrel
    }
    for (const [name, [p0a, p1a], [a0, a1]] of [
      ["side", L.linkMesh.side, [pts.bucket_rod, k]],
      ["guide", L.linkMesh.guide, [g, pts.bucket_rod]],
    ]) {
      const Rl = frame(a0, a1, [p1a[0] - p0a[0], p1a[1] - p0a[1]]);
      place(name + "_link", Rl, a0.clone().sub(new THREE.Vector3(p0a[0], 0, p0a[1]).applyMatrix4(Rl)));
    }
  };
}

// Cylinders modelled as URDF joints: head (revolute) -> rod (prismatic). Each rod's far eye is pinned to the
// link it pushes; per frame the head turns to point at that eye and the rod extends to reach it.
const PUSHES = {
  blade_arm_piston_joint: "blade_arm_link",
  blade_angle_piston_right_joint: "blade_angle_link",
  blade_angle_piston_left_joint: "blade_angle_link",
  cylinder_rod_prismatic_joint: "vessel_pin_link",
};

function cylinderPairs(THREE, root, byName) {
  const items = [];
  root.updateMatrixWorld(true);
  root.traverse((rodNode) => {
    const j = rodNode.userData?.urdf_joint;
    if (!j || j.type !== "prismatic" || !PUSHES[j.name]) return;
    const target = byName[PUSHES[j.name]];
    // a cylinder barrel is a free-turning (continuous) joint; a rod straight on a link (blade angle) has none
    const head = rodNode.parent?.userData?.urdf_joint?.type === "continuous" ? rodNode.parent : null;
    const axis = new THREE.Vector3(...j.axis_local).normalize(); // in the rod's parent frame
    // far eye: the rod mesh's furthest extent along the axis, in the rod node's frame at rest
    let reach = 0;
    rodNode.traverse((o) => {
      if (!o.isMesh) return;
      const pos = o.geometry.attributes.position;
      const m = new THREE.Matrix4().copy(rodNode.matrixWorld).invert().multiply(o.matrixWorld);
      const v = new THREE.Vector3();
      for (let i = 0; i < pos.count; i++) reach = Math.max(reach, v.fromBufferAttribute(pos, i).applyMatrix4(m).dot(axis));
    });
    const eyeLocal = axis.clone().multiplyScalar(reach); // rod frame
    const eyeOnTarget = rodNode.localToWorld(eyeLocal.clone()); // world at rest
    target.worldToLocal(eyeOnTarget);
    const anchor = head ?? rodNode.parent; // frame the rod's axis lives in
    const restDir = axis.clone();
    items.push({ rodNode, head, target, eyeOnTarget, anchor, axis, reach, restDir, restQ: head?.quaternion.clone(), restPos: rodNode.position.clone() });
  });
  if (!items.length) return null;
  const driven = items.flatMap((it) => [it.rodNode.userData.urdf_joint.name, it.head?.userData.urdf_joint.name].filter(Boolean));
  const eye = new THREE.Vector3();
  const q = new THREE.Quaternion();
  const update = () => {
    for (const it of items) {
      // the eye in the head's parent frame (head rotation reset to rest), then the turn and the stroke
      if (it.head) it.head.quaternion.copy(it.restQ);
      it.rodNode.position.copy(it.restPos);
      root.updateMatrixWorld(true);
      eye.copy(it.eyeOnTarget);
      it.target.localToWorld(eye);
      it.anchor.worldToLocal(eye); // anchor frame, at rest rotation
      const pivot = it.rodNode.position; // rod origin in the anchor frame (= head pin)
      const want = eye.clone().sub(pivot);
      if (it.head) {
        // turn the head about its joint axis so the rod axis points at the eye
        const jAxis = new THREE.Vector3(...it.head.userData.urdf_joint.axis_local).normalize();
        const a0 = it.axis.clone().projectOnPlane(jAxis).normalize();
        const a1 = want.clone().projectOnPlane(jAxis).normalize();
        const ang = Math.atan2(a0.clone().cross(a1).dot(jAxis), a0.dot(a1));
        it.head.quaternion.copy(it.restQ).multiply(q.setFromAxisAngle(jAxis, ang));
        want.applyAxisAngle(jAxis, -ang);
      }
      const stroke = want.dot(it.axis) - it.reach;
      it.rodNode.position.copy(it.restPos).addScaledVector(it.axis, stroke);
    }
  };
  update.driven = driven;
  return update;
}
