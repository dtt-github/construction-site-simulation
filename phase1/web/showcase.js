// Open-licensed real machines and site items from assets/registry.json, placed in the site.
// Machines keep their real joints (from the source URDF) in node extras; "Animate machines" drives
// them through their ranges to show the articulation. Static box colliders let site props hit them.

import { makeLinkage } from "./linkage.js";
import { steadyWidth } from "./statlist.js";

export async function addShowcase({ THREE, scene, loader, addStatic, terrainHeight, site, panel }) {
  const items = site.showcase ?? [];
  const animated = [];
  const linkages = [];
  for (const item of items) {
    let gltf;
    try {
      gltf = await loader.loadAsync(`../${item.file}`);
    } catch (err) {
      console.warn(`showcase: could not load ${item.file}`, err);
      continue;
    }
    const root = gltf.scene;
    const [x, y, z] = item.pos;
    root.position.set(x, terrainHeight(x, z) + (y ?? 0), z);
    root.rotation.y = THREE.MathUtils.degToRad(item.yaw ?? 0);
    scene.add(root);
    root.updateMatrixWorld(true);
    // cylinders and linkages follow the joints (linkage.js); free parts it can't pose stay hidden
    root.traverse((o) => {
      if (o.userData?.free_part) o.visible = false;
    });
    const linkage = makeLinkage(THREE, root);
    if (linkage) linkages.push(linkage);
    const driven = new Set(linkage?.drivenJoints ?? []);
    root.traverse((o) => {
      if (o.isMesh) o.castShadow = o.receiveShadow = true;
      const j = o.userData?.urdf_joint;
      if (j && j.type !== "fixed" && !driven.has(j.name) && !(item.animate_skip ?? []).includes(j.name)) {
        animated.push({ node: o, joint: j, rest: o.quaternion.clone(), restPos: o.position.clone(), phase: Math.random() * Math.PI * 2, cycle: item.cycle });
      }
    });
    linkage?.update();
    root.updateMatrixWorld(true);
    if (item.collider === "parts") {
      // one axis-aligned box per mesh, in world space (rough, but keeps props out of the machine)
      root.traverse((o) => {
        if (!o.isMesh) return;
        const b = new THREE.Box3().setFromObject(o);
        const s = b.getSize(new THREE.Vector3());
        if (s.x * s.y * s.z < 0.01) return;
        const c = b.getCenter(new THREE.Vector3());
        addStatic([s.x, s.y, s.z], [c.x, c.y, c.z], null);
      });
    } else if (Array.isArray(item.collider)) {
      // explicit footprint box [sx, sy, sz] centred on the item, resting on the ground
      const [sx, sy, sz] = item.collider;
      addStatic([sx, sy, sz], [root.position.x, root.position.y + sy / 2, root.position.z], null);
    }
  }

  // ---- articulation demo
  let on = false;
  let t = 0;
  const btn = document.createElement("button");
  btn.className = "secondary";
  btn.textContent = "Animate machines";
  btn.addEventListener("click", () => {
    on = !on;
    btn.textContent = on ? "Stop machines" : "Animate machines";
    if (!on) {
      for (const a of animated) {
        a.node.quaternion.copy(a.rest);
        a.node.position.copy(a.restPos);
      }
      for (const l of linkages) l.update();
    }
  });
  panel.querySelector(".buttons")?.appendChild(btn);
  steadyWidth(btn, ["Animate machines", "Stop machines"]);

  const q = new THREE.Quaternion();
  const axis = new THREE.Vector3();
  function setJoint(a, value) {
    axis.set(...a.joint.axis_local).normalize();
    if (a.joint.type === "prismatic") a.node.position.copy(a.restPos).addScaledVector(axis, value);
    else a.node.quaternion.copy(a.rest).multiply(q.setFromAxisAngle(axis, value));
  }
  // item.cycle: a work cycle as keyframes {period, keys: [[time s, {joint: value}], ...]}, eased between keys
  function cycleValue(cycle, name, time) {
    const keys = cycle.keys.filter(([, v]) => v[name] !== undefined);
    if (!keys.length) return undefined;
    const tt = time % cycle.period;
    for (let k = 0; k < keys.length; k++) {
      const [t0, v0] = keys[k];
      const [t1raw, v1] = keys[(k + 1) % keys.length];
      const t1 = k + 1 < keys.length ? t1raw : t1raw + cycle.period;
      const tq = tt < keys[0][0] ? tt + cycle.period : tt;
      if (tq >= t0 && tq <= t1) {
        const u = (tq - t0) / Math.max(t1 - t0, 1e-6);
        const e = u * u * (3 - 2 * u);
        return v0[name] + (v1[name] - v0[name]) * e;
      }
    }
    return keys[0][1][name];
  }
  function update(dt) {
    if (!on) return;
    t += dt;
    for (const a of animated) {
      const j = a.joint;
      axis.set(...j.axis_local).normalize();
      if (a.cycle) {
        const v = cycleValue(a.cycle, j.name, t);
        if (v !== undefined) setJoint(a, v);
        continue; // joints the cycle doesn't name stay put
      }
      const s = Math.sin(t * 0.6 + a.phase);
      if (j.type === "prismatic") {
        const lo = j.lower ?? 0, hi = j.upper ?? 0.2;
        a.node.position.copy(a.restPos).addScaledVector(axis, lo + (hi - lo) * (0.5 + 0.5 * s));
        continue;
      }
      let angle;
      if (j.type === "continuous") angle = 0.5 * s; // swing / wheels: gentle back-and-forth
      else {
        const lo = j.lower ?? -0.3, hi = j.upper ?? 0.3;
        angle = Math.min(Math.max(0, lo), hi) + 0.35 * (hi - lo) * s; // around the rest pose, within limits
        angle = Math.min(Math.max(angle, lo), hi);
      }
      a.node.quaternion.copy(a.rest).multiply(q.setFromAxisAngle(axis, angle));
    }
    for (const l of linkages) l.update();
  }
  window.__showcase = {
    count: items.length,
    animated: animated.length,
    joints: animated.map((a) => a.joint.name),
    linkages: linkages.length,
    // test hook: advance the articulation demo by t seconds and pose everything
    step(t) {
      on = true;
      update(t);
    },
    // test hook: set joints by name directly, e.g. pose({boom_joint: -0.4}), then pose the linkages
    pose(values) {
      for (const a of animated) if (values[a.joint.name] !== undefined) setJoint(a, values[a.joint.name]);
      for (const l of linkages) l.update();
    },
  };
  return { update };
}
