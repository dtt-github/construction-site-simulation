// Open-licensed real machines and site items from assets/registry.json, placed in the site.
// Machines keep their real joints (from the source URDF) in node extras; "Animate machines" drives
// them through their ranges to show the articulation. Static box colliders let site props hit them.

export async function addShowcase({ THREE, scene, loader, addStatic, terrainHeight, site, panel }) {
  const items = site.showcase ?? [];
  const animated = [];
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
    root.traverse((o) => {
      if (o.isMesh) o.castShadow = o.receiveShadow = true;
      const j = o.userData?.urdf_joint;
      if (j && j.type !== "fixed" && !(item.animate_skip ?? []).includes(j.name)) {
        animated.push({ node: o, joint: j, rest: o.quaternion.clone(), restPos: o.position.clone(), phase: Math.random() * Math.PI * 2 });
      }
    });
    scene.add(root);
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
    if (!on) for (const a of animated) {
      a.node.quaternion.copy(a.rest);
      a.node.position.copy(a.restPos);
    }
  });
  panel.querySelector(".buttons")?.appendChild(btn);

  const q = new THREE.Quaternion();
  const axis = new THREE.Vector3();
  function update(dt) {
    if (!on) return;
    t += dt;
    for (const a of animated) {
      const j = a.joint;
      axis.set(...j.axis_local).normalize();
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
  }
  window.__showcase = { count: items.length, animated: animated.length, joints: animated.map((a) => a.joint.name) };
  return { update };
}
