/* NexusVIT 3D viewport — 6-axis cobot, bin, table. Uses global THREE. */
(function (global) {
  "use strict";

  const HOME = [0.18, -0.55, 1.15, 0.0, 0.85, 0.2];
  const CAP = [0.42, 0.18, 0.95, 0.15, 1.25, 0.0];
  const PICK = [0.55, 0.35, 1.05, 0.1, 1.35, 0.35];

  function deg(rad) {
    return (rad * 180) / Math.PI;
  }

  function createMaterials() {
    return {
      white: new THREE.MeshStandardMaterial({
        color: 0xd9dee6,
        metalness: 0.55,
        roughness: 0.28,
      }),
      red: new THREE.MeshStandardMaterial({
        color: 0xc62828,
        metalness: 0.45,
        roughness: 0.32,
      }),
      dark: new THREE.MeshStandardMaterial({
        color: 0x2a3138,
        metalness: 0.7,
        roughness: 0.35,
      }),
      table: new THREE.MeshStandardMaterial({
        color: 0x6a717a,
        metalness: 0.2,
        roughness: 0.72,
      }),
      bin: new THREE.MeshStandardMaterial({
        color: 0x1a1e24,
        metalness: 0.15,
        roughness: 0.8,
      }),
      brass: new THREE.MeshStandardMaterial({
        color: 0xc4a35a,
        metalness: 0.8,
        roughness: 0.28,
      }),
      steel: new THREE.MeshStandardMaterial({
        color: 0x9aa3ad,
        metalness: 0.85,
        roughness: 0.22,
      }),
      alum: new THREE.MeshStandardMaterial({
        color: 0xcfd6dd,
        metalness: 0.7,
        roughness: 0.3,
      }),
    };
  }

  function mesh(geo, mat) {
    const m = new THREE.Mesh(geo, mat);
    m.castShadow = true;
    m.receiveShadow = true;
    return m;
  }

  function buildRobot(mats) {
    const root = new THREE.Group();
    const joints = [];

    const base = mesh(new THREE.CylinderGeometry(0.11, 0.13, 0.08, 32), mats.dark);
    base.position.y = 0.04;
    root.add(base);
    const collar = mesh(new THREE.CylinderGeometry(0.09, 0.1, 0.05, 24), mats.red);
    collar.position.y = 0.085;
    root.add(collar);

    const j0 = new THREE.Group();
    j0.position.y = 0.11;
    root.add(j0);
    joints.push(j0);

    const waist = mesh(new THREE.CylinderGeometry(0.075, 0.08, 0.12, 24), mats.white);
    waist.position.y = 0.06;
    j0.add(waist);
    const shoulderHouse = mesh(new THREE.BoxGeometry(0.16, 0.12, 0.12), mats.red);
    shoulderHouse.position.set(0, 0.15, 0);
    j0.add(shoulderHouse);

    const j1 = new THREE.Group();
    j1.position.set(0, 0.16, 0.02);
    j0.add(j1);
    joints.push(j1);

    const upper = mesh(new THREE.BoxGeometry(0.08, 0.42, 0.08), mats.white);
    upper.position.set(0, 0.21, 0);
    j1.add(upper);
    const upperCap = mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.1, 20), mats.red);
    upperCap.rotation.z = Math.PI / 2;
    upperCap.position.set(0.06, 0.0, 0);
    j1.add(upperCap);

    const j2 = new THREE.Group();
    j2.position.set(0, 0.42, 0);
    j1.add(j2);
    joints.push(j2);

    const elbow = mesh(new THREE.BoxGeometry(0.1, 0.1, 0.1), mats.red);
    j2.add(elbow);
    const forearm = mesh(new THREE.BoxGeometry(0.065, 0.34, 0.065), mats.white);
    forearm.position.set(0, 0.17, 0);
    j2.add(forearm);

    const j3 = new THREE.Group();
    j3.position.set(0, 0.36, 0);
    j2.add(j3);
    joints.push(j3);

    const wrist1 = mesh(new THREE.CylinderGeometry(0.04, 0.04, 0.08, 20), mats.red);
    wrist1.rotation.x = Math.PI / 2;
    j3.add(wrist1);

    const j4 = new THREE.Group();
    j4.position.set(0, 0.05, 0);
    j3.add(j4);
    joints.push(j4);

    const wrist2 = mesh(new THREE.BoxGeometry(0.07, 0.12, 0.07), mats.white);
    wrist2.position.y = 0.06;
    j4.add(wrist2);

    const j5 = new THREE.Group();
    j5.position.set(0, 0.13, 0);
    j4.add(j5);
    joints.push(j5);

    const flange = mesh(new THREE.CylinderGeometry(0.035, 0.04, 0.04, 20), mats.dark);
    flange.position.y = 0.02;
    j5.add(flange);

    const grip = new THREE.Group();
    grip.position.y = 0.05;
    j5.add(grip);
    const g1 = mesh(new THREE.BoxGeometry(0.012, 0.07, 0.018), mats.dark);
    g1.position.set(0.018, 0.03, 0);
    const g2 = g1.clone();
    g2.position.x = -0.018;
    grip.add(g1, g2);

    const tcp = new THREE.Object3D();
    tcp.position.y = 0.09;
    j5.add(tcp);

    return { root, joints, tcp };
  }

  function buildWorld(scene, mats) {
    const table = mesh(new THREE.BoxGeometry(1.6, 0.08, 1.0), mats.table);
    table.position.set(0.15, -0.04, 0.05);
    scene.add(table);

    const legGeo = new THREE.BoxGeometry(0.06, 0.42, 0.06);
    [
      [-0.55, -0.25, 0.42],
      [0.85, -0.25, 0.42],
      [-0.55, -0.25, -0.32],
      [0.85, -0.25, -0.32],
    ].forEach(([x, y, z]) => {
      const leg = mesh(legGeo, mats.dark);
      leg.position.set(x, y, z);
      scene.add(leg);
    });

    const bin = new THREE.Group();
    const floor = mesh(new THREE.BoxGeometry(0.42, 0.02, 0.32), mats.bin);
    floor.position.y = 0.02;
    bin.add(floor);
    const wallMat = mats.bin;
    const walls = [
      [0.42, 0.12, 0.02, 0, 0.08, 0.15],
      [0.42, 0.12, 0.02, 0, 0.08, -0.15],
      [0.02, 0.12, 0.32, 0.2, 0.08, 0],
      [0.02, 0.12, 0.32, -0.2, 0.08, 0],
    ];
    walls.forEach(([w, h, d, x, y, z]) => {
      const wall = mesh(new THREE.BoxGeometry(w, h, d), wallMat);
      wall.position.set(x, y, z);
      bin.add(wall);
    });
    bin.position.set(0.48, 0.0, 0.12);
    scene.add(bin);

    const partSpecs = [
      [0.06, 0.03, 0.03, mats.brass, -0.08, 0.04, 0.04, 0.4],
      [0.05, 0.02, 0.05, mats.steel, 0.06, 0.035, -0.05, -0.3],
      [0.04, 0.04, 0.02, mats.alum, 0.0, 0.04, 0.07, 0.8],
      [0.07, 0.02, 0.025, mats.brass, 0.1, 0.03, 0.02, 1.1],
      [0.03, 0.03, 0.03, mats.steel, -0.04, 0.04, -0.08, 0.2],
    ];
    const parts = [];
    partSpecs.forEach(([w, h, d, mat, x, y, z, rot]) => {
      const p = mesh(new THREE.BoxGeometry(w, h, d), mat);
      p.position.set(x, y, z);
      p.rotation.y = rot;
      bin.add(p);
      parts.push(p);
    });

    const grid = new THREE.GridHelper(3.2, 32, 0x3a4654, 0x1c2430);
    grid.position.y = -0.46;
    scene.add(grid);

    const axes = new THREE.AxesHelper(0.22);
    axes.position.set(0, 0.01, 0);
    scene.add(axes);

    return { bin, parts };
  }

  function attachOrbit(camera, canvas, target) {
    const state = { dragging: false, pan: false, lx: 0, ly: 0, spherical: new THREE.Spherical() };
    state.spherical.setFromVector3(camera.position.clone().sub(target));

    function apply() {
      camera.position.setFromSpherical(state.spherical).add(target);
      camera.lookAt(target);
    }

    canvas.addEventListener("pointerdown", (e) => {
      state.dragging = true;
      state.pan = e.button === 2 || e.shiftKey;
      state.lx = e.clientX;
      state.ly = e.clientY;
      canvas.setPointerCapture(e.pointerId);
    });
    canvas.addEventListener("pointerup", () => {
      state.dragging = false;
    });
    canvas.addEventListener("pointermove", (e) => {
      if (!state.dragging) return;
      const dx = e.clientX - state.lx;
      const dy = e.clientY - state.ly;
      state.lx = e.clientX;
      state.ly = e.clientY;
      if (state.pan) {
        const panScale = 0.0025 * state.spherical.radius;
        const right = new THREE.Vector3();
        const up = new THREE.Vector3();
        camera.getWorldDirection(right);
        right.cross(camera.up).normalize();
        up.copy(camera.up).normalize();
        target.addScaledVector(right, -dx * panScale);
        target.addScaledVector(up, dy * panScale);
      } else {
        state.spherical.theta -= dx * 0.005;
        state.spherical.phi = THREE.MathUtils.clamp(state.spherical.phi - dy * 0.005, 0.12, Math.PI - 0.12);
      }
      apply();
    });
    canvas.addEventListener("wheel", (e) => {
      e.preventDefault();
      state.spherical.radius = THREE.MathUtils.clamp(state.spherical.radius * (1 + e.deltaY * 0.001), 0.8, 6);
      apply();
    }, { passive: false });
    canvas.addEventListener("contextmenu", (e) => e.preventDefault());
    apply();
    return { apply, target, spherical: state.spherical };
  }

  function createRobotScene(canvas) {
    const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.setClearColor(0x07090d, 1);

    const scene = new THREE.Scene();
    scene.fog = new THREE.Fog(0x07090d, 3.2, 8);

    const camera = new THREE.PerspectiveCamera(42, 1, 0.05, 40);
    camera.position.set(1.55, 1.15, 1.75);
    const target = new THREE.Vector3(0.2, 0.28, 0.05);
    camera.lookAt(target);

    scene.add(new THREE.HemisphereLight(0xb9c7d6, 0x1a1e24, 0.7));
    const key = new THREE.DirectionalLight(0xffffff, 1.15);
    key.position.set(1.4, 2.2, 1.1);
    key.castShadow = true;
    key.shadow.mapSize.set(1024, 1024);
    scene.add(key);
    const fill = new THREE.DirectionalLight(0x88aadd, 0.35);
    fill.position.set(-1.2, 1.0, -0.8);
    scene.add(fill);
    const rim = new THREE.PointLight(0x2ee6a6, 0.25, 6);
    rim.position.set(-0.4, 1.4, 0.8);
    scene.add(rim);

    const mats = createMaterials();
    const world = buildWorld(scene, mats);
    const robot = buildRobot(mats);
    robot.root.position.set(0, 0, 0);
    scene.add(robot.root);

    const orbit = attachOrbit(camera, canvas, target);

    let joints = HOME.slice();
    const goal = HOME.slice();
    let playing = false;
    let cycleT = 0;
    let last = performance.now();
    let frames = 0;
    let fps = 60;
    let fpsT = last;
    const listeners = [];

    function setJoints(next, immediate) {
      for (let i = 0; i < 6; i++) goal[i] = next[i];
      if (immediate) {
        joints = next.slice();
        applyJoints();
      }
    }

    function applyJoints() {
      robot.joints[0].rotation.y = joints[0];
      robot.joints[1].rotation.z = joints[1];
      robot.joints[2].rotation.z = joints[2];
      robot.joints[3].rotation.y = joints[3];
      robot.joints[4].rotation.z = joints[4];
      robot.joints[5].rotation.y = joints[5];
    }

    function cartesian() {
      const p = new THREE.Vector3();
      robot.tcp.getWorldPosition(p);
      return {
        x: p.x * 1000,
        y: p.y * 1000,
        z: p.z * 1000,
        rx: deg(joints[3]),
        ry: deg(joints[4]),
        rz: deg(joints[5]),
      };
    }

    function notify() {
      const state = {
        joints: joints.map(deg),
        cart: cartesian(),
        fps,
      };
      listeners.forEach((fn) => fn(state));
    }

    function resize() {
      const parent = canvas.parentElement;
      const w = parent.clientWidth || 800;
      const h = parent.clientHeight || 560;
      renderer.setSize(w, h, false);
      camera.aspect = w / Math.max(h, 1);
      camera.updateProjectionMatrix();
    }

    function tick(now) {
      const dt = Math.min(0.05, (now - last) / 1000);
      last = now;
      frames += 1;
      if (now - fpsT > 500) {
        fps = Math.round((frames * 1000) / (now - fpsT));
        frames = 0;
        fpsT = now;
      }

      if (playing) {
        cycleT += dt * 0.28;
        const a = (Math.sin(cycleT) + 1) / 2;
        const b = (Math.sin(cycleT + 1.7) + 1) / 2;
        for (let i = 0; i < 6; i++) {
          goal[i] = THREE.MathUtils.lerp(HOME[i], CAP[i], a * 0.65 + b * 0.35);
        }
        goal[1] = THREE.MathUtils.lerp(HOME[1], PICK[1], a);
      }

      let changed = false;
      for (let i = 0; i < 6; i++) {
        const next = THREE.MathUtils.damp(joints[i], goal[i], 6.5, dt);
        if (Math.abs(next - joints[i]) > 1e-5) changed = true;
        joints[i] = next;
      }
      applyJoints();
      if (changed || playing) notify();

      renderer.render(scene, camera);
      requestAnimationFrame(tick);
    }

    applyJoints();
    resize();
    window.addEventListener("resize", resize);
    requestAnimationFrame(tick);
    notify();

    return {
      HOME,
      CAP,
      PICK,
      setJoints,
      goHome() {
        playing = false;
        setJoints(HOME);
      },
      goCap() {
        playing = false;
        setJoints(CAP);
      },
      goPick() {
        playing = false;
        setJoints(PICK);
      },
      play() {
        playing = true;
      },
      pause() {
        playing = false;
      },
      reset() {
        playing = false;
        cycleT = 0;
        setJoints(HOME, true);
        notify();
      },
      step() {
        playing = false;
        setJoints(joints.map((v, i) => v + (CAP[i] - HOME[i]) * 0.18));
      },
      onUpdate(fn) {
        listeners.push(fn);
      },
      resize,
      getJoints: () => joints.slice(),
    };
  }

  global.createRobotScene = createRobotScene;
})(window);
