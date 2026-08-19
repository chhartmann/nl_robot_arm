// control.js — Manual joint control + gripper + 3D robot visualization
const ControlPage = (() => {
  const JOINTS = [
    { name: 'joint_1', alias: 'a1', label: 'A1', min: -170, max: 170, home: 0 },
    { name: 'joint_2', alias: 'a2', label: 'A2', min: -195, max: 55, home: -90 },
    { name: 'joint_3', alias: 'a3', label: 'A3', min: -115, max: 165, home: 86 },
    { name: 'joint_4', alias: 'a4', label: 'A4', min: -200, max: 200, home: 0 },
    { name: 'joint_5', alias: 'a5', label: 'A5', min: -120, max: 120, home: 0 },
    { name: 'joint_6', alias: 'a6', label: 'A6', min: -350, max: 350, home: 0 },
  ];

  const GRIPPER = { name: 'robotiq_85_left_knuckle_joint', min: 0, max: 0.8 };
  const AXIS_STEP_DEG = 5;
  const AXIS_REPEAT_DELAY_MS = 80;
  const AXIS_MOVE_DURATION_SEC = 0.5;
  const CARTESIAN_STEP_M = 0.01;
  const CARTESIAN_REPEAT_DELAY_MS = 80;
  const CARTESIAN_VELOCITY_SCALING = 0.3;
  // base_link is fixed to world in the simulation and provides world-aligned axes.
  const CARTESIAN_REFERENCE_FRAME = 'base_link';
  const CARTESIAN_AXES = [
    { name: 'x', label: 'X' },
    { name: 'y', label: 'Y' },
    { name: 'z', label: 'Z' },
  ];
  const WORLD_OBJECTS_POLL_MS = 500;
  // Keyboard jog bindings: X/Y/Z move the TCP in Cartesian space, 1-6 jog the
  // joints. Holding Shift reverses the direction. Letters are matched on the
  // typed character (event.key) so they follow the user's keyboard layout
  // (e.g. QWERTZ users hit the key labeled Z for the Z axis). Digits are
  // matched on the physical key position (event.code), which is the same
  // number row on every layout.
  const KEYBOARD_MAP = {
    x: { type: 'cartesian', axis: 'x' },
    y: { type: 'cartesian', axis: 'y' },
    z: { type: 'cartesian', axis: 'z' },
    Digit1: { type: 'axis', index: 0 },
    Digit2: { type: 'axis', index: 1 },
    Digit3: { type: 'axis', index: 2 },
    Digit4: { type: 'axis', index: 3 },
    Digit5: { type: 'axis', index: 4 },
    Digit6: { type: 'axis', index: 5 },
  };
  // Fill colors for the ghost objects, matched against the object id.
  const OBJECT_COLORS = {
    red: 0xff4444,
    green: 0x44cc66,
    yellow: 0xffcc33,
    orange: 0xff8833,
    blue: 0x4488ff,
  };

  let jointValues = {};
  let gripperValue = 0;
  let gripperTarget = 0;
  const axisMoves = new Map();
  const cartesianMoves = new Map();
  const activeKeyboardKeys = new Map();
  let keyboardBound = false;
  let robotModel = null;
  let robotScene = null;
  let robotLoadGeneration = 0;
  let scene, camera, renderer, controls;
  let jointStateSub = null;
  let worldObjectsGroup = null;
  const objectMeshes = {};
  let worldObjectsPollTimer = null;
  let getObjectsService = null;

  function init() {
    buildAxisControls();
    buildCartesianControls();
    bindButtons();
    try {
      initThreeJS();
    } catch (e) {
      console.warn('3D visualization unavailable:', e);
    }
  }

  function buildAxisControls() {
    const container = document.getElementById('joint-controls');
    // Keep the 3D view usable during a rolling update where an old index.html
    // is briefly paired with the new JavaScript bundle.
    if (!container) {
      console.warn('joint control container is missing');
      return;
    }
    container.innerHTML = '';
    for (const j of JOINTS) {
      const row = document.createElement('div');
      row.className = 'joint-row';
      row.innerHTML = `
        <button class="axis-button" type="button" data-direction="-1"
                aria-label="Decrease ${j.label}">−</button>
        <div class="joint-reading">
          <span class="axis-label">${j.label}</span>
          <span class="joint-val" id="jv-${j.name}">${j.home.toFixed(1)}°</span>
          <span class="joint-range">${j.min}° to ${j.max}°</span>
        </div>
        <button class="axis-button" type="button" data-direction="1"
                aria-label="Increase ${j.label}">+</button>
      `;
      container.appendChild(row);
      jointValues[j.name] = j.home;

      row.querySelectorAll('.axis-button').forEach((button) => {
        const direction = Number(button.dataset.direction);
        bindHoldButton(
          button,
          () => startAxisMove(j, direction, button),
          () => stopAxisMove(j.name)
        );
      });
    }
  }

  function buildCartesianControls() {
    const container = document.getElementById('cartesian-controls');
    if (!container) {
      console.warn('Cartesian control container is missing');
      return;
    }
    container.innerHTML = '';
    for (const axis of CARTESIAN_AXES) {
      const row = document.createElement('div');
      row.className = 'cartesian-row';
      row.innerHTML = `
        <button class="axis-button" type="button" data-direction="-1"
                aria-label="Decrease TCP ${axis.label}">-</button>
        <div class="cartesian-reading">
          <span class="cartesian-label">${axis.label}</span>
          <span class="cartesian-value" id="cartesian-value-${axis.name}">—</span>
          <span class="cartesian-step">1 cm / step</span>
        </div>
        <button class="axis-button" type="button" data-direction="1"
                aria-label="Increase TCP ${axis.label}">+</button>
      `;
      container.appendChild(row);

      row.querySelectorAll('.axis-button').forEach((button) => {
        const direction = Number(button.dataset.direction);
        bindHoldButton(
          button,
          () => startCartesianMove(axis, direction, button),
          () => stopCartesianMove(axis.name)
        );
      });
    }
  }

  function bindHoldButton(button, onStart, onStop) {
    button.addEventListener('pointerdown', (event) => {
      event.preventDefault();
      onStart();
    });
    button.addEventListener('pointerup', onStop);
    button.addEventListener('pointercancel', onStop);
    button.addEventListener('keydown', (event) => {
      if (event.key === ' ' || event.key === 'Enter') {
        event.preventDefault();
        onStart();
      }
    });
    button.addEventListener('keyup', (event) => {
      if (event.key === ' ' || event.key === 'Enter') {
        event.preventDefault();
        onStop();
      }
    });
  }

  function bindButtons() {
    document.getElementById('btn-home').addEventListener('click', sendHome);
    document.getElementById('btn-grasp').addEventListener('click', sendGrasp);
    document.getElementById('btn-release').addEventListener('click', sendRelease);

    const gs = document.getElementById('gripper-slider');
    gs.addEventListener('input', () => {
      document.getElementById('gripper-value').textContent =
        parseFloat(gs.value).toFixed(2) + ' rad';
      gripperTarget = parseFloat(gs.value);
    });

    window.addEventListener('pointerup', stopAllMoves);
    window.addEventListener('pointercancel', stopAllMoves);
    window.addEventListener('blur', stopAllMoves);
  }

  function startAxisMove(joint, direction, button) {
    if (axisMoves.has(joint.name)) return;
    const move = {
      joint,
      direction,
      button,
      pressed: true,
      inFlight: false,
      targetDegrees: { ...jointValues },
    };
    axisMoves.set(joint.name, move);
    if (button) button.classList.add('active');
    showFeedback('running', `Moving ${joint.label} ${direction > 0 ? '+' : '-'}`);
    sendNextAxisStep(move);
  }

  function stopAxisMove(jointName) {
    const move = axisMoves.get(jointName);
    if (!move) return;
    move.pressed = false;
    if (move.button) move.button.classList.remove('active');
    axisMoves.delete(jointName);
    cancelInFlightGoal(move);
  }

  function stopAllAxisMoves() {
    for (const name of axisMoves.keys()) stopAxisMove(name);
  }

  function startCartesianMove(axis, direction, button) {
    if (cartesianMoves.has(axis.name)) return;
    const move = { axis, direction, button, pressed: true, inFlight: false };
    cartesianMoves.set(axis.name, move);
    if (button) button.classList.add('active');
    showFeedback('running', `Moving TCP ${axis.label} ${direction > 0 ? '+' : '-'}`);
    sendNextCartesianStep(move);
  }

  function stopCartesianMove(axisName) {
    const move = cartesianMoves.get(axisName);
    if (!move) return;
    move.pressed = false;
    if (move.button) move.button.classList.remove('active');
    cartesianMoves.delete(axisName);
    cancelInFlightGoal(move);
  }

  function stopAllCartesianMoves() {
    for (const name of cartesianMoves.keys()) stopCartesianMove(name);
  }

  function stopAllMoves() {
    stopAllAxisMoves();
    stopAllCartesianMoves();
    stopAllKeyboardMoves();
  }

  function cancelInFlightGoal(move) {
    const goal = move.currentGoal;
    if (move.inFlight && goal && goal.cancelAction) {
      move.inFlight = false;
      goal.cancelAction();
    }
  }

  function isTypingTarget(target) {
    if (!target) return false;
    const tag = target.tagName;
    return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' ||
      target.isContentEditable;
  }

  function onKeyDown(event) {
    if (isTypingTarget(event.target)) return;
    const binding = KEYBOARD_MAP[event.code] ||
      KEYBOARD_MAP[event.key.toLowerCase()];
    if (!binding) return;
    if (activeKeyboardKeys.has(event.code)) return;
    event.preventDefault();
    const direction = event.shiftKey ? -1 : 1;
    activeKeyboardKeys.set(event.code, binding);
    if (binding.type === 'cartesian') {
      const axis = CARTESIAN_AXES.find(a => a.name === binding.axis);
      startCartesianMove(axis, direction, null);
    } else {
      startAxisMove(JOINTS[binding.index], direction, null);
    }
  }

  function onKeyUp(event) {
    if (!activeKeyboardKeys.has(event.code)) return;
    const binding = activeKeyboardKeys.get(event.code);
    event.preventDefault();
    activeKeyboardKeys.delete(event.code);
    if (binding.type === 'cartesian') {
      stopCartesianMove(binding.axis);
    } else {
      stopAxisMove(JOINTS[binding.index].name);
    }
  }

  function stopAllKeyboardMoves() {
    for (const [code, binding] of activeKeyboardKeys) {
      if (binding.type === 'cartesian') {
        stopCartesianMove(binding.axis);
      } else {
        stopAxisMove(JOINTS[binding.index].name);
      }
    }
    activeKeyboardKeys.clear();
  }

  function bindKeyboard() {
    if (keyboardBound) return;
    keyboardBound = true;
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);
  }

  function unbindKeyboard() {
    if (!keyboardBound) return;
    keyboardBound = false;
    window.removeEventListener('keydown', onKeyDown);
    window.removeEventListener('keyup', onKeyUp);
    stopAllKeyboardMoves();
  }

  function sendNextCartesianStep(move) {
    if (!move.pressed || move.inFlight) return;

    const translation = { x: 0, y: 0, z: 0 };
    translation[move.axis.name] = move.direction * CARTESIAN_STEP_M;
    move.inFlight = true;
    const goal = ROSConn.sendActionGoal(
      'skills/move_relative',
      'nlra_interfaces/action/MoveRelative',
      {
        translation,
        // A zero quaternion is the action's "no rotation" value.
        rotation_delta: { x: 0, y: 0, z: 0, w: 0 },
        reference_frame: CARTESIAN_REFERENCE_FRAME,
        velocity_scaling: CARTESIAN_VELOCITY_SCALING,
        acceleration_scaling: CARTESIAN_VELOCITY_SCALING,
      }
    );
    move.currentGoal = goal;
    goal.then((res) => {
      move.inFlight = false;
      const r = res.result || res;
      if (r.cancelled) return;
      if (!r.success) {
        stopCartesianMove(move.axis.name);
        showFeedback('error', `Failed to move TCP ${move.axis.label}: ${r.message}`);
        return;
      }
      if (move.pressed) {
        setTimeout(() => sendNextCartesianStep(move), CARTESIAN_REPEAT_DELAY_MS);
      }
    }).catch((err) => {
      move.inFlight = false;
      stopCartesianMove(move.axis.name);
      showFeedback('error', `Cartesian action error: ${err}`);
    });
  }

  function sendNextAxisStep(move) {
    if (!move.pressed || move.inFlight) return;

    const current = move.targetDegrees[move.joint.name];
    const remaining = move.direction < 0
      ? current - move.joint.min
      : move.joint.max - current;
    if (remaining <= 0) {
      stopAxisMove(move.joint.name);
      showFeedback('error', `${move.joint.label} is at its limit`);
      return;
    }
    const stepDeg = Math.min(AXIS_STEP_DEG, remaining);
    const nextTarget = {
      ...move.targetDegrees,
      [move.joint.name]: current + move.direction * stepDeg,
    };

    move.inFlight = true;
    const goal = ROSConn.sendActionGoal(
      'skills/move_joints',
      'nlra_interfaces/action/MoveJoints',
      {
        positions: JOINTS.map(j => nextTarget[j.name] * Math.PI / 180),
        duration: AXIS_MOVE_DURATION_SEC,
      }
    );
    move.currentGoal = goal;
    goal.then((res) => {
      move.inFlight = false;
      const r = res.result || res;
      if (r.cancelled) return;
      if (!r.success) {
        stopAxisMove(move.joint.name);
        showFeedback('error', `Failed to move ${move.joint.label}: ${r.message}`);
        return;
      }
      move.targetDegrees = nextTarget;
      if (move.pressed) setTimeout(() => sendNextAxisStep(move), AXIS_REPEAT_DELAY_MS);
    }).catch((err) => {
      move.inFlight = false;
      stopAxisMove(move.joint.name);
      showFeedback('error', `Action error: ${err}`);
    });
  }

  function sendHome() {
    showFeedback('running', 'Returning to home pose...');
    ROSConn.sendActionGoal(
      'skills/home',
      'nlra_interfaces/action/Home',
      { open_gripper: true },
      (fb) => {
        if (fb.phase) showFeedback('running', `Home: ${fb.phase}`);
      }
    ).then((res) => {
      const r = res.result || res;
      showFeedback(r.success ? 'success' : 'error',
        r.success ? 'At home' : `Failed: ${r.message}`);
    }).catch((err) => showFeedback('error', `Home error: ${err}`));
  }

  function sendGrasp() {
    const pos = gripperTarget;
    showFeedback('running', `Grasping at ${pos.toFixed(2)} rad...`);
    ROSConn.sendActionGoal(
      'skills/grasp',
      'nlra_interfaces/action/Grasp',
      { position: pos, max_effort: 50.0 },
      (fb) => {
        if (fb.phase) showFeedback('running', `Grasp: ${fb.phase}`);
      }
    ).then((res) => {
      const r = res.result || res;
      showFeedback(r.success ? 'success' : 'error',
        r.object_detected ? 'Object gripped' : (r.message || 'Grasp done'));
    }).catch((err) => showFeedback('error', `Grasp error: ${err}`));
  }

  function sendRelease() {
    showFeedback('running', 'Opening gripper...');
    ROSConn.sendActionGoal(
      'skills/release',
      'nlra_interfaces/action/Release',
      {},
      (fb) => {
        if (fb.phase) showFeedback('running', `Release: ${fb.phase}`);
      }
    ).then((res) => {
      const r = res.result || res;
      showFeedback(r.success ? 'success' : 'error',
        r.success ? 'Gripper open' : `Failed: ${r.message}`);
    }).catch((err) => showFeedback('error', `Release error: ${err}`));
  }

  function showFeedback(type, msg) {
    const bar = document.getElementById('control-feedback');
    bar.className = `feedback-bar ${type}`;
    bar.textContent = msg;
    bar.classList.remove('hidden');
    if (type === 'success' || type === 'error') {
      setTimeout(() => bar.classList.add('hidden'), 4000);
    }
  }

  // ── Three.js 3D visualization ──
  function initThreeJS() {
    if (scene) return;

    const container = document.getElementById('robot-viz');
    const w = container.clientWidth || 600;
    const h = container.clientHeight || 400;

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1d27);

    camera = new THREE.PerspectiveCamera(50, w / h, 0.01, 10);
    camera.position.set(1.7891967177342514, 1.003632497242595, -0.4837088630458563);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(w, h);
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.target.set(-0.2452312924615365, 0.6090540484045234, 0.1862775545761634);
    controls.update();

    // Lights
    scene.add(new THREE.AmbientLight(0xffffff, 0.6));
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(1, 2, 3);
    scene.add(dirLight);

    // Grid + axes
    const grid = new THREE.GridHelper(2, 20, 0x444444, 0x222222);
    scene.add(grid);
    const axes = new THREE.AxesHelper(0.2);
    scene.add(axes);

    // Keep the real and fallback models in one replaceable container.
    robotScene = new THREE.Group();
    robotScene.name = 'robot-visualization';
    scene.add(robotScene);

    // Ghost overlay for world-model objects. Kept out of robotScene, which is
    // cleared whenever the robot model reloads. The group carries the same
    // Z-up -> Y-up rotation as the robot model, so ROS world-frame poses can
    // be applied to its children verbatim.
    worldObjectsGroup = new THREE.Group();
    worldObjectsGroup.name = 'world-objects';
    worldObjectsGroup.rotation.x = -Math.PI / 2;
    scene.add(worldObjectsGroup);

    // Show a model immediately. The real URDF and meshes load asynchronously;
    // leaving the group empty makes a slow or failed mesh request look like a
    // broken 3D view.
    buildFallbackRobot(robotLoadGeneration);
    loadURDF();

    window.addEventListener('resize', onResize);
    animate();
  }

  function loadURDF() {
    const generation = ++robotLoadGeneration;
    let pendingMeshes = 0;
    let meshFailures = 0;
    let parsedRobot = null;
    let parsed = false;

    const installParsedRobot = () => {
      if (!parsed || pendingMeshes !== 0 || generation !== robotLoadGeneration) return;
      if (meshFailures > 0) {
        console.warn(`URDF mesh loading failed for ${meshFailures} mesh(es)`);
        buildFallbackRobot(generation);
        return;
      }
      replaceRobotModel(parsedRobot);
      // URDF frames are Z-up; stand the model upright in three.js's Y-up
      // scene (a +90 deg X rotation would map the arm's +Z to -Y).
      robotModel.rotation.x = -Math.PI / 2;
      applyJointAngles();
      updateCartesianPosition();
    };

    let loader;
    try {
      loader = new URDFLoader();
    } catch (e) {
      console.warn('URDFLoader unavailable, using fallback visualization:', e);
      buildFallbackRobot(generation);
      return;
    }

    // Map the package:// mesh URIs in the served URDF to HTTP paths the web
    // server exposes under /meshes/.
    loader.packages = {
      agilus: '/meshes/agilus',
      robotiq: '/meshes/robotiq',
    };

    // Load meshes ourselves so that (a) the three.js loaders are available and
    // (b) we undo THREE.ColladaLoader's automatic Z_UP -> Y_UP rotation of the
    // Collada assets. The URDF joint frames are already Z_UP; without undoing
    // that rotation every mesh would be tilted 90 degrees off its joint axis.
    loader.loadMeshCb = (path, manager, done) => {
      pendingMeshes++;
      const finish = (error) => {
        if (error) meshFailures++;
        pendingMeshes--;
        installParsedRobot();
      };
      if (/\.dae$/i.test(path)) {
        const loader3 = new THREE.ColladaLoader(manager);
        loader3.load(path, (dae) => {
          dae.scene.quaternion.identity();
          done(dae.scene);
          finish();
        }, undefined, (err) => {
          done(null, err);
          finish(err);
        });
      } else if (/\.stl$/i.test(path)) {
        const stlLoader = new THREE.STLLoader(manager);
        stlLoader.load(path, (geometry) => {
          done(new THREE.Mesh(geometry, new THREE.MeshPhongMaterial()));
          finish();
        }, undefined, (err) => {
          done(null, err);
          finish(err);
        });
      } else {
        const error = new Error('No loader for ' + path);
        done(null, error);
        finish(error);
      }
    };

    // Load the plain URDF served by the web server (the source xacro cannot be
    // parsed by URDFLoader).
    loader.load(
      '/models/agilus_robotiq.urdf',
      (robot) => {
        if (generation !== robotLoadGeneration) return;
        parsedRobot = robot;
        parsed = true;
        installParsedRobot();
      },
      (err) => {
        if (generation !== robotLoadGeneration) return;
        console.warn('URDF load failed, using fallback visualization:', err);
        buildFallbackRobot(generation);
      }
    );
  }

  function replaceRobotModel(model) {
    if (!robotScene) return;
    robotScene.clear();
    robotModel = model;
    robotScene.add(model);
  }

  function buildFallbackRobot(generation) {
    if (generation !== robotLoadGeneration || !robotScene) return;

    const group = new THREE.Group();

    // Base
    const baseMat = new THREE.MeshPhongMaterial({ color: 0x444444 });
    const base = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.08, 0.04, 32), baseMat);
    base.position.y = 0.02;
    group.add(base);

    // Joint links as colored cylinders
    const colors = [0x3366ff, 0x33cc66, 0xff6633, 0xcc33ff, 0xffcc33, 0x33cccc];
    const lengths = [0.15, 0.2, 0.2, 0.1, 0.08, 0.05];
    const joints = [];

    let prev = base;
    let y = 0.04;
    for (let i = 0; i < 6; i++) {
      const link = new THREE.Mesh(
        new THREE.CylinderGeometry(0.02, 0.02, lengths[i], 16),
        new THREE.MeshPhongMaterial({ color: colors[i] })
      );
      link.position.y = y + lengths[i] / 2;
      group.add(link);

      const sphere = new THREE.Mesh(
        new THREE.SphereGeometry(0.025, 16, 16),
        new THREE.MeshPhongMaterial({ color: 0x888888 })
      );
      sphere.position.y = y;
      group.add(sphere);

      joints.push({ link, sphere, length: lengths[i], baseY: y });
      y += lengths[i] + 0.02;
    }

    // Gripper
    const gripGroup = new THREE.Group();
    gripGroup.position.y = y;
    const fingerMat = new THREE.MeshPhongMaterial({ color: 0xaaaaaa });
    const finger1 = new THREE.Mesh(new THREE.BoxGeometry(0.008, 0.05, 0.015), fingerMat);
    finger1.position.set(0.012, 0.025, 0);
    const finger2 = new THREE.Mesh(new THREE.BoxGeometry(0.008, 0.05, 0.015), fingerMat);
    finger2.position.set(-0.012, 0.025, 0);
    gripGroup.add(finger1, finger2);
    group.add(gripGroup);

    replaceRobotModel(group);
    robotModel = { _fallback: true, joints, gripGroup, finger1, finger2 };
  }

  function updateRobotModel() {
    if (!robotModel) return;
    if (robotModel._fallback) {
      updateFallbackRobot();
    } else {
      applyJointAngles();
    }
    updateCartesianPosition();
  }

  function updateCartesianPosition() {
    if (!robotModel || robotModel._fallback || !robotModel.links ||
        !robotModel.links.tool0) return;

    const position = new THREE.Vector3();
    robotModel.updateMatrixWorld(true);
    robotModel.links.tool0.getWorldPosition(position);
    // The URDF model is rotated from Z-up into Three.js Y-up at the root.
    const coordinates = { x: position.x, y: -position.z, z: position.y };
    for (const axis of CARTESIAN_AXES) {
      const value = document.getElementById(`cartesian-value-${axis.name}`);
      if (value) value.textContent = `${coordinates[axis.name].toFixed(3)} m`;
    }
  }

  function applyJointAngles() {
    if (!robotModel || !robotModel.links) return;
    for (const j of JOINTS) {
      const deg = jointValues[j.name];
      const rad = deg * Math.PI / 180;
      if (robotModel.joints && robotModel.joints[j.name]) {
        robotModel.joints[j.name].setJointValue(rad);
      }
    }
    const g = robotModel.joints && robotModel.joints[GRIPPER.name];
    if (g) {
      g.setJointValue(gripperValue);
    }
  }

  function updateFallbackRobot() {
    if (!robotModel || !robotModel.joints) return;
    const angles = JOINTS.map(j => jointValues[j.name] * Math.PI / 180);

    // Simple kinematic chain: each joint rotates around Z axis
    for (let i = 0; i < robotModel.joints.length; i++) {
      const { sphere, link, baseY, length } = robotModel.joints[i];
      const angle = angles[i];

      // Accumulated rotation
      const euler = new THREE.Euler(0, 0, 0, 'XYZ');
      for (let k = 0; k <= i; k++) {
        euler.set(0, angles[k] * (i === k ? 1 : 0), 0, 'XYZ');
      }

      // Simple vertical stacking with rotation offset
      const offset = Math.sin(angle) * length * 0.3;
      link.position.x = offset * (i + 1) / 6;
      link.rotation.z = angle * 0.3;
    }

    // Gripper open/close
    const gap = 0.012 + gripperValue * 0.015;
    if (robotModel.finger1) robotModel.finger1.position.x = gap;
    if (robotModel.finger2) robotModel.finger2.position.x = -gap;
  }

  function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  }

  function onResize() {
    if (!renderer || !camera) return;
    const container = document.getElementById('robot-viz');
    if (!container) return;
    const w = container.clientWidth;
    const h = container.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  }

  window.getRobotCameraPose = function () {
    if (!camera || !controls) return null;
    return {
      position: camera.position.toArray(),
      target: controls.target.toArray(),
    };
  };

  // ── Joint state subscription (the live robot state drives the controls and model) ──
  function startJointStateSubscription() {
    if (jointStateSub) return;
    jointStateSub = ROSConn.subscribe(
      '/joint_states', 'sensor_msgs/JointState',
      (msg) => {
        for (let i = 0; i < msg.name.length; i++) {
          const name = msg.name[i];
          const j = JOINTS.find(j => j.name === name);
          if (j) {
            const deg = msg.position[i] * 180 / Math.PI;
            jointValues[name] = deg;
            document.getElementById(`jv-${name}`).textContent = deg.toFixed(1) + '°';
          }
          if (name === GRIPPER.name) {
            gripperValue = msg.position[i];
          }
        }
        updateRobotModel();
      },
      100  // throttle to 10 Hz
    );
  }

  // ── World-model object overlay (ghost meshes from ground-truth poses) ──
  function colorForObject(id) {
    for (const name of Object.keys(OBJECT_COLORS)) {
      if (id.includes(name)) return OBJECT_COLORS[name];
    }
    return 0x999999;
  }

  function buildObjectGeometry(obj) {
    const sx = obj.size[0];
    const sy = obj.size[1];
    const sz = obj.size[2];
    if (obj.kind === 'cylinder') {
      // CylinderGeometry is Y-axis aligned; rotate it onto the local (ROS)
      // Z axis so the group rotation stands it upright in the scene.
      const geo = new THREE.CylinderGeometry(sx / 2, sx / 2, sz, 24);
      geo.rotateX(Math.PI / 2);
      return geo;
    }
    return new THREE.BoxGeometry(sx, sy, sz);
  }

  function buildObjectMesh(obj) {
    const geometry = buildObjectGeometry(obj);
    const color = colorForObject(obj.id);
    const group = new THREE.Group();
    group.name = `world-object-${obj.id}`;
    if (obj.graspable) {
      group.add(new THREE.Mesh(geometry, new THREE.MeshPhongMaterial({
        color, transparent: true, opacity: 0.35,
      })));
    }
    // Wireframe edges for all objects (the only representation for static,
    // non-graspable ones like the table and tray, so they don't occlude).
    group.add(new THREE.LineSegments(
      new THREE.EdgesGeometry(geometry),
      new THREE.LineBasicMaterial({ color })
    ));
    return group;
  }

  function disposeObjectMesh(group) {
    group.traverse((child) => {
      if (child.geometry) child.geometry.dispose();
      if (child.material) child.material.dispose();
    });
  }

  function updateWorldObjects(objects) {
    if (!worldObjectsGroup) return;
    const seen = new Set();
    for (const obj of objects) {
      if (!obj.pose || !obj.pose.pose || !obj.size) continue;
      seen.add(obj.id);
      let entry = objectMeshes[obj.id];
      if (!entry) {
        entry = buildObjectMesh(obj);
        objectMeshes[obj.id] = entry;
        worldObjectsGroup.add(entry);
      }
      const p = obj.pose.pose.position;
      const q = obj.pose.pose.orientation;
      entry.position.set(p.x, p.y, p.z);
      entry.quaternion.set(q.x, q.y, q.z, q.w);
    }
    for (const id of Object.keys(objectMeshes)) {
      if (!seen.has(id)) {
        worldObjectsGroup.remove(objectMeshes[id]);
        disposeObjectMesh(objectMeshes[id]);
        delete objectMeshes[id];
      }
    }
  }

  function startWorldObjectsPolling() {
    if (worldObjectsPollTimer) return;
    if (!getObjectsService) {
      getObjectsService = ROSConn.makeServiceCaller(
        '/world_model/get_objects', 'nlra_interfaces/srv/GetObjects'
      );
    }
    const poll = () => {
      getObjectsService({ kind_filter: '' })
        .then((resp) => updateWorldObjects(resp.objects || []))
        .catch(() => {});  // world model not up yet; try again next tick
    };
    poll();
    worldObjectsPollTimer = setInterval(poll, WORLD_OBJECTS_POLL_MS);
  }

  function onActivate() {
    startJointStateSubscription();
    startWorldObjectsPolling();
    bindKeyboard();
    onResize();
  }

  function onDeactivate() {
    stopAllMoves();
    unbindKeyboard();
  }

  return { init, onActivate, onDeactivate };
})();
