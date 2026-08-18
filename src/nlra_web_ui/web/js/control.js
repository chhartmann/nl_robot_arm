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

  let sliders = {};
  let sliderValues = {};
  let touched = {};
  let gripperTouched = false;
  let robotModel = null;
  let robotScene = null;
  let robotLoadGeneration = 0;
  let scene, camera, renderer, controls;
  let jointStateSub = null;

  function init() {
    buildSliders();
    bindButtons();
    try {
      initThreeJS();
    } catch (e) {
      console.warn('3D visualization unavailable:', e);
    }
  }

  function buildSliders() {
    const container = document.getElementById('joint-sliders');
    container.innerHTML = '';
    for (const j of JOINTS) {
      const row = document.createElement('div');
      row.className = 'joint-row';
      row.innerHTML = `
        <label>${j.label}</label>
        <input type="range" min="${j.min}" max="${j.max}" step="0.5" value="${j.home}"
               data-joint="${j.name}" data-alias="${j.alias}">
        <span class="joint-val" id="jv-${j.name}">${j.home.toFixed(1)}°</span>
      `;
      container.appendChild(row);
      const slider = row.querySelector('input');
      slider.addEventListener('input', () => {
        const deg = parseFloat(slider.value);
        document.getElementById(`jv-${j.name}`).textContent = deg.toFixed(1) + '°';
        sliderValues[j.name] = deg;
        touched[j.name] = true;
        updateRobotModel();
      });
      sliders[j.name] = slider;
      sliderValues[j.name] = j.home;
    }
  }

  function bindButtons() {
    document.getElementById('btn-set-joints').addEventListener('click', sendJoints);
    document.getElementById('btn-home').addEventListener('click', sendHome);
    document.getElementById('btn-grasp').addEventListener('click', sendGrasp);
    document.getElementById('btn-release').addEventListener('click', sendRelease);

    const gs = document.getElementById('gripper-slider');
    gs.addEventListener('input', () => {
      document.getElementById('gripper-value').textContent =
        parseFloat(gs.value).toFixed(2) + ' rad';
      gripperTouched = true;
      updateRobotModel();
    });
  }

  function sendJoints() {
    const joints = {};
    for (const j of JOINTS) {
      joints[j.alias] = sliderValues[j.name];
    }
    showFeedback('running', `Setting joints: ${JSON.stringify(joints)}`);

    ROSConn.sendActionGoal(
      'skills/move_joints',
      'nlra_interfaces/action/MoveJoints',
      { positions: JOINTS.map(j => sliderValues[j.name] * Math.PI / 180), duration: 5.0 },
      (fb) => {
        if (fb.phase) showFeedback('running', `MoveJoints: ${fb.phase}`);
      }
    ).then((res) => {
      const r = res.result || res;
      if (r.success) {
        touched = {};
        gripperTouched = false;
      }
      showFeedback(r.success ? 'success' : 'error',
        r.success ? 'Joints set successfully' : `Failed: ${r.message}`);
    }).catch((err) => {
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
      if (r.success) setSliderFromHome();
    }).catch((err) => showFeedback('error', `Home error: ${err}`));
  }

  function sendGrasp() {
    const pos = parseFloat(document.getElementById('gripper-slider').value);
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
      if (r.success) gripperTouched = false;
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
      if (r.success) gripperTouched = false;
      showFeedback(r.success ? 'success' : 'error',
        r.success ? 'Gripper open' : `Failed: ${r.message}`);
    }).catch((err) => showFeedback('error', `Release error: ${err}`));
  }

  function setSliderFromHome() {
    for (const j of JOINTS) {
      sliders[j.name].value = j.home;
      sliderValues[j.name] = j.home;
      document.getElementById(`jv-${j.name}`).textContent = j.home.toFixed(1) + '°';
    }
    document.getElementById('gripper-slider').value = 0;
    document.getElementById('gripper-value').textContent = '0.00 rad';
    touched = {};
    gripperTouched = false;
    updateRobotModel();
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

    // Load URDF
    loadURDF();

    window.addEventListener('resize', onResize);
    animate();
  }

  function loadURDF() {
    const generation = ++robotLoadGeneration;

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
      if (/\.dae$/i.test(path)) {
        const loader3 = new THREE.ColladaLoader(manager);
        loader3.load(path, (dae) => {
          dae.scene.quaternion.identity();
          done(dae.scene);
        }, undefined, (err) => done(null, err));
      } else if (/\.stl$/i.test(path)) {
        const stlLoader = new THREE.STLLoader(manager);
        stlLoader.load(path, (geometry) => {
          done(new THREE.Mesh(geometry, new THREE.MeshPhongMaterial()));
        }, undefined, (err) => done(null, err));
      } else {
        done(null, new Error('No loader for ' + path));
      }
    };

    // Load the plain URDF served by the web server (the source xacro cannot be
    // parsed by URDFLoader).
    loader.load(
      '/models/agilus_robotiq.urdf',
      (robot) => {
        if (generation !== robotLoadGeneration) return;
        replaceRobotModel(robot);
        // URDF frames are Z-up; stand the model upright in three.js's Y-up
        // scene (a +90 deg X rotation would map the arm's +Z to -Y).
        robotModel.rotation.x = -Math.PI / 2;
        applyJointAngles();
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
  }

  function applyJointAngles() {
    if (!robotModel || !robotModel.links) return;
    for (const j of JOINTS) {
      const deg = sliderValues[j.name];
      const rad = deg * Math.PI / 180;
      if (robotModel.joints && robotModel.joints[j.name]) {
        robotModel.joints[j.name].setJointValue(rad);
      }
    }
    const g = robotModel.joints && robotModel.joints[GRIPPER.name];
    if (g) {
      g.setJointValue(parseFloat(document.getElementById('gripper-slider').value));
    }
  }

  function updateFallbackRobot() {
    if (!robotModel || !robotModel.joints) return;
    const angles = JOINTS.map(j => sliderValues[j.name] * Math.PI / 180);

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
    const gripperVal = parseFloat(document.getElementById('gripper-slider').value);
    const gap = 0.012 + gripperVal * 0.015;
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

  // ── Joint state subscription (updates sliders from live state) ──
  function startJointStateSubscription() {
    if (jointStateSub) return;
    jointStateSub = ROSConn.subscribe(
      '/joint_states', 'sensor_msgs/JointState',
      (msg) => {
        for (let i = 0; i < msg.name.length; i++) {
          const name = msg.name[i];
          const j = JOINTS.find(j => j.name === name);
          if (j && !touched[name]) {
            const deg = msg.position[i] * 180 / Math.PI;
            sliders[name].value = deg;
            sliderValues[name] = deg;
            document.getElementById(`jv-${name}`).textContent = deg.toFixed(1) + '°';
          }
          if (name === GRIPPER.name && !gripperTouched) {
            document.getElementById('gripper-slider').value = msg.position[i];
            document.getElementById('gripper-value').textContent =
              msg.position[i].toFixed(2) + ' rad';
          }
        }
        updateRobotModel();
      },
      100  // throttle to 10 Hz
    );
  }

  function onActivate() {
    startJointStateSubscription();
    onResize();
  }

  return { init, onActivate };
})();
