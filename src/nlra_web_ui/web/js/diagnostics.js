// diagnostics.js — Joint states, controller status, planner status, world model, gripper
const DiagnosticsPage = (() => {
  const ARM_JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'];
  let subs = [];
  let pollTimer = null;
  let getObjects = null;
  let getControllerState = null;

  function init() {}

  function onActivate() {
    if (subs.length > 0) return; // already subscribed
    startSubscriptions();
    startPolling();
  }

  function onDeactivate() {
    // Keep subscriptions alive but stop polling
  }

  function startSubscriptions() {
    // Joint states
    subs.push(ROSConn.subscribe(
      '/joint_states', 'sensor_msgs/JointState', updateJointStates, 200
    ));

    // Gripper sensor (contact sensors from Gazebo)
    subs.push(ROSConn.subscribe(
      '/gripper_contact', 'sensor_msgs/JointState',
      updateGripperSensor, 500
    ));

    // Try to get controller state via service polling
    getObjects = ROSConn.makeServiceCaller(
      '/world_model/get_objects', 'nlra_interfaces/srv/GetObjects'
    );
  }

  function startPolling() {
    pollWorldModel();
    pollControllerState();
    pollTimer = setInterval(() => {
      pollWorldModel();
      pollControllerState();
    }, 2000);
  }

  function updateJointStates(msg) {
    const tbody = document.querySelector('#diag-joints tbody');
    tbody.innerHTML = '';
    for (let i = 0; i < msg.name.length; i++) {
      const name = msg.name[i];
      const pos = msg.position[i] !== undefined ? msg.position[i].toFixed(4) : '—';
      const vel = msg.velocity[i] !== undefined ? msg.velocity[i].toFixed(4) : '—';
      const tr = document.createElement('tr');
      tr.innerHTML = `<td>${name}</td><td>${pos}</td><td>${vel}</td>`;
      tbody.appendChild(tr);
    }

    // Update gripper sensor display from joint states
    const knuckleIdx = msg.name.indexOf('robotiq_85_left_knuckle_joint');
    if (knuckleIdx >= 0) {
      const el = document.getElementById('diag-gripper');
      const pos = msg.position[knuckleIdx];
      const vel = msg.velocity[knuckleIdx];
      el.innerHTML = `
        Knuckle position: <strong>${pos.toFixed(4)} rad</strong><br>
        Knuckle velocity: <strong>${vel.toFixed(4)} rad/s</strong>
      `;
    }
  }

  function updateGripperSensor(msg) {
    // If we get contact sensor data, show it
    const el = document.getElementById('diag-gripper');
    if (msg.name && msg.position) {
      let html = '';
      for (let i = 0; i < msg.name.length; i++) {
        html += `${msg.name[i]}: <strong>${msg.position[i].toFixed(4)}</strong><br>`;
      }
      el.innerHTML += html;
    }
  }

  function pollWorldModel() {
    if (!getObjects) return;
    getObjects({ kind_filter: '' }).then((resp) => {
      const tbody = document.querySelector('#diag-objects tbody');
      tbody.innerHTML = '';
      if (resp.objects && resp.objects.length > 0) {
        for (const obj of resp.objects) {
          const p = obj.pose.pose.position;
          const tr = document.createElement('tr');
          tr.innerHTML = `
            <td>${obj.id}</td>
            <td>${obj.kind}</td>
            <td>(${p.x.toFixed(3)}, ${p.y.toFixed(3)}, ${p.z.toFixed(3)})</td>
            <td>${obj.graspable ? '✓' : '—'}</td>
          `;
          tbody.appendChild(tr);
        }
      } else {
        tbody.innerHTML = '<tr><td colspan="4" style="color:var(--text-dim)">No objects</td></tr>';
      }
    }).catch(() => {});
  }

  function pollControllerState() {
    // Try to query controller manager via list_controllers service
    if (!ROSConn.isConnected) return;

    const callControllerList = ROSConn.makeServiceCaller(
      '/controller_manager/list_controllers',
      'controller_manager_msgs/srv/ListControllers'
    );

    callControllerList({}).then((resp) => {
      const el = document.getElementById('diag-controllers');
      if (resp.controller && resp.controller.length > 0) {
        let html = '';
        for (const c of resp.controller) {
          const stateClass = c.state === 'active' ? 'ok' : 'warn';
          html += `
            <div style="margin-bottom:0.4rem">
              <strong>${c.name}</strong>
              <span class="badge ${stateClass}">${c.state}</span>
              <span style="color:var(--text-dim);font-size:0.8rem">${c.type}</span>
            </div>
          `;
        }
        el.innerHTML = html;
      } else {
        el.innerHTML = '<span class="badge warn">No controllers found</span>';
      }
    }).catch(() => {
      document.getElementById('diag-controllers').innerHTML =
        '<span class="badge error">Controller manager unavailable</span>';
    });

    // Query motion planner state. The request of moveit_msgs/QueryPlannerInterfaces
    // is empty (planner_ids is a response field), so an empty request must be
    // sent or rosbridge raises NonexistentFieldException.
    const callPlannerState = ROSConn.makeServiceCaller(
      '/query_planner_interface',
      'moveit_msgs/srv/QueryPlannerInterfaces'
    );
    callPlannerState({}).then(() => {
      document.getElementById('diag-planner').innerHTML =
        '<span class="badge ok">MoveIt move_group available</span>';
    }).catch(() => {
      document.getElementById('diag-planner').innerHTML =
        '<span class="badge warn">MoveIt move_group not responding</span>';
    });
  }

  return { init, onActivate, onDeactivate };
})();
