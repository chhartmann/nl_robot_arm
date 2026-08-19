// ros_connection.js — manages the roslibjs connection to rosbridge
const ROSConn = (() => {
  let ros = null;
  let connected = false;
  let handlersBound = false;
  let pendingUrl = null;
  let actionSeq = 0;
  const listeners = [];

  // Ensure a Ros instance always exists so helpers never operate on null
  function ensureRos() {
    if (!ros) ros = new ROSLIB.Ros();
    return ros;
  }

  // roslibjs buffers every op sent while disconnected and replays them all on
  // reconnect. Subscriptions must keep buffering so they come back after the
  // socket reopens, but service/action ops must NOT: they are stale by then
  // (e.g. minutes of world-model polls) and flushing the whole batch at once
  // stalls rosbridge - each call does a blocking DDS graph query - past its
  // 5 s per-call timeout, producing the "Timeout exceeded while waiting for
  // service response" flood. So service/action helpers reject immediately
  // instead of letting the op sit in roslibjs's buffer.
  function isReady() {
    return connected && ros && ros.isConnected;
  }

  function bindHandlers() {
    if (handlersBound) return;
    handlersBound = true;
    ros.on('connection', () => {
      connected = true;
      notify('connected');
    });
    ros.on('error', (err) => {
      console.error('ROS connection error:', err);
      notify('error', err);
    });
    ros.on('close', () => {
      connected = false;
      notify('disconnected');
      if (pendingUrl) setTimeout(() => connect(pendingUrl), 3000);
    });
  }

  function connect(url) {
    pendingUrl = url;
    ensureRos();
    bindHandlers();
    if (!ros.isConnected) {
      try {
        ros.connect(url);
      } catch (e) {
        console.warn('ROS connect failed:', e);
      }
    }
  }

  function notify(event, data) {
    for (const fn of listeners) fn(event, data);
  }

  return {
    connect,
    on(fn) { listeners.push(fn); },
    get ros() { return ensureRos(); },
    get isConnected() { return connected; },

    // Helper: create a subscriber
    subscribe(topic, type, callback, throttleMs) {
      const r = ensureRos();
      let lastTime = 0;
      const sub = new ROSLIB.Topic({ ros: r, name: topic, messageType: type });
      sub.subscribe((msg) => {
        const now = Date.now();
        if (throttleMs && now - lastTime < throttleMs) return;
        lastTime = now;
        callback(msg);
      });
      return { sub, unsubscribe: () => sub.unsubscribe() };
    },

    // Helper: call a service
    callService(name, type, request) {
      const r = ensureRos();
      return new Promise((resolve, reject) => {
        if (!isReady()) { reject(new Error('ROS not connected')); return; }
        const svc = new ROSLIB.Service({ ros: r, name, serviceType: type });
        const req = new ROSLIB.ServiceRequest(request);
        svc.callService(req, resolve, reject);
      });
    },

    // Helper: send an action goal and get result + feedback.
    // Uses rosbridge's native action ops (send_action_goal / action_feedback /
    // action_result), which is what ROS 2 rosbridge_server exposes. The
    // classic roslibjs ActionClient speaks the ROS 1 actionlib protocol and
    // cannot reach these action servers.
    //
    // NOTE: roslibjs never emits a generic 'message' event on the Ros object
    // (its SocketAdapter only emits topic / service / status events), so a
    // handler registered with r.on('message', ...) would never see the
    // action_result / action_feedback ops and the promise would never settle.
    // We therefore listen on the raw WebSocket for those ops instead.
    sendActionGoal(actionName, actionType, goalMsg, onFeedback) {
      const r = ensureRos();
      const cid = 'action:' + (++actionSeq);
      let socket = r.socket;
      let cancelled = false;

      const promise = new Promise((resolve, reject) => {
        if (!isReady()) { reject(new Error('ROS not connected')); return; }
        const handler = (event) => {
          let message;
          try {
            message = JSON.parse(event.data);
          } catch (e) {
            return;
          }
          if (!message || message.id !== cid) return;
          if (message.op === 'action_result') {
            if (socket && socket.removeEventListener) {
              socket.removeEventListener('message', handler);
            }
            if (cancelled) {
              // The goal was cancelled from the client side; treat it as a
              // clean stop rather than a failure so callers can distinguish
              // "released the jog key" from a real action error. This must be
              // checked before the result branch: rosbridge forwards the
              // cancelled goal's actual result message (success=false,
              // message="cancelled") with result:true.
              resolve({ cancelled: true });
            } else if (message.result === true && message.values) {
              resolve({ result: message.values, status: message.status });
            } else {
              reject(new Error(
                typeof message.values === 'string' ? message.values : 'action failed'));
            }
          } else if (message.op === 'action_feedback' && onFeedback) {
            onFeedback(message.values || {});
          }
        };

        const sendGoal = () => {
          r.callOnConnection({
            op: 'send_action_goal',
            action: actionName,
            action_type: actionType,
            args: goalMsg || {},
            id: cid,
            feedback: !!onFeedback,
          });
        };

        // The socket may not exist yet if the goal is sent before the
        // connection completes; wait for the connection so we never miss the
        // action response.
        if (!socket) {
          r.once('connection', () => {
            socket = r.socket;
            if (socket && socket.addEventListener) {
              socket.addEventListener('message', handler);
              sendGoal();
            } else {
              reject(new Error('WebSocket unavailable'));
            }
          });
        } else if (socket.addEventListener) {
          socket.addEventListener('message', handler);
          sendGoal();
        } else {
          reject(new Error('WebSocket unavailable'));
        }
      });

      // Cancel the in-progress goal. Safe to call even if the goal already
      // completed: the rosbridge server ignores cancels for unknown ids and
      // the pending result is handled by the `cancelled` flag above.
      promise.cancelAction = () => {
        cancelled = true;
        try {
          r.callOnConnection({
            op: 'cancel_action_goal',
            action: actionName,
            id: cid,
          });
        } catch (e) {
          // socket not ready; the goal will just complete on its own
        }
      };

      return promise;
    },

    // Helper: create a service caller (returns a callable)
    makeServiceCaller(name, type) {
      const r = ensureRos();
      return (request) => {
        return new Promise((resolve, reject) => {
          if (!isReady()) { reject(new Error('ROS not connected')); return; }
          const svc = new ROSLIB.Service({ ros: r, name, serviceType: type });
          const req = new ROSLIB.ServiceRequest(request);
          svc.callService(req, resolve, reject);
        });
      };
    },

    // Helper: call a service with a custom server-response timeout in seconds.
    // roslibjs's Service.callService cannot set the rosbridge per-call
    // `timeout` field, which defaults to 5 s. That is too short for blocking
    // services like /nl_command that wait for the whole manipulation task to
    // finish, so we send the raw call_service op and watch for the response
    // on the socket ourselves (same approach as sendActionGoal).
    callServiceWithTimeout(name, type, request, timeoutSec) {
      const r = ensureRos();
      return new Promise((resolve, reject) => {
        if (!isReady()) { reject(new Error('ROS not connected')); return; }
        const cid = 'service:' + (++actionSeq);
        let socket = r.socket;

        const handler = (event) => {
          let message;
          try {
            message = JSON.parse(event.data);
          } catch (e) {
            return;
          }
          if (!message || message.id !== cid) return;
          if (socket && socket.removeEventListener) {
            socket.removeEventListener('message', handler);
          }
          if (message.op === 'service_response' && message.result === true) {
            resolve(message.values);
          } else {
            reject(new Error(
              typeof message.values === 'string' ? message.values : 'service call failed'));
          }
        };

        const sendCall = () => {
          r.callOnConnection({
            op: 'call_service',
            service: name,
            type,
            args: request || {},
            id: cid,
            timeout: timeoutSec,
          });
        };

        if (!socket) {
          r.once('connection', () => {
            socket = r.socket;
            if (socket && socket.addEventListener) {
              socket.addEventListener('message', handler);
              sendCall();
            } else {
              reject(new Error('WebSocket unavailable'));
            }
          });
        } else if (socket.addEventListener) {
          socket.addEventListener('message', handler);
          sendCall();
        } else {
          reject(new Error('WebSocket unavailable'));
        }
      });
    },
  };
})();
