// chat.js — NL chat interface via /nl_command service + orchestrator feedback
const ChatPage = (() => {
  let nlService = null;
  let feedbackSub = null;
  let lastGoalId = null;

  function init() {
    document.getElementById('btn-chat-send').addEventListener('click', sendCommand);
    document.getElementById('chat-input').addEventListener('keydown', (e) => {
      if (e.key === 'Enter' && !e.shiftKey) {
        e.preventDefault();
        sendCommand();
      }
    });
  }

  function setupServices() {
    if (nlService) return;
    nlService = ROSConn.makeServiceCaller(
      '/nl_command', 'nlra_interfaces/srv/NLCommand'
    );

    // Subscribe to orchestrator feedback for step-by-step updates
    feedbackSub = ROSConn.subscribe(
      '/orchestrator/execute_task/_action/feedback',
      'nlra_interfaces/action/ExecuteTask_FeedbackMessage',
      (fb) => {
        updateProgress(fb.feedback || fb);
      },
      200
    );
  }

  function sendCommand() {
    const input = document.getElementById('chat-input');
    const text = input.value.trim();
    if (!text) return;
    input.value = '';

    setupServices();

    addMessage('user', text);
    showFeedback('running', 'Sending to robot...');

    nlService({ text }).then((resp) => {
      if (resp.success) {
        if (resp.task) {
          addMessage('system', `Grounded: ${resp.task} ${resp.args_json}`);
        }
        if (resp.response) {
          addMessage('robot', resp.response);
          showFeedback('success', resp.response);
        } else if (!resp.task) {
          addMessage('robot', '(done, no message)');
          showFeedback('success', '(done, no message)');
        } else {
          showFeedback('success', 'Task complete');
        }
      } else {
        addMessage('error', resp.error || 'Unknown error');
        showFeedback('error', resp.error || 'Unknown error');
      }
    }).catch((err) => {
      addMessage('error', `Service call failed: ${err}`);
      showFeedback('error', `Service call failed: ${err}`);
    });
  }

  function updateProgress(fb) {
    if (!fb.step && fb.step_index === 0 && fb.step_count === 0) return;
    const pct = Math.round((fb.progress || 0) * 100);
    const step = fb.step || '';
    const idx = (fb.step_index || 0) + 1;
    const total = fb.step_count || '?';
    showFeedback('running', `[${idx}/${total}] ${step} (${pct}%)`);
  }

  function addMessage(type, text) {
    const log = document.getElementById('chat-log');
    const div = document.createElement('div');
    div.className = `chat-msg ${type}`;
    div.textContent = text;
    log.appendChild(div);
    log.scrollTop = log.scrollHeight;
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

  function onActivate() {
    setupServices();
    document.getElementById('chat-input').focus();
  }

  return { init, onActivate };
})();
