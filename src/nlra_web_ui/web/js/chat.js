// chat.js — NL chat interface via /nl_command service + orchestrator feedback
const ChatPage = (() => {
  // /nl_command blocks until the whole manipulation task finishes (the
  // orchestrator allows up to 300 s). The default rosbridge service timeout is
  // 5 s, so we pass an explicit long timeout on this call.
  const NL_SERVICE_TIMEOUT_S = 300;
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
    nlService = (request) => ROSConn.callServiceWithTimeout(
      '/nl_command', 'nlra_interfaces/srv/NLCommand', request, NL_SERVICE_TIMEOUT_S
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
      renderTrace(resp);
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

  // Show the complete LLM input (system prompt incl. live world state + user
  // text) and output (raw assistant reply / tool call) for each attempt round.
  function renderTrace(resp) {
    let trace;
    try {
      trace = JSON.parse(resp.llm_trace_json || '[]');
    } catch (e) {
      return;
    }
    if (!Array.isArray(trace) || trace.length === 0) return;

    const log = document.getElementById('chat-log');
    const wrap = document.createElement('div');
    wrap.className = 'llm-trace';

    const header = document.createElement('div');
    header.className = 'llm-trace-header';
    header.textContent = 'LLM input/output';
    wrap.appendChild(header);

    for (const entry of trace) {
      const input = document.createElement('details');
      input.className = 'llm-trace-round';
      input.open = true;
      const sIn = document.createElement('summary');
      sIn.textContent = `Round ${entry.round} — LLM input`;
      const preSys = document.createElement('pre');
      preSys.textContent = `[system]\n${entry.system || ''}`;
      const preUser = document.createElement('pre');
      preUser.textContent = `[user]\n${entry.user || ''}`;
      input.appendChild(sIn);
      input.appendChild(preSys);
      input.appendChild(preUser);
      wrap.appendChild(input);

      const output = document.createElement('details');
      output.className = 'llm-trace-round';
      output.open = true;
      const sOut = document.createElement('summary');
      sOut.textContent = `Round ${entry.round} — LLM output`;
      const preOut = document.createElement('pre');
      preOut.className = 'llm-response';
      preOut.textContent = entry.assistant
        ? JSON.stringify(entry.assistant, null, 2)
        : '(no LLM reply)';
      output.appendChild(sOut);
      output.appendChild(preOut);
      wrap.appendChild(output);
    }

    log.appendChild(wrap);
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
