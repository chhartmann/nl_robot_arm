// app.js — Main entry: tab switching, ROS connection, page lifecycle
(function() {
  const pages = {
    control: ControlPage,
    diagnostics: DiagnosticsPage,
  };

  let activePage = 'control';

  function normalizeChatLayout() {
    // Migrate a page cached from the previous release in place. This prevents
    // a stale index.html from bringing back a separate chat tab after the JS
    // bundle has already been updated.
    document.querySelector('.tab[data-page="chat"]')?.remove();
    const legacyPage = document.getElementById('page-chat');
    if (!legacyPage) return;

    const layout = legacyPage.querySelector('.chat-layout');
    const controlLayout = document.querySelector('#page-control .control-layout');
    if (!layout || !controlLayout) return;

    const panel = document.createElement('div');
    panel.className = 'chat-panel';
    const heading = document.createElement('h2');
    heading.textContent = 'Natural Language';
    panel.append(heading, layout);
    controlLayout.appendChild(panel);
    legacyPage.remove();
  }

  function init() {
    normalizeChatLayout();

    // Tab switching
    document.querySelectorAll('.tab').forEach((tab) => {
      tab.addEventListener('click', () => switchPage(tab.dataset.page));
    });

    // Connect to rosbridge first so a page-initialization failure can never
    // block the ROS connection
    connectToROS();

    // Initialize all pages (isolate failures so one broken page can't take
    // down the others)
    for (const page of [ChatPage, ...Object.values(pages)]) {
      try {
        page.init();
      } catch (e) {
        console.warn('page init failed:', e);
      }
    }

    // Activate the initially-visible page lifecycle
    if (pages[activePage] && pages[activePage].onActivate) {
      pages[activePage].onActivate();
    }
  }

  function connectToROS() {
    // Get rosbridge port from config endpoint, or use default
    fetch('/api/config')
      .then(r => r.json())
      .then(config => {
        const port = config.rosbridge_port || 9090;
        const host = window.location.hostname || 'localhost';
        ROSConn.connect(`ws://${host}:${port}`);
      })
      .catch(() => {
        // Fallback: try localhost:9090
        ROSConn.connect(`ws://${window.location.hostname || 'localhost'}:9090`);
      });

    ROSConn.on((event) => {
      const status = document.getElementById('conn-status');
      const text = document.getElementById('conn-text');
      if (event === 'connected') {
        status.className = 'connected';
        text.textContent = 'connected';
      } else if (event === 'disconnected') {
        status.className = 'disconnected';
        text.textContent = 'reconnecting...';
      } else {
        status.className = 'disconnected';
        text.textContent = 'error';
      }
    });
  }

  function switchPage(name) {
    if (name === activePage) return;

    // Deactivate current
    if (pages[activePage] && pages[activePage].onDeactivate) {
      pages[activePage].onDeactivate();
    }

    // Update tabs
    document.querySelectorAll('.tab').forEach((t) => {
      t.classList.toggle('active', t.dataset.page === name);
    });

    // Update pages
    document.querySelectorAll('.page').forEach((p) => {
      p.classList.toggle('active', p.id === `page-${name}`);
    });

    activePage = name;

    // Activate new page
    if (pages[name] && pages[name].onActivate) {
      pages[name].onActivate();
    }
  }

  // Boot
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
