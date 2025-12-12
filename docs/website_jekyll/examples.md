---
layout: splash
title: Examples
permalink: /examples/
header:
  overlay_image: /assets/images/hero/hero.jpg
  overlay_filter: 0.25
  caption: "Pick an example to run. Click a card to view the steps."
feature_row:
  - image_path: /assets/images/examples/connect.png
    alt: "Connect to the Exo"
    title: "Connect to the Exo"
    excerpt: "Find your COM port and verify the serial link at 57600 baud."
    url: "/examples/#modal-connect"
    btn_label: "Open"
    btn_class: "btn--primary"

  - image_path: /assets/images/examples/bringup.png
    alt: "Hardware Controls"
    title: "Hardware Controls"
    excerpt: "Home, enable/disable, get/set params, LEDs—direct device commands."
    url: "/examples/#modal-hw"
    btn_label: "Open"
    btn_class: "btn--primary"

  - image_path: /assets/images/examples/telemetry.png
    alt: "Telemetry + GUI"
    title: "Telemetry + GUI"
    excerpt: "Stream sensors and drive the GUI."
    url: "/examples/#modal-telemetry"
    btn_label: "Open"
    btn_class: "btn--primary"

feature_row2:
  - image_path: /assets/images/examples/emg.png
    alt: "Live EMG Classification"
    title: "Live EMG Classification"
    excerpt: "250 ms windows with 50 ms step, real-time predictions."
    url: "/examples/#modal-live-emg"
    btn_label: "Open"
    btn_class: "btn--primary"

  - image_path: /assets/images/examples/ros2_coming.png
    alt: "ROS2 Teleop (Coming Soon)"
    title: "ROS2 Teleop"
    excerpt: "Coming soon."
    url: "#"             # disabled
    btn_label: "Coming Soon"
    btn_class: "btn--inverse"

  - image_path: /assets/images/examples/lsl.png
    alt: "LSL Streaming"
    title: "LSL Streaming"
    excerpt: "Publish/consume exo data via Lab Streaming Layer."
    url: "/examples/#modal-lsl"
    btn_label: "Open"
    btn_class: "btn--primary"

---

{% include feature_row %}
{% include feature_row id="feature_row2" %}
<style>
  .archive__item a[href="#"] { pointer-events: none; }
  .archive__item a[href="#"] .btn { opacity:.6; }
  .archive__item a[href="#"] img { filter: grayscale(100%) contrast(.9) brightness(.9); }
</style>


<style>
  /* VS Code Dark Theme Modal Styles */
  .ex-modal {
    position: fixed; inset: 0; display: none; align-items: center; justify-content: center;
    background: rgba(0,0,0,.75); z-index: 1000; padding: 1.5rem;
    backdrop-filter: blur(2px);
  }
  .ex-modal[aria-hidden="false"] { 
    display: flex;
    animation: fadeIn 0.25s cubic-bezier(0.4, 0, 0.2, 1);
  }
  
  .ex-modal__panel {
    max-width: 900px; width: 100%; max-height: 85vh; overflow: auto;
    background: #252526;
    color: #d4d4d4;
    border: 1px solid #3e3e42;
    border-radius: 6px;
    box-shadow: 0 20px 60px rgba(0,0,0,.5), 0 0 0 1px rgba(86, 156, 214, 0.2);
    padding: 1.5rem 2rem 2rem;
    animation: slideUp 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  }
  
  .ex-modal__header { 
    display: flex; 
    align-items: center; 
    justify-content: space-between; 
    gap: 1rem; 
    margin-bottom: 1.25rem;
    padding-bottom: 1rem;
    border-bottom: 1px solid #3e3e42;
  }
  
  .ex-modal__title { 
    margin: 0; 
    font-size: 1.5rem; 
    font-weight: 600; 
    color: #569cd6;
    letter-spacing: -0.01em;
  }
  
  .ex-modal__close { 
    background: transparent;
    border: 0;
    font-size: 1.75rem;
    line-height: 1;
    cursor: pointer;
    color: #858585;
    padding: 4px 8px;
    border-radius: 4px;
    transition: all 0.2s ease;
  }
  
  .ex-modal__close:hover {
    background: rgba(86, 156, 214, 0.15);
    color: #d4d4d4;
  }
  
  body.modal-open { overflow: hidden; }
  
  /* Content styling */
  .ex-modal h4 {
    color: #4ec9b0;
    font-size: 1.1rem;
    font-weight: 600;
    margin-top: 1.5rem;
    margin-bottom: 0.75rem;
    border-left: 3px solid #569cd6;
    padding-left: 0.75rem;
  }
  
  .ex-modal h4:first-of-type {
    margin-top: 1rem;
  }
  
  .ex-modal p {
    line-height: 1.6;
    color: #d4d4d4;
  }
  
  .ex-modal p strong {
    color: #dcdcaa;
    font-weight: 600;
  }
  
  .ex-modal ul {
    margin: 0.75rem 0;
    padding-left: 1.5rem;
  }
  
  .ex-modal li {
    margin-bottom: 0.5rem;
    line-height: 1.6;
  }
  
  .ex-modal code {
    background: #1e1e1e;
    color: #ce9178;
    padding: 2px 6px;
    border-radius: 3px;
    font-size: 0.9rem;
    border: 1px solid #3e3e42;
  }
  
  .ex-modal pre {
    margin: 0.75rem 0;
    background: #1e1e1e;
    border: 1px solid #3e3e42;
    border-radius: 4px;
    padding: 1rem;
    overflow-x: auto;
  }
  
  .ex-modal pre code {
    background: transparent;
    border: none;
    padding: 0;
    color: #d4d4d4;
    font-size: 0.9rem;
  }
  
  .ex-modal em {
    color: #858585;
    font-style: italic;
  }
  
  /* Scrollbar styling for modal */
  .ex-modal__panel::-webkit-scrollbar {
    width: 12px;
  }
  
  .ex-modal__panel::-webkit-scrollbar-track {
    background: #1e1e1e;
    border-radius: 0 6px 6px 0;
  }
  
  .ex-modal__panel::-webkit-scrollbar-thumb {
    background: #424242;
    border-radius: 6px;
  }
  
  .ex-modal__panel::-webkit-scrollbar-thumb:hover {
    background: #4e4e4e;
  }
  
  @keyframes fadeIn {
    from { opacity: 0; }
    to { opacity: 1; }
  }
  
  @keyframes slideUp {
    from { 
      transform: translateY(30px);
      opacity: 0;
    }
    to { 
      transform: translateY(0);
      opacity: 1;
    }
  }
</style>

<!-- Connect to the Exo -->
<div class="ex-modal" id="modal-connect" role="dialog" aria-modal="true" aria-hidden="true" aria-labelledby="t-connect">
  <div class="ex-modal__panel">
    <div class="ex-modal__header">
      <h3 class="ex-modal__title" id="t-connect">Connect to the Exo</h3>
      <button class="ex-modal__close" aria-label="Close">×</button>
    </div>

    <p><strong>Goal:</strong> confirm the board is visible on your computer, then exercise the CLI: list ports, query info, home, send gestures/LED, monitor output.</p>

    <h4>Prerequisites</h4>
    <ul>
      <li>Controller flashed (e.g., <em>OpenRB-150</em>) and powered.</li>
      <li>USB data cable connected to your PC.</li>
      <li>Python 3.10+ and <code>pyserial</code> installed:
        <pre><code class="language-powershell">pip install pyserial</code></pre>
      </li>
      <li><code>tools\hand_exo_cli.py</code> present (from this repo).</li>
      <li>Default baud: <strong>57600</strong> (change if your firmware differs).</li>
    </ul>

    <h4>1) Find the serial port</h4>
    <pre><code class="language-powershell"># Windows
python tools\hand_exo_cli.py --list-ports
# Or: Device Manager → Ports (COM & LPT)</code></pre>
    <pre><code class="language-bash"># macOS
ls /dev/tty.usbmodem* /dev/tty.usbserial* 2>/dev/null
# Linux
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null</code></pre>

    <h4>2) Connect and query device info</h4>
    <pre><code class="language-powershell">python tools\hand_exo_cli.py --connect COM5 --baud 57600 --info --read-for 2</code></pre>
    <p><em>Expected:</em> a brief info block (firmware, board, etc.) without errors.</p>

    <h4>3) Home axes</h4>
    <pre><code class="language-powershell"># Home all (default)
python tools\hand_exo_cli.py --connect COM5 --home --read-for 3

# Home a specific target (if supported, e.g., "thumb")
python tools\hand_exo_cli.py --connect COM5 --home thumb --read-for 3</code></pre>

    <h4>4) Send commands</h4>
    <pre><code class="language-powershell"># Gesture demo
python tools\hand_exo_cli.py --connect COM5 --send "gc:pinch:a" --read-for 2

# Toggle LED (firmware must support these tokens)
python tools\hand_exo_cli.py --connect COM5 --send "led:1:on" --read-for 1
python tools\hand_exo_cli.py --connect COM5 --send "led:1:off"</code></pre>

    <h4>5) Continuous monitor (after actions)</h4>
    <pre><code class="language-powershell"># Send a command then tail the serial stream (Ctrl-C to stop)
python tools\hand_exo_cli.py --connect COM5 --send "sys:info" --monitor</code></pre>

    <h4>6) Useful options</h4>
    <pre><code class="language-powershell"># Different line ending (if your firmware expects CRLF)
python tools\hand_exo_cli.py --connect COM5 --eol crlf --send "sys:info" --read-for 2

# Custom info command
python tools\hand_exo_cli.py --connect COM5 --info --info-cmd "sys:ver" --read-for 2

# Change baud rate (match your firmware)
python tools\hand_exo_cli.py --connect COM5 --baud 115200 --info --read-for 2</code></pre>

    <h4>Troubleshooting</h4>
    <ul>
      <li><em>No port appears:</em> try a different USB cable/port; on Windows, set driver to WinUSB/CDC via Zadig.</li>
      <li><em>Port busy:</em> close Arduino Serial Monitor/PuTTY and retry.</li>
      <li><em>Linux permissions:</em> <code>sudo usermod -a -G dialout $USER</code> then log out/in.</li>
      <li><em>Wrong EOL:</em> try <code>--eol crlf</code> if commands are ignored.</li>
    </ul>
  </div>
</div>

<!-- Telemetry + GUI -->
<div class="ex-modal" id="modal-telemetry" role="dialog" aria-modal="true" aria-hidden="true" aria-labelledby="t-telemetry">
  <div class="ex-modal__panel">
    <div class="ex-modal__header">
      <h3 class="ex-modal__title" id="t-telemetry">Telemetry + GUI</h3>
      <button class="ex-modal__close" aria-label="Close">×</button>
    </div>
    <p>Run the telemetry server and GUI in separate terminals.</p>
<pre><code class="language-powershell"># terminal 1 — telemetry
python tools\telemetry_server.py

# terminal 2 — GUI
python tools\gui.py</code></pre>
    <p><strong>Expected:</strong> live sensor stream + control panel.</p>
  </div>
</div>

<!-- Live EMG -->
<div class="ex-modal" id="modal-live-emg" role="dialog" aria-modal="true" aria-hidden="true" aria-labelledby="t-live-emg">
  <div class="ex-modal__panel">
    <div class="ex-modal__header">
      <h3 class="ex-modal__title" id="t-live-emg">Live EMG Classification</h3>
      <button class="ex-modal__close" aria-label="Close">×</button>
    </div>
    <p>Stream <strong>250 ms</strong> windows with <strong>50 ms</strong> step and classify in real time.</p>
<pre><code class="language-powershell">python examples\05_emg_feature_pipeline\realtime_predict.py --device intan --win 0.25 --step 0.05</code></pre>
  </div>
</div>

<!-- ROS2 Teleop (Coming Soon) -->
<div class="ex-modal" id="modal-ros2" role="dialog" aria-modal="true" aria-hidden="true" aria-labelledby="t-ros2">
  <div class="ex-modal__panel">
    <div class="ex-modal__header">
      <h3 class="ex-modal__title" id="t-ros2">ROS2 Teleop</h3>
      <button class="ex-modal__close" aria-label="Close">×</button>
    </div>
    <p><em>Coming soon:</em> launch files, joystick mapping, and MoveIt scene setup.</p>
  </div>
</div>

<!-- Hardware Controls -->
<div class="ex-modal" id="modal-hw" role="dialog" aria-modal="true" aria-hidden="true" aria-labelledby="t-hw">
  <div class="ex-modal__panel">
    <div class="ex-modal__header">
      <h3 class="ex-modal__title" id="t-hw">Hardware Controls</h3>
      <button class="ex-modal__close" aria-label="Close">×</button>
    </div>

    <p>Direct device control using <code>tools\hand_exo_cli.py</code>. These examples send plain text tokens over serial.
       Exact tokens may differ per firmware—try <code>help</code> or see your Doxygen docs for the authoritative list.</p>

    <div class="notice--info">
      <strong>Tip:</strong> Chain multiple <code>--send</code> flags to run a short sequence, and use <code>--read-for</code> to capture responses.
    </div>

    <h4>Status & Info</h4>
<pre><code class="language-powershell"># Print device info (uses --info-cmd, defaults to "sys:info")
python tools\hand_exo_cli.py --connect COM5 --info --read-for 2

# Ask firmware for version/help (if supported)
python tools\hand_exo_cli.py --connect COM5 --send "sys:ver" --read-for 1
python tools\hand_exo_cli.py --connect COM5 --send "help" --read-for 3</code></pre>

    <h4>Homing & Motion</h4>
<pre><code class="language-powershell"># Home all or a specific target (if supported, e.g., "thumb" / "index")
python tools\hand_exo_cli.py --connect COM5 --send "home all"   --read-for 3
python tools\hand_exo_cli.py --connect COM5 --send "home thumb" --read-for 3

# Demo gesture (example token)
python tools\hand_exo_cli.py --connect COM5 --send "gc:pinch:a" --read-for 2</code></pre>

    <h4>Enable / Disable Actuators</h4>
<pre><code class="language-powershell"># Common patterns (your firmware may vary):
#   mot:all:on / mot:all:off
#   motor:enable:all / motor:disable:all
python tools\hand_exo_cli.py --connect COM5 --send "mot:all:on"  --read-for 1
python tools\hand_exo_cli.py --connect COM5 --send "mot:all:off" --read-for 1</code></pre>

    <h4>Get / Set Parameters</h4>
    <p>Many firmwares expose a <code>get:PARAM</code> / <code>set:PARAM:VALUE</code> convention.
       Replace names/values with those documented by your firmware.</p>
<pre><code class="language-powershell"># Examples (adjust to your command set)
python tools\hand_exo_cli.py --connect COM5 --send "get:baud"           --read-for 1
python tools\hand_exo_cli.py --connect COM5 --send "set:baud:57600"     --read-for 1
python tools\hand_exo_cli.py --connect COM5 --send "get:lim:curr"       --read-for 1
python tools\hand_exo_cli.py --connect COM5 --send "set:lim:curr:0.8"   --read-for 1
python tools\hand_exo_cli.py --connect COM5 --send "get:lim:vel"        --read-for 1
python tools\hand_exo_cli.py --connect COM5 --send "set:lim:vel:0.5"    --read-for 1</code></pre>

    <h4>Sensors & Calibration</h4>
<pre><code class="language-powershell"># Read sensors (examples; adapt to your tokens)
python tools\hand_exo_cli.py --connect COM5 --send "read:force"  --read-for 1
python tools\hand_exo_cli.py --connect COM5 --send "read:emg"    --read-for 1

# Zero / tare (examples)
python tools\hand_exo_cli.py --connect COM5 --send "zero:force"  --read-for 1
python tools\hand_exo_cli.py --connect COM5 --send "cal:emg:base" --read-for 2</code></pre>

    <h4>IO & Utilities</h4>
<pre><code class="language-powershell"># LED control (already supported in your earlier docs)
python tools\hand_exo_cli.py --connect COM5 --send "led:1:on"  --read-for 1
python tools\hand_exo_cli.py --connect COM5 --send "led:1:off" --read-for 1

# Save & load config (if your firmware supports)
python tools\hand_exo_cli.py --connect COM5 --send "cfg:save"    --read-for 1
python tools\hand_exo_cli.py --connect COM5 --send "cfg:load:0"  --read-for 1</code></pre>

    <h4>Recipe — Safety-limited demo</h4>
<pre><code class="language-powershell">python tools\hand_exo_cli.py --connect COM5 `
  --send "mot:all:on" `
  --send "set:lim:curr:0.6" `
  --send "home all" `
  --send "gc:pinch:a" `
  --send "mot:all:off" `
  --read-for 2</code></pre>

    <div class="notice--warning">
      <strong>Safety:</strong> keep current/velocity limits conservative for first runs. Use an E-stop and clear workspace.
    </div>
  </div>
</div>


<!-- LSL Streaming -->
<div class="ex-modal" id="modal-lsl" role="dialog" aria-modal="true" aria-hidden="true" aria-labelledby="t-lsl">
  <div class="ex-modal__panel">
    <div class="ex-modal__header">
      <h3 class="ex-modal__title" id="t-lsl">LSL Streaming</h3>
      <button class="ex-modal__close" aria-label="Close">×</button>
    </div>

    <p>Stream exo telemetry over <strong>Lab Streaming Layer (LSL)</strong> using your provided demo scripts.</p>

    <h4>Prerequisites</h4>
    <pre><code class="language-powershell">pip install pylsl</code></pre>

    <h4>1) Start an outlet (publisher)</h4>
    <p><em>Option A — hardware bridge (recommended when the exo is connected):</em></p>
<pre><code class="language-powershell">python examples\lsl\lsl_broadcast_test.py</code></pre>
    <p><em>Option B — synthetic data (no hardware required):</em></p>
<pre><code class="language-powershell">python examples\lsl\demo_send.py</code></pre>

    <h4>2) Consume/print samples</h4>
    <p>Basic subscriber that prints incoming data:</p>
<pre><code class="language-powershell">python examples\lsl\lsl_subscribe_test.py</code></pre>

    <h4>3) Visualize</h4>
    <p>Pick any of the provided visualizers:</p>
<pre><code class="language-powershell"># Grid plot of multiple channels
python examples\lsl\lsl_grid_plot.py

# Stacked time series
python examples\lsl\lsl_stacked_plot.py

# RMS bar plot (windowed feature)
python examples\lsl\lsl_rms_barplot.py</code></pre>

    <h4>4) Gesture/control subscribers (optional)</h4>
<pre><code class="language-powershell"># Subscribe to gesture labels or control messages
python examples\lsl\lsl_gesture_sub.py

# Classifier-trigger demo
python examples\lsl\lsl_classifier_trigger.py

# Gesture controller (publishes control/acts on streams)
python examples\lsl\lsl_gesture_controller.py</code></pre>

    <h4>Troubleshooting</h4>
    <ul>
      <li><em>No data in subscribers:</em> ensure one of the publishers (<code>lsl_broadcast_test.py</code> or <code>demo_send.py</code>) is running first.</li>
      <li><em>Multiple streams:</em> some scripts may have hardcoded stream names—open the file to confirm the <code>name</code>/<code>type</code> if needed.</li>
      <li><em>High CPU:</em> lower plotting refresh or sample rate in the scripts.</li>
    </ul>
  </div>
</div>

<script>
(function() {
  function $(sel, root=document){ return root.querySelector(sel); }
  function $all(sel, root=document){ return Array.from(root.querySelectorAll(sel)); }

  function openModal(modal){
    modal.setAttribute('aria-hidden', 'false');
    document.body.classList.add('modal-open');
    const btn = modal.querySelector('.ex-modal__close');
    if (btn) btn.focus();
  }
  function closeModal(modal){
    modal.setAttribute('aria-hidden', 'true');
    document.body.classList.remove('modal-open');
    if (location.hash.startsWith('#modal-')) history.replaceState(null, "", location.pathname);
  }

  // Intercept any link that contains "#modal-" IF the matching modal exists on this page
  document.addEventListener('click', function(ev) {
    const a = ev.target.closest('a');
    if (!a) return;
    const href = a.getAttribute('href') || "";
    const hashPos = href.indexOf('#modal-');
    if (hashPos === -1) return;
    const hash = href.slice(hashPos);        // "#modal-…"
    const modal = $(hash);
    if (!modal) return;                      // link goes elsewhere

    ev.preventDefault();
    openModal(modal);
    history.replaceState(null, "", hash);    // keep hash without navigating away
  }, true);

  // Close on ESC or outside click
  $all('.ex-modal').forEach(modal => {
    modal.addEventListener('click', e => { if (e.target === modal) closeModal(modal); });
    const btn = modal.querySelector('.ex-modal__close');
    if (btn) btn.addEventListener('click', () => closeModal(modal));
  });
  document.addEventListener('keydown', e => {
    if (e.key === 'Escape') $all('.ex-modal[aria-hidden="false"]').forEach(closeModal);
  });

  // Open if this page is loaded with a "#modal-…" hash
  if (location.hash.startsWith('#modal-')) {
    const m = $(location.hash);
    if (m) openModal(m);
  }
})();
</script>
