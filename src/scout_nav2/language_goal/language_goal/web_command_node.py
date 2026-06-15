"""web_command_node: a tiny web UI for the NavigateToObject service (demo).

Serves one page with a text box; a free-text command is POSTed to /command,
which calls /semantic_nav/navigate_to_object and shows the response. This makes
the natural-language (LLM) interface visible in a browser for the demo, instead
of raw `ros2 service call`. Python stdlib http.server only -- no extra deps.

The LLM parsing happens in goal_commander_node; this node just relays the typed
command to that service and displays the returned message.
"""

from __future__ import annotations

import json
import threading
import time
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from semantic_nav_msgs.srv import NavigateToObject


PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>Semantic Navigation</title>
<style>
  :root { color-scheme: dark; }
  body { margin:0; font-family: system-ui, sans-serif; background:#0f1115;
         color:#e6e6e6; display:flex; min-height:100vh; align-items:center;
         justify-content:center; }
  .card { width:min(640px, 92vw); background:#171a21; border:1px solid #262b36;
          border-radius:16px; padding:28px 28px 22px; box-shadow:0 10px 40px #0008; }
  h1 { font-size:20px; margin:0 0 4px; }
  p.sub { margin:0 0 18px; color:#8b93a7; font-size:13px; }
  .row { display:flex; gap:10px; }
  input { flex:1; padding:14px 16px; font-size:16px; border-radius:10px;
          border:1px solid #2c3340; background:#0f1115; color:#e6e6e6; }
  input:focus { outline:none; border-color:#4f8cff; }
  button { padding:14px 20px; font-size:16px; border:none; border-radius:10px;
           background:#4f8cff; color:#fff; cursor:pointer; }
  button:disabled { opacity:.5; cursor:default; }
  .chips { margin:14px 0 0; display:flex; flex-wrap:wrap; gap:8px; }
  .chip { padding:7px 12px; font-size:13px; border-radius:999px; cursor:pointer;
          background:#222734; border:1px solid #2c3340; color:#c7cedb; }
  #out { margin-top:18px; padding:14px 16px; border-radius:10px; font-size:14px;
         white-space:pre-wrap; display:none; }
  #out.ok { background:#10271a; border:1px solid #1f6b3b; }
  #out.no { background:#2a1414; border:1px solid #6b1f1f; }
  #out.wait { background:#1b2030; border:1px solid #2c3340; color:#9aa3b5; }
</style>
</head>
<body>
  <div class="card">
    <h1>Send the robot with natural language</h1>
    <p class="sub">Type a command in plain language. An LLM interprets the
       intent to choose the navigation goal.</p>
    <div class="row">
      <input id="cmd" placeholder="e.g. go to the farthest fire extinguisher"
             autocomplete="off"/>
      <button id="go">Send</button>
    </div>
    <div class="chips" id="chips"></div>
    <div id="out"></div>
  </div>
<script>
  const examples = [
    "go to the fire extinguisher",
    "go to the nearest fire extinguisher",
    "take me to the farthest fire extinguisher",
    "head over to the closest extinguisher please",
    "go to the refrigerator"
  ];
  const chips = document.getElementById("chips");
  const input = document.getElementById("cmd");
  const out = document.getElementById("out");
  const go = document.getElementById("go");
  examples.forEach(function(e) {
    const c = document.createElement("span");
    c.className = "chip"; c.textContent = e;
    c.onclick = function() { input.value = e; input.focus(); };
    chips.appendChild(c);
  });
  function show(cls, text) {
    out.style.display = "block"; out.className = cls; out.textContent = text;
  }
  async function send() {
    const command = input.value.trim();
    if (!command) { input.focus(); return; }
    go.disabled = true;
    show("wait", "Sending...");
    try {
      const r = await fetch("/command", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({command: command})
      });
      const j = await r.json();
      show(j.accepted ? "ok" : "no",
           (j.accepted ? "Accepted\\n" : "Rejected\\n") + (j.message || ""));
    } catch (err) {
      show("no", "Request failed: " + err);
    } finally {
      go.disabled = false;
    }
  }
  go.onclick = send;
  input.addEventListener("keydown", function(ev) {
    if (ev.key === "Enter") { send(); }
  });
</script>
</body>
</html>
"""


class WebCommandNode(Node):
    """ROS node that bridges a small HTTP page to the NavigateToObject service."""

    def __init__(self):
        super().__init__("web_command_node")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("service_name", "/semantic_nav/navigate_to_object")
        self.declare_parameter("call_timeout_sec", 60.0)

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        self.service_name = self.get_parameter("service_name").value
        self.call_timeout = float(self.get_parameter("call_timeout_sec").value)

        self.cb_group = ReentrantCallbackGroup()
        self.client = self.create_client(
            NavigateToObject, self.service_name, callback_group=self.cb_group
        )

        handler = partial(_Handler, self)
        self.httpd = ThreadingHTTPServer((self.host, self.port), handler)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(
            f"web_command_node up: open http://localhost:{self.port}"
        )

    def send_command(self, command: str) -> dict:
        """Call the NavigateToObject service and return {accepted, message}."""
        if not self.client.wait_for_service(timeout_sec=5.0):
            return {
                "accepted": False,
                "message": f"Service {self.service_name} is unavailable.",
            }
        req = NavigateToObject.Request()
        req.command = command
        future = self.client.call_async(req)
        deadline = time.monotonic() + self.call_timeout
        while time.monotonic() < deadline:
            if future.done():
                res = future.result()
                return {"accepted": bool(res.accepted), "message": res.message}
            time.sleep(0.02)
        return {"accepted": False, "message": "Service call timed out."}


class _Handler(BaseHTTPRequestHandler):
    def __init__(self, node: WebCommandNode, *args, **kwargs):
        self.node = node
        super().__init__(*args, **kwargs)

    def log_message(self, *args):  # silence default stderr access logging
        pass

    def _send(self, code: int, content_type: str, body: bytes):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path != "/command":
            self.send_error(404)
            return
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            data = json.loads(raw.decode("utf-8"))
            command = str(data.get("command", "")).strip()
        except (ValueError, UnicodeDecodeError):
            command = ""
        if not command:
            result = {"accepted": False, "message": "Empty command."}
        else:
            result = self.node.send_command(command)
        body = json.dumps(result, ensure_ascii=False).encode("utf-8")
        self._send(200, "application/json; charset=utf-8", body)


def main(args=None):
    rclpy.init(args=args)
    node = WebCommandNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
