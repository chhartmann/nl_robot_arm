"""NL Robot Arm — web server node.

Serves the web UI static files and exposes configuration to the frontend
via a JSON endpoint. It also serves a processed (plain) URDF of the robot
plus its mesh files so the browser can render the 3D view with URDFLoader.
"""
import http.server
import json
import os
import re
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory


class WebConfigServer(Node):
    """ROS 2 node that serves static files and provides a /api/config endpoint."""

    def __init__(self):
        super().__init__("nlra_web_ui_server")

        # Find the web/ directory via ament package share
        try:
            pkg_share = get_package_share_directory("nlra_web_ui")
            self._web_dir = os.path.join(pkg_share, "web")
        except Exception:
            # Fallback for development layout
            self._web_dir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(
                    os.path.abspath(__file__)))),
                "web")

        if not os.path.isdir(self._web_dir):
            self.get_logger().error(f"web/ directory not found at {self._web_dir}")
            self._web_dir = "."

        self.declare_parameter("port", 8080)
        self.declare_parameter("rosbridge_port", 9090)
        port = self.get_parameter("port").value
        rb_port = self.get_parameter("rosbridge_port").value

        # Plain URDF for the 3D view (None if generation fails; the frontend
        # then falls back to a primitive visualization).
        self._generated_urdf = self._generate_urdf()

        # Serve static files
        handler = self._make_handler()
        self._httpd = http.server.HTTPServer(("0.0.0.0", port), handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever,
                                        daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"web UI serving at http://0.0.0.0:{port} (rosbridge :{rb_port})")

    def _generate_urdf(self):
        """Process the robot xacro into a plain URDF the browser can load.

        Rewrites the mesh URIs from package://... and file://$(find ...)...
        forms into package://<alias>/... forms that the frontend maps to HTTP
        paths served under /meshes/.
        """
        try:
            desc_share = get_package_share_directory("agilus_robotiq_description")
            xacro_file = os.path.join(desc_share, "urdf", "agilus_robotiq.urdf.xacro")
            controller_config = os.path.join(
                desc_share, "config", "agilus_robotiq_controllers.yaml")
            proc = subprocess.run(
                ["xacro", xacro_file, f"controller_config:={controller_config}"],
                capture_output=True, text=True, check=True, timeout=120)
            urdf = proc.stdout
        except Exception as e:
            self.get_logger().error(f"failed to generate URDF for 3D view: {e}")
            return None

        urdf = re.sub(r'file://[^" ]*?/robotiq_description/meshes/',
                      'package://robotiq/meshes/', urdf)
        urdf = urdf.replace('package://agilus_robotiq_description/meshes/',
                            'package://agilus/meshes/')
        return urdf

    def _make_handler(self):
        web_dir = self._web_dir
        node = self

        class Handler(http.server.SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=web_dir, **kwargs)

            def end_headers(self):
                # The UI is served from a local development/simulation node;
                # stale cached bundles otherwise survive a rebuild and can
                # mix old markup with new JavaScript.
                self.send_header("Cache-Control", "no-store")
                super().end_headers()

            def do_GET(self):
                if self.path == "/api/config":
                    rb_port = node.get_parameter("rosbridge_port").value
                    body = json.dumps({"rosbridge_port": rb_port}).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path == "/models/agilus_robotiq.urdf":
                    self._serve_urdf()
                    return
                if self.path.startswith("/meshes/"):
                    self._serve_mesh(self.path)
                    return
                super().do_GET()

            def _serve_urdf(self):
                body = node._generated_urdf
                if body is None:
                    self.send_error(500, "URDF generation failed")
                    return
                body = body.encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/xml")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _serve_mesh(self, path):
                # /meshes/<pkg>/meshes/<rel> maps to
                # share/<package>/meshes/<rel> for agilus_robotiq_description
                # and robotiq_description.
                parts = path.split("/")
                if len(parts) < 4 or parts[1] != "meshes":
                    self.send_error(404)
                    return
                package = parts[2]
                rel = "/".join(parts[3:])
                if package not in ("agilus", "robotiq"):
                    self.send_error(404)
                    return
                ros_pkg = ("agilus_robotiq_description"
                           if package == "agilus" else "robotiq_description")
                try:
                    base = os.path.normpath(get_package_share_directory(ros_pkg))
                except Exception:
                    self.send_error(404)
                    return
                # Check the logical path stays inside the package share dir.
                # (The files may be symlinks to the workspace src/ tree.)
                full = os.path.normpath(os.path.join(base, rel))
                if os.path.commonpath([base, full]) != base:
                    self.send_error(403)
                    return
                try:
                    with open(full, "rb") as f:
                        data = f.read()
                except OSError:
                    self.send_error(404)
                    return
                ctype = ("model/vnd.collada+xml" if full.endswith(".dae")
                         else "application/sla" if full.endswith(".stl")
                         else "application/octet-stream")
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, format, *args):
                node.get_logger().debug(format % args)

        return Handler

    def destroy_node(self):
        self._httpd.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebConfigServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
