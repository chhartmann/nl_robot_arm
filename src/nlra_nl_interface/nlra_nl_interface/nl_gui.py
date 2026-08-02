#!/usr/bin/env python3
"""NL Robot Arm — chat GUI for natural-language commands.

Opens its own window: type commands, see the grounded task and the robot's
answer. Runs ROS on a background thread, UI in the main thread.

Usage (with the nlra stack running):
  ros2 run nlra_nl_interface nl_gui
"""
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from nlra_interfaces.srv import NLCommand

from PyQt5.QtCore import Qt, QObject, QTimer, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QHBoxLayout, QLabel, QLineEdit,
                             QMainWindow, QPushButton, QTextBrowser, QVBoxLayout,
                             QWidget)

SERVICE = "nl_command"


class Bridge(QObject):
    """Qt-side signals fed from the ROS worker thread."""
    command = pyqtSignal(str, str, str)      # text, task line, answer line
    status = pyqtSignal(str)


class ROSWorker:
    """ROS node + executor on a dedicated thread."""

    def __init__(self, bridge):
        self._bridge = bridge
        self._node = Node("nl_gui_ros")
        self._client = self._node.create_client(NLCommand, SERVICE)

    def start(self):
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        self._executor.spin()

    def service_ready(self):
        return self._client.service_is_ready()

    def send(self, text):
        if not self.service_ready():
            self._bridge.command.emit(
                text, "", f"service {SERVICE} not available — "
                          "is the nlra stack running?")
            return
        req = NLCommand.Request(text=text)
        fut = self._client.call_async(req)
        fut.add_done_callback(lambda f: self._on_done(f, text))

    def _on_done(self, fut, text):
        task_line, answer = "", ""
        try:
            resp = fut.result()
        except Exception as e:  # noqa: BLE001
            answer = f"service error: {e}"
        else:
            if resp.success:
                if resp.task:
                    task_line = f"[{resp.task} {resp.args_json}]"
                if resp.response:
                    answer = resp.response
                if not task_line and not answer:
                    answer = "(done, no message)"
            else:
                answer = f"FAILED: {resp.error or '(no error message)'}"
        self._bridge.command.emit(text, task_line, answer)

    def shutdown(self):
        self._node.destroy_node()


class MainWindow(QMainWindow):
    def __init__(self, worker, bridge):
        super().__init__()
        self._worker = worker
        self._bridge = bridge
        self._bridge.command.connect(self._show)
        self._bridge.status.connect(self._status)

        self.setWindowTitle("NL Robot Arm — command window")
        self.resize(640, 480)

        self._log = QTextBrowser()
        self._log.setOpenExternalLinks(False)
        self._log.append("<i>Ask the robot — e.g. \"put the red cube into "
                         "the blue tray\"</i>")

        self._entry = QLineEdit()
        self._entry.setPlaceholderText("type a command and press Enter…")
        self._entry.returnPressed.connect(self._send)

        self._send_btn = QPushButton("Send")
        self._send_btn.clicked.connect(self._send)

        row = QHBoxLayout()
        row.addWidget(self._entry, 1)
        row.addWidget(self._send_btn)

        self._status = QLabel("waiting for /nl_command…")
        self._status.setStyleSheet("color: #888;")

        layout = QVBoxLayout()
        layout.addWidget(self._log, 1)
        layout.addLayout(row)
        layout.addWidget(self._status)

        central = QWidget()
        central.setLayout(layout)
        self.setCentralWidget(central)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._check_service)
        self._timer.start(2000)

    def _check_service(self):
        ready = self._worker.service_ready()
        self._status.setText("connected to /nl_command" if ready
                             else "waiting for /nl_command…")
        self._send_btn.setEnabled(ready)
        self._entry.setEnabled(ready)

    def _send(self):
        text = self._entry.text().strip()
        if not text:
            return
        self._entry.clear()
        self._log.append(f"<b>you:</b> {text}")
        self._worker.send(text)

    def _show(self, text, task_line, answer):
        self._log.append(f"<b>you:</b> {text}")
        if task_line:
            self._log.append(f"<span style='color:#0066cc;'>{task_line}</span>")
        self._log.append(answer)
        self._log.verticalScrollBar().setValue(
            self._log.verticalScrollBar().maximum())

    def _status(self, text):
        self._status.setText(text)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication([])

    bridge = Bridge()
    worker = ROSWorker(bridge)
    worker.start()

    win = MainWindow(worker, bridge)
    win.show()

    try:
        app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        worker.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
