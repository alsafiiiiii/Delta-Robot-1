#!/usr/bin/env python3
"""Realtime TF plot for end-effector using PyQtGraph (much faster than Matplotlib).

XY, XZ, YZ planes, 3D Isometric View, and pitch/yaw time-series.

Install deps:
  pip install pyqtgraph PyOpenGL PyOpenGL_accelerate

Run with e.g.:
  source /opt/ros/<distro>/setup.bash
  python3 plot_ee_tf.py \
    --parent-frame delta_robot/world_link \
    --commanded delta_robot/commanded_end_effector_pin \
    --ee ee_link \
    --calculated delta_robot/calculated_fk_end_effector_pin \
    --actual delta_robot/actual_fk_end_effector_pin

Controls:
  --max-points N     keep last N points   (default 1000)
  --interval-ms N    plot update rate ms  (default 50)
  --time-window N    RPY plot window (s)  (default 5.0)

Keyboard / GUI:
  "Freeze & Save" button — pauses updates, saves one PNG per window
  "Resume"        button — restarts live updates
"""

import argparse
import os
import time
import threading
from collections import deque
import math
import signal
import sys

import numpy as np

import rclpy
from rclpy.node import Node
import tf2_ros

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtWidgets, QtCore, QtGui

# ---------------------------------------------------------------------------
# Visual style
# ---------------------------------------------------------------------------

# RGBA tuples (0-255)
COLORS = {
    "sim":        (30,  144, 255, 230),   # dodger blue
    "commanded":  (255, 140,   0, 230),   # orange
    "ee":         (50,  205,  50, 230),   # lime green
    "calculated": (220,  50,  50, 230),   # red
    "actual":     (180,  60, 220, 230),   # purple
}

# Qt pen dash styles
_Qt = QtCore.Qt
LINE_STYLES = {
    "sim":        _Qt.SolidLine,
    "commanded":  _Qt.DashLine,
    "ee":         _Qt.DotLine,
    "calculated": _Qt.DashDotLine,
    "actual":     _Qt.DashDotDotLine,
}

LABELS = {
    "sim":        "Sim (Gazebo)",
    "commanded":  "Desired",
    "ee":         "Measured (ToF/IMU)",
    "calculated": "IK→FK",
    "actual":     "Servo FB (FK)",
}

LINE_WIDTH = 2.0

# Fixed plot limits (mm)
XY_RANGE = 70.0
XY_RANGE_LOW   = -60.0
XY_RANGE_HIGH  = 80.0
Z_MIN      = -410.0
Z_MAX      = -330.0
Z_CENTER   = (Z_MIN + Z_MAX) / 2.0          # ≈ -380  — used to aim 3-D camera

# ---------------------------------------------------------------------------
# ROS node — only does TF sampling, no GUI
# ---------------------------------------------------------------------------

class TfSamplerNode(Node):
    def __init__(self, parent_frame, frames, max_points):
        super().__init__("plot_ee_tf")
        self.parent_frame = parent_frame
        self.frames = frames          # dict: key -> child_frame_id
        self.max_points = max_points

        self._keys = list(frames.keys())

        # Per-frame deques
        self.data = {
            k: {
                "x":     deque(maxlen=max_points),
                "y":     deque(maxlen=max_points),
                "z":     deque(maxlen=max_points),
                "roll":  deque(maxlen=max_points),
                "pitch": deque(maxlen=max_points),
                "yaw":   deque(maxlen=max_points),
                "t":     deque(maxlen=max_points),
            }
            for k in self._keys
        }

        self._lock = threading.Lock()

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Spin on background thread
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self,), daemon=True
        )
        self._spin_thread.start()

    # ------------------------------------------------------------------
    def snapshot(self):
        """Return a thread-safe copy of all data."""
        with self._lock:
            return {
                k: {field: list(self.data[k][field]) for field in self.data[k]}
                for k in self._keys
            }

    # ------------------------------------------------------------------
    def sample_all(self):
        """Fetch latest TF for every frame and store. Called from Qt timer."""
        for k in self._keys:
            result = self._fetch(k)
            if result is None:
                continue
            x, y, z, roll, pitch, yaw, ts = result
            with self._lock:
                d = self.data[k]
                d["x"].append(x);      d["y"].append(y);   d["z"].append(z)
                d["roll"].append(roll); d["pitch"].append(pitch); d["yaw"].append(yaw)
                d["t"].append(ts)

    # ------------------------------------------------------------------
    def _fetch(self, key):
        frame = self.frames.get(key)
        if not frame:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                self.parent_frame, frame, rclpy.time.Time()
            )
            x  = t.transform.translation.x
            y  = t.transform.translation.y
            z  = t.transform.translation.z
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            roll, pitch, yaw = _quat_to_euler(qx, qy, qz, qw)
            ts = (float(t.header.stamp.sec)
                  + float(t.header.stamp.nanosec) * 1e-9)
            return x, y, z, roll, pitch, yaw, ts
        except Exception:
            return None

    # ------------------------------------------------------------------
    def get_start_position(self, timeout=2.0):
        order  = ["ee", "actual", "commanded", "calculated", "sim"]
        end_t  = time.time() + timeout
        while time.time() < end_t:
            time.sleep(0.05)
            for k in order:
                r = self._fetch(k)
                if r:
                    self.get_logger().info(
                        f"Start pose from '{k}': "
                        f"x={r[0]*1e3:.1f} mm, y={r[1]*1e3:.1f} mm, z={r[2]*1e3:.1f} mm"
                    )
                    return r[0], r[1], r[2]
        self.get_logger().warning("No TF found within timeout — centering at 0,0,0")
        return 0.0, 0.0, 0.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _quat_to_euler(qx, qy, qz, qw):
    t0   = 2.0 * (qw * qx + qy * qz)
    t1   = 1.0 - 2.0 * (qx*qx + qy*qy)
    roll = math.atan2(t0, t1)
    t2   = 2.0 * (qw * qy - qz * qx)
    t2   = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)
    t3   = 2.0 * (qw * qz + qx * qy)
    t4   = 1.0 - 2.0 * (qy*qy + qz*qz)
    yaw  = math.atan2(t3, t4)
    return roll, pitch, yaw


def _make_pen(key, width=LINE_WIDTH):
    r, g, b, a = COLORS[key]
    return pg.mkPen(color=(r, g, b, a), width=width, style=LINE_STYLES[key])


def _color_gl(key, alpha=0.9):
    """Return RGBA float tuple 0–1 for GL."""
    r, g, b, _ = COLORS[key]
    return (r/255.0, g/255.0, b/255.0, alpha)


# ---------------------------------------------------------------------------
# Control panel (Freeze / Resume / status)
# ---------------------------------------------------------------------------

class ControlPanel(QtWidgets.QWidget):
    """Small always-visible toolbar for freeze/resume and save."""

    def __init__(self, plot_app: "PlotApp"):
        super().__init__()
        self._app = plot_app
        self.setWindowTitle("EE TF Plotter — Controls")
        self.setWindowFlags(
            QtCore.Qt.Window |
            QtCore.Qt.WindowStaysOnTopHint |
            QtCore.Qt.WindowCloseButtonHint
        )
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose, False)

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        # ── title label ──────────────────────────────────────────────
        title = QtWidgets.QLabel("End-Effector TF Plotter")
        title.setStyleSheet("font-weight: bold; font-size: 13px;")
        root.addWidget(title)

        # ── buttons ──────────────────────────────────────────────────
        btn_row = QtWidgets.QHBoxLayout()
        self.btn_freeze = QtWidgets.QPushButton("⏸  Freeze & Save")
        self.btn_resume = QtWidgets.QPushButton("▶  Resume")
        self.btn_resume.setEnabled(False)

        for btn in (self.btn_freeze, self.btn_resume):
            btn.setMinimumHeight(36)
            btn.setMinimumWidth(140)
            btn_row.addWidget(btn)

        root.addLayout(btn_row)

        # ── status label ─────────────────────────────────────────────
        self.lbl_status = QtWidgets.QLabel("● Live")
        self.lbl_status.setStyleSheet("color: #28a745; font-weight: bold;")
        root.addWidget(self.lbl_status)

        # ── saved-files list ─────────────────────────────────────────
        self.lbl_saved = QtWidgets.QLabel("")
        self.lbl_saved.setWordWrap(True)
        self.lbl_saved.setStyleSheet("font-size: 11px; color: #555;")
        root.addWidget(self.lbl_saved)

        self.adjustSize()
        self.setFixedWidth(340)

        # connections
        self.btn_freeze.clicked.connect(self._on_freeze)
        self.btn_resume.clicked.connect(self._on_resume)

    # ------------------------------------------------------------------
    def _on_freeze(self):
        self._app._timer.stop()
        self.btn_freeze.setEnabled(False)
        self.btn_resume.setEnabled(True)
        self.lbl_status.setText("⏸ Frozen")
        self.lbl_status.setStyleSheet("color: #e67e22; font-weight: bold;")
        self.lbl_saved.setText("Saving images…")
        QtWidgets.QApplication.processEvents()   # flush UI before grabbing

        saved = self._app._save_snapshots()

        lines = [f"Saved {len(saved)} files:"]
        for p in saved:
            lines.append(f"  • {os.path.basename(p)}")
        self.lbl_saved.setText("\n".join(lines))

    def _on_resume(self):
        self._app._timer.start(self._app.interval_ms)
        self.btn_freeze.setEnabled(True)
        self.btn_resume.setEnabled(False)
        self.lbl_status.setText("● Live")
        self.lbl_status.setStyleSheet("color: #28a745; font-weight: bold;")
        self.lbl_saved.setText("")


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

class PlotApp:
    def __init__(self, node: TfSamplerNode, interval_ms: int, time_window_s: float):
        self.node          = node
        self.interval_ms   = interval_ms
        self.time_window_s = time_window_s
        self._keys         = list(node.frames.keys())

        pg.setConfigOptions(antialias=True, background="w", foreground="k")
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)

        self._build_2d_windows()
        self._build_3d_window()
        self._build_rpy_window()

        # Control panel (last, so it can reference self)
        self._ctrl = ControlPanel(self)
        self._ctrl.show()

        # Qt timer drives both TF sampling and plot updates from the main thread
        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self._tick)
        self._timer.start(interval_ms)

    # ------------------------------------------------------------------
    # Window construction
    # ------------------------------------------------------------------

    def _styled_plot(self, win, title, xlabel, ylabel):
        p = win.addPlot(title=title)
        p.setLabel("bottom", xlabel)
        p.setLabel("left",   ylabel)
        p.showGrid(x=False, y=False)
        p.getAxis("top").setStyle(showValues=False)
        p.getAxis("right").setStyle(showValues=False)
        leg = p.addLegend(offset=(-10, 10))   # top-right
        leg.setLabelTextColor("k")
        return p

    def _build_2d_windows(self):
        keys = self._keys

        # --- XY ---
        self.win_xy = pg.GraphicsLayoutWidget(title="XY Plane")
        self.win_xy.resize(700, 700)
        p = self._styled_plot(self.win_xy, "XY Plane", "X (mm)", "Y (mm)")
        p.setXRange(XY_RANGE_LOW, XY_RANGE_HIGH, padding=0)
        p.setYRange(XY_RANGE_LOW, XY_RANGE_HIGH, padding=0)
        p.setAspectLocked(True)
        self.curves_xy = {k: p.plot(pen=_make_pen(k), name=LABELS[k]) for k in keys}

        # --- XZ ---
        self.win_xz = pg.GraphicsLayoutWidget(title="XZ Plane")
        self.win_xz.resize(700, 700)
        p = self._styled_plot(self.win_xz, "XZ Plane", "X (mm)", "Z (mm)")
        p.setXRange(XY_RANGE_LOW, XY_RANGE_HIGH, padding=0)
        p.setYRange(Z_MIN, Z_MAX, padding=0)
        self.curves_xz = {k: p.plot(pen=_make_pen(k), name=LABELS[k]) for k in keys}

        # --- YZ ---
        self.win_yz = pg.GraphicsLayoutWidget(title="YZ Plane")
        self.win_yz.resize(700, 700)
        p = self._styled_plot(self.win_yz, "YZ Plane", "Y (mm)", "Z (mm)")
        p.setXRange(XY_RANGE_LOW, XY_RANGE_HIGH, padding=0)
        p.setYRange(Z_MIN, Z_MAX, padding=0)
        self.curves_yz = {k: p.plot(pen=_make_pen(k), name=LABELS[k]) for k in keys}

    def _build_3d_window(self):
        self.win_3d = gl.GLViewWidget()
        self.win_3d.setWindowTitle("3D Isometric View")
        self.win_3d.resize(900, 900)
        self.win_3d.setBackgroundColor("w")          # white to match 2-D windows
 
        # Camera centred on the actual data volume
        self.win_3d.opts["center"] = QtGui.QVector3D(0.0, 0.0, Z_CENTER)
        data_span = max(XY_RANGE * 2, abs(Z_MAX - Z_MIN))
        self.win_3d.setCameraPosition(
            distance=data_span * 1.3,
            elevation=30,
            azimuth=225,
        )
 
        # ── Axis Origin (Corner) ─────────────────────────────────────
        # Set the origin to the side bounding box corner
        cx, cy, cz = -80.0, -80.0, -430.0
        x_max, y_max, z_max = 80.0, 80.0, Z_MAX
 
        tick_len   = 3.0          # half-length of each tick cross-stroke (mm)
        lbl_offset = tick_len * 4 # label pull-back from axis (mm)
 
        # ── helper: add a GLTextItem ─────────────────────────────────
        def _txt(pos, text, qcolor):
            item = gl.GLTextItem(
                pos=np.array(pos, dtype=np.float32),
                text=text,
                color=qcolor,
            )
            self.win_3d.addItem(item)
 
        # ── helper: batch tick-mark segments ────────────────────────
        def _ticks(pts_list, color_f):
            self.win_3d.addItem(gl.GLLinePlotItem(
                pos=np.array(pts_list, dtype=np.float32),
                color=color_f, width=1.2, antialias=True, mode="lines",
            ))
 
        # ── Grid ─────────────────────────────────────────────────────
        grid = gl.GLGridItem()
        grid.setSize(x=160, y=160, z=1) # Span covers -80 to +80
        grid.setSpacing(x=10, y=10, z=10)
        grid.translate(0, 0, cz) # Drop the grid down to the new Z floor
        self.win_3d.addItem(grid)
 
        # ── X axis (red) ─────────────────────────────────────────────
        self.win_3d.addItem(gl.GLLinePlotItem(
            pos=np.array([[cx, cy, cz],
                          [x_max, cy, cz]], dtype=np.float32),
            color=(0.8, 0, 0, 0.8), width=1.5, antialias=True, mode="lines",
        ))
        x_vals = np.arange(int(cx), int(x_max) + 1, 20)
        tick_pts = []
        for v in x_vals:
            tick_pts += [[v, cy - tick_len, cz], [v, cy + tick_len, cz]]
            _txt([v, cy - lbl_offset, cz], str(v), QtGui.QColor(180, 0, 0))
        _ticks(tick_pts, (0.8, 0, 0, 0.6))
        _txt([0, cy-25, cz], "X (mm)", QtGui.QColor(200, 0, 0))
 
        # ── Y axis (green) ───────────────────────────────────────────
        self.win_3d.addItem(gl.GLLinePlotItem(
            pos=np.array([[cx, cy, cz],
                          [cx, y_max, cz]], dtype=np.float32),
            color=(0, 0.6, 0, 0.8), width=1.5, antialias=True, mode="lines",
        ))
        y_vals = np.arange(int(cy), int(y_max) + 1, 20)
        tick_pts = []
        for v in y_vals:
            tick_pts += [[cx - tick_len, v, cz], [cx + tick_len, v, cz]]
            # Shifted Z down by 8 so it sits below the corner vertex
            _txt([cx - lbl_offset, v, cz - 8], str(v), QtGui.QColor(0, 140, 0))
            
        _ticks(tick_pts, (0, 0.6, 0, 0.6))
        _txt([cx-35,0, cz], "Y (mm)", QtGui.QColor(0, 150, 0))
 
        # ── Z axis (blue) ────────────────────────────────────────────
        z_axis_y = 80.0  # <--- Set your new Y location for the Z-axis here (e.g., y_max)

        self.win_3d.addItem(gl.GLLinePlotItem(
            pos=np.array([[cx, z_axis_y, cz], [cx, z_axis_y, z_max]], dtype=np.float32),
            color=(0, 0, 0.9, 0.8), width=1.5, antialias=True, mode="lines",
        ))
        
        z_vals = np.arange(int(cz), int(z_max) + 1, 10)
        tick_pts = []
        for v in z_vals:
            # Draw tick marks crossing the Z axis at the new Y location
            tick_pts += [[cx - tick_len, z_axis_y, v], [cx + tick_len, z_axis_y, v]]
            # Position the text labels
            _txt([cx - lbl_offset - 10, z_axis_y+10, v], str(int(v)), QtGui.QColor(0, 0, 190))
            
        _ticks(tick_pts, (0, 0, 0.9, 0.6))
        
        # Position the "Z (mm)" axis title
        _txt([cx-50, z_axis_y, -380], "Z (mm)", QtGui.QColor(0, 0, 200))
 
        # ── Data trace lines ─────────────────────────────────────────
        # Initialise with a 2-point dummy inside the working volume;
        # line_strip mode requires ≥ 2 points or GL will error.
        _dummy = np.array([[0, 0, Z_CENTER], [0, 0, Z_CENTER]], dtype=np.float32)
        self.lines_3d = {}
        for k in self._keys:
            item = gl.GLLinePlotItem(
                pos=_dummy,
                color=_color_gl(k),
                width=LINE_WIDTH + 0.5,
                antialias=True,
                mode="line_strip",
            )
            self.win_3d.addItem(item)
            self.lines_3d[k] = item
        # ── Legend Overlay (2D Floating HUD) ─────────────────────────
        self.legend_3d = QtWidgets.QLabel(self.win_3d)
        # Let mouse drag events pass through the legend so it doesn't block camera rotation
        self.legend_3d.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        self.legend_3d.setStyleSheet("""
            QLabel {
                background-color: rgba(255, 255, 255, 210);
                border: 1px solid #aaa;
                border-radius: 4px;
                padding: 6px 10px;
                font-family: sans-serif;
                font-size: 12px;
                color: black;
            }
        """)
        
        # Build HTML for the legend using the exact colors and labels
        html_lines = ["<div style='margin-bottom: 2px;'><b>Legend</b></div>"]
        for k in self._keys:
            r, g, b, _ = COLORS[k]
            # Use a colored square text character
            html_lines.append(
                f"<span style='color: rgb({r},{g},{b}); font-size: 16px;'>■</span> {LABELS[k]}"
            )
            
        self.legend_3d.setText("<br>".join(html_lines))
        self.legend_3d.move(15, 15)  # Anchor to top-left corner
        self.legend_3d.adjustSize()

    def _build_rpy_window(self):
        self.win_rpy = pg.GraphicsLayoutWidget(title="End-Effector Rotation vs Time")
        self.win_rpy.resize(900, 600)

        self.plot_pitch = self._styled_plot(
            self.win_rpy, "Pitch vs Time", "Time (s)", "Pitch (deg)"
        )
        self.win_rpy.nextRow()
        self.plot_yaw = self._styled_plot(
            self.win_rpy, "Yaw vs Time", "Time (s)", "Yaw (deg)"
        )

        self.curves_pitch = {
            k: self.plot_pitch.plot(pen=_make_pen(k), name=LABELS[k])
            for k in self._keys
        }
        self.curves_yaw = {
            k: self.plot_yaw.plot(pen=_make_pen(k), name=LABELS[k])
            for k in self._keys
        }

    # ------------------------------------------------------------------
    # Main update tick (Qt timer, runs on GUI thread)
    # ------------------------------------------------------------------

    def _tick(self):
        # 1. Sample TFs (fast — just a TF buffer lookup)
        self.node.sample_all()

        # 2. Get thread-safe snapshot
        snap = self.node.snapshot()

        MM = 1e3
        t0 = None
        for k in self._keys:
            ts = snap[k]["t"]
            if ts and (t0 is None or ts[0] < t0):
                t0 = ts[0]
        if t0 is None:
            return

        t_latest = None
        for k in self._keys:
            ts = snap[k]["t"]
            if ts and (t_latest is None or ts[-1] > t_latest):
                t_latest = ts[-1]
        t_latest_rel = (t_latest - t0) if t_latest is not None else 0.0

        if t_latest_rel < self.time_window_s:
            t_win_min, t_win_max = 0.0, self.time_window_s
        else:
            t_win_min = t_latest_rel - self.time_window_s
            t_win_max = t_latest_rel

        # 3. Push data to curves
        all_pitch, all_yaw = [], []

        for k in self._keys:
            d       = snap[k]
            xs      = np.array(d["x"]) * MM
            ys      = np.array(d["y"]) * MM
            zs      = np.array(d["z"]) * MM
            ts      = np.array(d["t"])
            pitches = np.degrees(np.array(d["pitch"]))
            yaws    = np.degrees(np.array(d["yaw"]))
            n       = len(xs)

            # 2-D planes
            if n:
                self.curves_xy[k].setData(xs, ys)
                self.curves_xz[k].setData(xs, zs)
                self.curves_yz[k].setData(ys, zs)
            else:
                self.curves_xy[k].setData([], [])
                self.curves_xz[k].setData([], [])
                self.curves_yz[k].setData([], [])

            # 3-D — GLLinePlotItem in line_strip mode requires ≥ 2 points
            if n >= 2:
                pts = np.column_stack([xs, ys, zs]).astype(np.float32)
                self.lines_3d[k].setData(pos=pts)
            elif n == 1:
                # Duplicate the single point so GL doesn't error
                pts = np.array([[xs[0], ys[0], zs[0]],
                                [xs[0], ys[0], zs[0]]], dtype=np.float32)
                self.lines_3d[k].setData(pos=pts)
            # n == 0: leave the harmless dummy in place

            # Time-series
            if n:
                t_rel = ts - t0
                self.curves_pitch[k].setData(t_rel, pitches)
                self.curves_yaw[k].setData(t_rel, yaws)
                all_pitch.extend(pitches.tolist())
                all_yaw.extend(yaws.tolist())
            else:
                self.curves_pitch[k].setData([], [])
                self.curves_yaw[k].setData([], [])

        # Time axis window
        self.plot_pitch.setXRange(t_win_min, t_win_max, padding=0)
        self.plot_yaw.setXRange(t_win_min, t_win_max, padding=0)

        # Auto y-limits with padding
        def _auto_ylim(plot, vals, fallback=(-5, 5)):
            if not vals:
                plot.setYRange(*fallback, padding=0)
                return
            lo, hi   = min(vals), max(vals)
            span     = hi - lo
            pad      = max(span * 0.1, 1.0) if span > 0 else max(abs(lo) * 0.05, 1.0)
            plot.setYRange(lo - pad, hi + pad, padding=0)

        _auto_ylim(self.plot_pitch, all_pitch)
        _auto_ylim(self.plot_yaw,   all_yaw)

    # ------------------------------------------------------------------
    # Screenshot helper
    # ------------------------------------------------------------------

    def _save_snapshots(self) -> list[str]:
        """Capture each window to a color PNG.

        Returns the list of all saved paths.
        """
        ts      = time.strftime("%Y%m%d_%H%M%S")
        out_dir = os.path.expanduser("~/ee_tf_snapshots")
        os.makedirs(out_dir, exist_ok=True)

        saved = []

        windows = {
            "01_XY":  self.win_xy,
            "02_XZ":  self.win_xz,
            "03_YZ":  self.win_yz,
            "04_3D":  self.win_3d,
            "05_RPY": self.win_rpy,
        }

        for label, win in windows.items():
            # ── 1. grab a QImage from the widget ─────────────────────
            if isinstance(win, gl.GLViewWidget):
                # grabFramebuffer() takes NO keyword arguments in this version
                qimg_color = win.grabFramebuffer()
            else:
                qimg_color = win.grab().toImage()

            # ── 2. save color copy ────────────────────────────────────
            color_path = os.path.join(out_dir, f"ee_tf_{label}_color_{ts}.png")
            if qimg_color.save(color_path):
                saved.append(color_path)
            else:
                print(f"[WARN] Failed to save {color_path}", file=sys.stderr)

        print(f"[INFO] {len(saved)} images saved to {out_dir}")
        return saved

    # ------------------------------------------------------------------
    def show_all(self):
        for w in [self.win_xy, self.win_xz, self.win_yz,
                  self.win_3d, self.win_rpy]:
            w.show()

    def exec(self):
        self.show_all()
        self.app.exec()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Realtime TF plot (PyQtGraph) — XY/XZ/YZ/3D/RPY"
    )
    parser.add_argument("--parent-frame",  default="delta_robot/world_link")
    parser.add_argument("--sim",           default="delta_robot/end_effector_pin")
    parser.add_argument("--commanded",     default="delta_robot/commanded_end_effector_pin")
    parser.add_argument("--ee",            default="ee_link")
    parser.add_argument("--calculated",    default="delta_robot/calculated_fk_end_effector_pin")
    parser.add_argument("--actual",        default="delta_robot/actual_fk_end_effector_pin")
    parser.add_argument("--max-points",    type=int,   default=1000)
    parser.add_argument("--interval-ms",   type=int,   default=50)
    parser.add_argument("--time-window",   type=float, default=5.0)
    args = parser.parse_args()

    frames = {
        "sim":        args.sim,
        "commanded":  args.commanded,
        "ee":         args.ee,
        "calculated": args.calculated,
        "actual":     args.actual,
    }

    rclpy.init()
    node = TfSamplerNode(args.parent_frame, frames, args.max_points)
    node.get_start_position(timeout=2.0)

    gui = PlotApp(node, interval_ms=args.interval_ms, time_window_s=args.time_window)

    def _shutdown(*_):
        gui._timer.stop()
        QtWidgets.QApplication.quit()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)

    # Let Ctrl+C work even while Qt is blocking
    sigint_timer = QtCore.QTimer()
    sigint_timer.start(200)
    sigint_timer.timeout.connect(lambda: None)

    try:
        gui.exec()
    except KeyboardInterrupt:
        _shutdown()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()