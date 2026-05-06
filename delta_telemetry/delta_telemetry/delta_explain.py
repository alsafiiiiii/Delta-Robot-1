import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import RadioButtons, TextBox
from matplotlib.patches import Wedge
import matplotlib.gridspec as gridspec


# --- ROBOT LOGIC CLASS ---
class InteractiveDelta:
    def __init__(self):
        # --- Robot Geometry ---
        self.L = 0.105  # Upper Arm (Bicep)
        self.l = 0.205  # Forearm (Rod)
        self.R = 0.104  # Base Radius
        self.r = 0.040  # Effector Radius
        self.phi = np.array([0, 2 * np.pi / 3, 4 * np.pi / 3])

        # Geometry pre-calculation
        self.base_x = self.R * np.cos(self.phi)
        self.base_y = self.R * np.sin(self.phi)
        self.ee_x_local = self.r * np.cos(self.phi)
        self.ee_y_local = self.r * np.sin(self.phi)

    def get_math_state(self, i, x, y, z):
        cos_phi = np.cos(self.phi[i])
        sin_phi = np.sin(self.phi[i])

        # Step 0: Real World Joint Location
        joint_x = x + self.r * cos_phi
        joint_y = y + self.r * sin_phi

        # Step 1: Coordinate Rotation
        x_prime = x * cos_phi + y * sin_phi
        y_prime = -x * sin_phi + y * cos_phi

        # Step 2: Virtual Parameters
        J = (self.R - self.r) - x_prime
        val_under_sqrt = self.l**2 - y_prime**2
        l_eff = np.sqrt(val_under_sqrt) if val_under_sqrt >= 0 else 0

        # Step 3: Intersection Logic
        d2 = J**2 + z**2
        d = np.sqrt(d2)

        a = (self.L**2 - l_eff**2 + d2) / (2 * d) if d > 0 else 0
        h_arg = self.L**2 - a**2
        h = np.sqrt(h_arg) if h_arg >= 0 else 0

        dx = -J / d
        dz = z / d

        # Elbow coordinates in 2D Plane
        e2d_x = a * dx - h * dz
        e2d_z = a * dz + h * dx

        # Step 4: Angle
        theta_rad = np.arctan2(e2d_z, e2d_x)
        theta_deg = np.degrees(theta_rad)

        return {
            "joint_3d": (joint_x, joint_y, z),
            "x_prime": x_prime,
            "y_prime": y_prime,
            "J": J,
            "l_eff": l_eff,
            "target_2d": (-J, z),
            "elbow_2d": (e2d_x, e2d_z),
            "theta_deg": theta_deg,
            "valid": val_under_sqrt >= 0 and h_arg >= 0,
        }

    def solve_ik(self, x, y, z):
        elbows = []
        valid_global = True
        for i in range(3):
            state = self.get_math_state(i, x, y, z)
            if not state["valid"]:
                valid_global = False
                break

            ex_local = state["elbow_2d"][0]
            ez_local = state["elbow_2d"][1]

            bx = self.R * np.cos(self.phi[i])
            by = self.R * np.sin(self.phi[i])

            elbow_x = bx + ex_local * np.cos(self.phi[i])
            elbow_y = by + ex_local * np.sin(self.phi[i])
            elbow_z = ez_local
            elbows.append([elbow_x, elbow_y, elbow_z])

        if not valid_global:
            return None
        return np.array(elbows)


# --- VISUALIZATION SETUP ---
robot = InteractiveDelta()

# Initialize Figure with Custom Grid Layout
fig = plt.figure(figsize=(16, 9), facecolor="#f0f0f0")
gs = gridspec.GridSpec(
    2, 3, width_ratios=[1.5, 1, 0.5], height_ratios=[1, 1], wspace=0.2, hspace=0.2
)

# 1. 3D View (Left Column, Spanning both rows)
ax_3d = fig.add_subplot(gs[:, 0], projection="3d", proj_type="ortho")
ax_3d.set_facecolor("white")

# 2. 2D Math View (Middle Top)
ax_2d = fig.add_subplot(gs[0, 1])
ax_2d.set_facecolor("white")

# 3. Equation Text Area (Middle Bottom)
ax_eq = fig.add_subplot(gs[1, 1])
ax_eq.axis("off")

# 4. Control Panel (Right Column)
# We will create sub-axes within this area manually for buttons
panel_bg = fig.add_subplot(gs[:, 2])
panel_bg.axis("off")
panel_bg.set_facecolor("#e0e0e0")  # Gray background for controls

# --- GRAPHICS HOLDERS ---
lines_bicep = []
lines_forearm = []
lines_plate = []
scatter_elbows = None
proj_plane = None
proj_offset_line = None
text_labels_3d = []

# 2D Holders
(math_motor,) = ax_2d.plot([], [], "ko", markersize=8, label="Motor Pivot")
(math_bicep,) = ax_2d.plot([], [], "r-", lw=3, label="Bicep")
(math_forearm,) = ax_2d.plot([], [], "c-", lw=3, label="Forearm")
(math_target,) = ax_2d.plot([], [], "bo", label="Proj. Joint")
(math_elbow,) = ax_2d.plot([], [], "ro", label="Elbow")
angle_wedge = None

# Initial State
active_arm_idx = 0
current_pos = [0.0, 0.0, -0.25]


def init_plot():
    global scatter_elbows, proj_plane, proj_offset_line, angle_wedge

    # --- 3D Init ---
    for _ in range(3):
        (lb,) = ax_3d.plot([], [], [], "r-", lw=4, solid_capstyle="round")
        (lf,) = ax_3d.plot([], [], [], "c-", lw=2)
        (lp,) = ax_3d.plot([], [], [], "k--", lw=1)
        lines_bicep.append(lb)
        lines_forearm.append(lf)
        lines_plate.append(lp)

    # Draw Base
    bx = np.append(robot.base_x, robot.base_x[0])
    by = np.append(robot.base_y, robot.base_y[0])
    ax_3d.plot(bx, by, [0] * 4, "k-", lw=2)

    scatter_elbows = ax_3d.scatter([], [], [], color="black", s=50)
    (proj_offset_line,) = ax_3d.plot(
        [], [], [], color="purple", linestyle="--", lw=2, zorder=10
    )

    ax_3d.set_xlim(-0.25, 0.25)
    ax_3d.set_ylim(-0.25, 0.25)
    ax_3d.set_zlim(-0.4, 0.1)
    ax_3d.set_xlabel("X")
    ax_3d.set_ylabel("Y")
    ax_3d.set_zlabel("Z")
    ax_3d.set_title("3D Orthographic View", fontsize=12, fontweight="bold", pad=20)

    # --- 2D Init ---
    ax_2d.set_aspect("equal")
    ax_2d.set_xlim(-0.3, 0.1)
    ax_2d.set_ylim(-0.4, 0.1)
    ax_2d.grid(True, alpha=0.3, linestyle="--")
    ax_2d.set_title("2D Projection Plane", fontsize=12, fontweight="bold")
    ax_2d.set_xlabel("Radial Distance (m)")
    ax_2d.set_ylabel("Height Z (m)")

    angle_wedge = Wedge((0, 0), 0.05, 0, 0, color="green", alpha=0.3)
    ax_2d.add_patch(angle_wedge)
    ax_2d.legend(loc="upper left", fontsize="x-small", frameon=True)


def draw_projection_plane(arm_idx):
    global proj_plane
    if proj_plane:
        proj_plane.remove()

    h_range = np.linspace(-0.4, 0.1, 2)
    r_range = np.linspace(0, 0.3, 2)
    H, R = np.meshgrid(h_range, r_range)

    phi = robot.phi[arm_idx]
    X = R * np.cos(phi)
    Y = R * np.sin(phi)
    Z = H

    proj_plane = ax_3d.plot_surface(X, Y, Z, color="gray", alpha=0.15)


def update_equations(state, i, x, y):
    ax_eq.clear()
    ax_eq.axis("off")

    y_p = state["y_prime"]
    l_eff = state["l_eff"]
    theta = state["theta_deg"]
    jx, jy, jz = state["joint_3d"]

    text_color = "#333333" if state["valid"] else "red"

    s = f"Active Calculation: Arm {i + 1}\n"
    s += "==============================\n\n"

    s += "Step 0: Find Ball Joint Location\n"
    s += f"Joint = ({jx:.3f}, {jy:.3f}, {jz:.3f})\n\n"

    s += "Step 1: Perpendicular Offset (y')\n"
    s += f"Distance from plane: {y_p:.4f} m\n"
    s += "(Shown as Purple Line in 3D)\n\n"

    s += "Step 2: Effective Rod Length\n"
    s += r"$l_{eff} = \sqrt{l^2 - y'^2}$" + f" = {l_eff:.4f} m\n\n"

    s += "Step 3: Solve 2D Intersection\n"
    s += f"Circle 1 Radius: {robot.L}\n"
    s += f"Circle 2 Radius: {l_eff:.4f}\n\n"

    s += "Step 4: Motor Angle\n"
    s += r"$\theta = \mathbf{" + f"{theta:.2f}" + r"^{\circ}}$"

    if not state["valid"]:
        s += "\n\n!!! UNREACHABLE !!!"

    ax_eq.text(
        0.05,
        0.95,
        s,
        transform=ax_eq.transAxes,
        verticalalignment="top",
        fontsize=11,
        color=text_color,
        family="monospace",
        linespacing=1.4,
    )


def update_plot(val=None):
    global text_labels_3d
    x, y, z = current_pos

    elbows = robot.solve_ik(x, y, z)
    state = robot.get_math_state(active_arm_idx, x, y, z)

    # Clear 3D labels
    for txt in text_labels_3d:
        txt.remove()
    text_labels_3d = []

    if elbows is None:
        ax_3d.set_title("TARGET UNREACHABLE", color="red", fontweight="bold")
    else:
        ax_3d.set_title(f"3D View (Pos: {x:.3f}, {y:.3f}, {z:.3f})", fontweight="bold")

        # Draw Robot
        scatter_elbows._offsets3d = (elbows[:, 0], elbows[:, 1], elbows[:, 2])
        for i in range(3):
            bx, by = robot.base_x[i], robot.base_y[i]
            px = x + robot.ee_x_local[i]
            py = y + robot.ee_y_local[i]
            lines_bicep[i].set_data_3d(
                [bx, elbows[i, 0]], [by, elbows[i, 1]], [0, elbows[i, 2]]
            )
            lines_forearm[i].set_data_3d(
                [elbows[i, 0], px], [elbows[i, 1], py], [elbows[i, 2], z]
            )

            # Plate
            next_i = (i + 1) % 3
            px_next = x + robot.ee_x_local[next_i]
            py_next = y + robot.ee_y_local[next_i]
            lines_plate[i].set_data_3d([px, px_next], [py, py_next], [z, z])

    # --- SPECIFIC VISUALIZATION ---
    draw_projection_plane(active_arm_idx)

    jx, jy, jz = state["joint_3d"]
    phi = robot.phi[active_arm_idx]

    radial_dist = state["x_prime"] + robot.r
    proj_x = radial_dist * np.cos(phi)
    proj_y = radial_dist * np.sin(phi)

    proj_offset_line.set_data_3d([jx, proj_x], [jy, proj_y], [z, z])

    # 3D Labels
    t1 = ax_3d.text(jx, jy, z - 0.02, "Joint", color="blue", fontsize=8)
    mid_x, mid_y = (jx + proj_x) / 2, (jy + proj_y) / 2
    t2 = ax_3d.text(
        mid_x, mid_y, z + 0.01, "y'", color="purple", fontsize=10, fontweight="bold"
    )
    text_labels_3d.extend([t1, t2])

    # Update 2D
    math_motor.set_data([0], [0])
    e2d = state["elbow_2d"]
    t2d = state["target_2d"]
    math_bicep.set_data([0, e2d[0]], [0, e2d[1]])
    math_target.set_data([t2d[0]], [t2d[1]])
    math_forearm.set_data([e2d[0], t2d[0]], [e2d[1], t2d[1]])
    math_elbow.set_data([e2d[0]], [e2d[1]])

    # Angle
    theta = state["theta_deg"]
    if theta < 0:
        angle_wedge.set_theta1(theta)
        angle_wedge.set_theta2(0)
    else:
        angle_wedge.set_theta1(0)
        angle_wedge.set_theta2(theta)

    update_equations(state, active_arm_idx, x, y)
    fig.canvas.draw_idle()


# --- CONTROL PANEL UI ---

# 1. Title
fig.text(0.85, 0.92, "CONTROLS", fontsize=14, fontweight="bold", ha="center")


# 2. Text Boxes for Position
def submit_x(text):
    try:
        current_pos[0] = float(text)
        update_plot()
    except Exception:
        pass


def submit_y(text):
    try:
        current_pos[1] = float(text)
        update_plot()
    except Exception:
        pass


def submit_z(text):
    try:
        current_pos[2] = float(text)
        update_plot()
    except Exception:
        pass


box_height = 0.04
box_width = 0.1
left_align = 0.82

ax_box_x = plt.axes([left_align, 0.80, box_width, box_height])
text_box_x = TextBox(ax_box_x, "X : ", initial=str(current_pos[0]))
text_box_x.on_submit(submit_x)

ax_box_y = plt.axes([left_align, 0.74, box_width, box_height])
text_box_y = TextBox(ax_box_y, "Y : ", initial=str(current_pos[1]))
text_box_y.on_submit(submit_y)

ax_box_z = plt.axes([left_align, 0.68, box_width, box_height])
text_box_z = TextBox(ax_box_z, "Z : ", initial=str(current_pos[2]))
text_box_z.on_submit(submit_z)

# 3. Arm Selection (Radio Buttons)
fig.text(0.85, 0.58, "Select View", fontsize=12, fontweight="bold", ha="center")
ax_radio = plt.axes([0.80, 0.45, 0.12, 0.12], facecolor="#f0f0f0")
radio = RadioButtons(ax_radio, ("Arm 1", "Arm 2", "Arm 3"))


def change_arm(label):
    global active_arm_idx
    if label == "Arm 1":
        active_arm_idx = 0
        ax_3d.view_init(elev=0, azim=-90)
    elif label == "Arm 2":
        active_arm_idx = 1
        ax_3d.view_init(elev=0, azim=30)
    else:
        active_arm_idx = 2
        ax_3d.view_init(elev=0, azim=150)
    update_plot()


radio.on_clicked(change_arm)

# 4. Instructions
instr_text = (
    "INSTRUCTIONS:\n"
    "- Enter X, Y, Z coordinates\n"
    "  to move effector.\n"
    "- Select Arm to rotate view\n"
    "  and see projection math.\n"
    "- Purple line = Offset.\n"
)
fig.text(
    0.85,
    0.25,
    instr_text,
    fontsize=9,
    ha="center",
    va="center",
    bbox=dict(facecolor="white", alpha=0.8, edgecolor="gray"),
)


# Start
ax_3d.view_init(elev=0, azim=-90)
init_plot()
update_plot()
plt.show()
