#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException, Buffer, TransformListener
import numpy as np


class DeltaCalipers(Node):

    def __init__(self):
        super().__init__('delta_calipers')
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.measure)
        self.get_logger().info('Delta Digital Caliper active...')

    # ── TF helpers ───────────────────────────────────────────────────────────

    def get_tf_translation(self, parent_frame, child_frame):
        """Full XYZ translation of child expressed in parent frame."""
        try:
            t = self.tf_buffer.lookup_transform(
                parent_frame, child_frame, rclpy.time.Time()
            ).transform.translation
            return np.array([t.x, t.y, t.z])
        except TransformException as e:
            self.get_logger().warn(f'TF miss [{parent_frame} → {child_frame}]: {e}')
            return None

    # ── Logging helpers ──────────────────────────────────────────────────────

    def _row(self, label, dist_m):
        self.get_logger().info(f'  {label:<50} {dist_m * 1000.0:8.2f} mm')

    def _xyz(self, label, t):
        self.get_logger().info(
            f'  {label:<50} '
            f'x={t[0]*1000:7.2f}  y={t[1]*1000:7.2f}  z={t[2]*1000:7.2f}  mm'
        )

    # ── Main measurement routine ─────────────────────────────────────────────

    def measure(self):

        PREFIX = 'delta_robot'

        # All relative translations
        t_world_to_horn1   = self.get_tf_translation(f'{PREFIX}/world_link',    f'{PREFIX}/servo_1_horn')
        t_horn1_to_pivot1  = self.get_tf_translation(f'{PREFIX}/servo_1_horn',  f'{PREFIX}/pivot_arm_1')
        t_pivot1_to_rc1    = self.get_tf_translation(f'{PREFIX}/pivot_arm_1',   f'{PREFIX}/rod_cap_1')
        t_rc1_to_pivot4    = self.get_tf_translation(f'{PREFIX}/rod_cap_1',     f'{PREFIX}/pivot_arm_4')
        t_ee_to_pivot4     = self.get_tf_translation(f'{PREFIX}/end_effector',  f'{PREFIX}/pivot_arm_4')

        if any(t is None for t in [
            t_world_to_horn1, t_horn1_to_pivot1,
            t_pivot1_to_rc1, t_rc1_to_pivot4, t_ee_to_pivot4
        ]):
            self.get_logger().warn('Some TF frames not yet available, skipping.')
            return

        sep = '─' * 70
        self.get_logger().info(f'\n{sep}')
        self.get_logger().info(f'  {"MEASUREMENT":<50} {"VALUE":>10}')
        self.get_logger().info(sep)

        # 1. world_link → servo_1_horn   (X = radial reach across base)
        self._xyz('1. world → horn1  [full xyz]',          t_world_to_horn1)
        self._row('   └─ X axis only',                      abs(t_world_to_horn1[0]))

        self.get_logger().info('')

        # 2. servo_1_horn → pivot_arm_1  (X = bicep horizontal reach)
        self._xyz('2. horn1 → pivot_arm_1  [full xyz]',    t_horn1_to_pivot1)
        self._row('   └─ X axis only  (bicep horizontal)',  abs(t_horn1_to_pivot1[0]))

        self.get_logger().info('')

        # 3. pivot_arm_1 → rod_cap_1  (sanity check — should be small Z offset)
        self._xyz('3. pivot_arm_1 → rod_cap_1  [full xyz check]', t_pivot1_to_rc1)

        self.get_logger().info('')

        # 4. rod_cap_1 → pivot_arm_4  (Y = forearm length along rod axis)
        self._xyz('4. rod_cap_1 → pivot_arm_4  [full xyz]', t_rc1_to_pivot4)
        self._row('   └─ Y axis only  (forearm length)',     abs(t_rc1_to_pivot4[1]))

        self.get_logger().info('')

        # 5. end_effector → pivot_arm_4  (Z = vertical height in EE frame)
        self._xyz('5. end_effector → pivot_arm_4  [full xyz]', t_ee_to_pivot4)
        self._row('   └─ Z axis only  (EE vertical depth)',      abs(t_ee_to_pivot4[2]))

        self.get_logger().info(sep)


def main(args=None):
    rclpy.init(args=args)
    node = DeltaCalipers()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()