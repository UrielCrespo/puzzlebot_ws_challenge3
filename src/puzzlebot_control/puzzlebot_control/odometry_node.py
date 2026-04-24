#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import numpy as np

class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')

        # 3) Parámetros del Puzzlebot
        self.r = 0.0556
        self.l = 0.1746

        # 3b) ── NUEVOS: Filtro y deadzone ────────────────────────────
        # α del filtro pasa-bajas exponencial (EMA)
        # α=1.0 sin filtro, α=0.3 suave, α=0.1 agresivo
        self.declare_parameter('alpha_filter',     0.3)
        # Deadzone: velocidades angulares menores se consideran ruido
        self.declare_parameter('encoder_deadzone', 0.05)  # rad/s

        # 4) Pose inicial
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # 5) Velocidades angulares — crudas y filtradas
        self.phi_r_raw = None
        self.phi_l_raw = None
        self.phi_r     = 0.0   # valor filtrado
        self.phi_l     = 0.0   # valor filtrado

        # 6) Publisher de pose
        self.pose_pub = self.create_publisher(
            Pose2D, '/pose', qos.qos_profile_sensor_data
        )

        # 7) Suscriptores a encoders
        self.create_subscription(
            Float32, '/VelocityEncR', self.wr_callback,
            qos.qos_profile_sensor_data
        )
        self.create_subscription(
            Float32, '/VelocityEncL', self.wl_callback,
            qos.qos_profile_sensor_data
        )
        self.create_subscription(
            Pose2D, '/reset_pose', self.reset_pose_callback, 10
        )

        # 8) ── CAMBIADO: Timer a 20 Hz ────────────────────────────────
        # 50 Hz muestreaba demasiado rápido — introducía ruido sin info
        # tau_lineal ~ 0.62s → dinámica ~1.6 Hz → 20 Hz es 12× suficiente
        self.timer_period = 0.05  # 20 Hz (antes 0.02 = 50 Hz)

        # 9) Tiempo anterior
        self.last_time = self.get_clock().now()

        # 10) Timer
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        self.get_logger().info(
            f'OdometryNode started: publishing /pose at {1/self.timer_period:.0f} Hz'
        )

    # ── Callbacks de encoders — solo guardan valor crudo ──────────────
    def wr_callback(self, msg: Float32):
        self.phi_r_raw = float(msg.data)

    def wl_callback(self, msg: Float32):
        self.phi_l_raw = float(msg.data)

    # ── Reset pose ────────────────────────────────────────────────────
    def reset_pose_callback(self, msg: Pose2D):
        self.x     = msg.x
        self.y     = msg.y
        self.theta = msg.theta
        self.get_logger().info(
            f'Pose reseteada a: x={msg.x:.3f} y={msg.y:.3f} θ={msg.theta:.3f}'
        )

    # ── Filtro EMA con deadzone ───────────────────────────────────────
    def apply_filter(self, raw, filtered, alpha, deadzone):
        # 1) Deadzone: ruido cerca de cero → cero
        if abs(raw) < deadzone:
            raw = 0.0
        # 2) Filtro pasa-bajas exponencial
        return alpha * raw + (1.0 - alpha) * filtered

    # ── Timer principal ───────────────────────────────────────────────
    def timer_cb(self):
        if self.phi_r_raw is None or self.phi_l_raw is None:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt == 0.0:
            return

        # ── Leer parámetros (permite tuning en vivo) ─────────────────
        alpha    = self.get_parameter('alpha_filter').value
        deadzone = self.get_parameter('encoder_deadzone').value

        # ── Aplicar deadzone + filtro EMA a ambos encoders ───────────
        self.phi_r = self.apply_filter(self.phi_r_raw, self.phi_r, alpha, deadzone)
        self.phi_l = self.apply_filter(self.phi_l_raw, self.phi_l, alpha, deadzone)

        # ── Modelo cinemático diferencial ────────────────────────────
        v = self.r * (self.phi_r + self.phi_l) / 2.0
        w = self.r * (self.phi_r - self.phi_l) / self.l

        # ── Integración de Euler ─────────────────────────────────────
        self.x     += v * np.cos(self.theta) * dt
        self.y     += v * np.sin(self.theta) * dt
        self.theta += w * dt

        # ── Normalizar θ entre -π y π ────────────────────────────────
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        # ── Publicar pose ────────────────────────────────────────────
        pose_msg = Pose2D()
        pose_msg.x     = self.x
        pose_msg.y     = self.y
        pose_msg.theta = self.theta
        self.pose_pub.publish(pose_msg)

        self.get_logger().info(
            f'x={self.x:.3f}  y={self.y:.3f}  θ={np.degrees(self.theta):.2f}° '
            f'| φR={self.phi_r:.2f} φL={self.phi_l:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()