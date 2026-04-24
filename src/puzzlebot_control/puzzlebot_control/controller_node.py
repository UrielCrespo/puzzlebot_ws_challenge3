#!/usr/bin/env python3

import rclpy
from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool
from puzzlebot_msgs.msg import GoalPose
import numpy as np

def wrap_to_pi(theta):
    result = np.fmod((theta + np.pi), (2 * np.pi))
    if result < 0:
        result += 2 * np.pi
    return result - np.pi

# ── Máquina de estados ────────────────────────────────────────────────
GIRANDO      = 0
AVANZANDO    = 1
LISTO        = 2
SETTLING     = 3
SETTLING_WP  = 4


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        # 3) Ganancias PID (calculadas en MATLAB, NO MODIFICAR)
        self.declare_parameter('kp_v', 0.5013)
        self.declare_parameter('ki_v', 1.3311)
        self.declare_parameter('kd_v', 0.0000)
        self.declare_parameter('kp_w', 0.4775)
        self.declare_parameter('ki_w', 1.2013)
        self.declare_parameter('kd_w', 0.0000)

        # 4) Límites y tolerancias
        self.declare_parameter('v_max', 0.25)
        self.declare_parameter('w_max', 0.15)
        self.declare_parameter('dist_tolerance',  0.1)
        self.declare_parameter('angle_tolerance', 0.08)

        # 4b) Rampa y frenado lineal
        self.declare_parameter('zona_frenado', 0.4)
        self.declare_parameter('ramp_rate',    0.4)

        # 4c) Corrección angular durante avance
        self.declare_parameter('corr_angular', 0.3)

        # 4d) Frenado angular y settling
        self.declare_parameter('zona_frenado_ang', 0.9)
        self.declare_parameter('w_min',            0.12)
        self.declare_parameter('omega_tol',        0.08)
        self.declare_parameter('settle_time',      0.20)
        self.declare_parameter('settle_time_wp',   0.25)

        # 5) Goal — ahora viene del path_generator via /goal
        # Ya NO hay waypoints hardcodeados
        self.x_goal        = None
        self.y_goal        = None
        self.goal_received = False
        self.goal_label    = ''
        self.goal_is_final = False

        # 6) Pose actual
        self.x_r       = 0.0
        self.y_r       = 0.0
        self.theta_r   = 0.0
        self.pose_received = False

        # 7) PID lineal
        self.integral_v  = 0.0
        self.prev_ed     = 0.0

        # 8) PID angular
        self.integral_w  = 0.0
        self.prev_etheta = 0.0

        # 9) Tiempo
        self.last_time = self.get_clock().now()

        # 10) Máquina de estados
        self.state        = LISTO   # espera hasta recibir primer goal
        self.done         = False
        self.v_ramp       = 0.0
        self.prev_theta_r = 0.0
        self.settle_timer = 0.0

        # 11) Publishers
        self.cmd_pub     = self.create_publisher(Twist,  '/cmd_vel',    10)
        self.reset_pub   = self.create_publisher(Pose2D, '/reset_pose', 10)
        self.reached_pub = self.create_publisher(Bool,   '/reached',    10)

        # 12) Suscriptores
        self.create_subscription(
            Pose2D, '/pose', self.pose_callback,
            qos.qos_profile_sensor_data
        )
        # Suscriptor al goal del path_generator
        self.create_subscription(
            GoalPose, '/goal', self.goal_callback, 10
        )

        # 13) Timer 20 Hz
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        self.get_logger().info('ControllerNode PID started ✔')
        self.get_logger().info('Esperando goal del path_generator...')

    # ── Callback de pose ──────────────────────────────────────────────
    def pose_callback(self, msg: Pose2D):
        self.x_r           = msg.x
        self.y_r           = msg.y
        self.theta_r       = msg.theta
        self.pose_received = True

    # ── Callback de goal — recibe waypoints del path_generator ────────
    def goal_callback(self, msg: GoalPose):
        self.x_goal        = msg.x
        self.y_goal        = msg.y
        self.goal_label    = msg.label
        self.goal_is_final = msg.is_final
        self.goal_received = True
        self.done          = False
        self.state         = GIRANDO
        self.reset_pid()
        self.get_logger().info(
            f'Nuevo goal recibido: {msg.label} '
            f'({msg.x:.2f}, {msg.y:.2f}) '
            f'{"[ÚLTIMO]" if msg.is_final else ""}'
        )

    # ── Reset PID ─────────────────────────────────────────────────────
    def reset_pid(self):
        self.integral_v  = 0.0
        self.prev_ed     = 0.0
        self.integral_w  = 0.0
        self.prev_etheta = 0.0
        self.last_time   = self.get_clock().now()
        self.v_ramp      = 0.0

    # ── Waypoint alcanzado — notifica al path_generator ───────────────
    def waypoint_reached(self):
        # Corregir pose: x,y exactos del goal, theta real del robot
        pose_correcta       = Pose2D()
        pose_correcta.x     = self.x_goal
        pose_correcta.y     = self.y_goal
        pose_correcta.theta = self.theta_r
        self.reset_pub.publish(pose_correcta)
        self.get_logger().info(
            f'Pose corregida: ({self.x_goal:.2f}, {self.y_goal:.2f}) '
            f'θ_real={np.degrees(self.theta_r):.1f}°'
        )

        # Notificar al path_generator
        reached_msg      = Bool()
        reached_msg.data = True
        self.reached_pub.publish(reached_msg)
        self.get_logger().info(
            f'✔ {self.goal_label} alcanzado — notificando path_generator'
        )

        # Pausar hasta recibir siguiente goal
        self.goal_received = False
        self.done          = True

        if self.goal_is_final:
            self.get_logger().info('Trayectoria completa!')

    # ── PID lineal ────────────────────────────────────────────────────
    def compute_pid_v(self, e_d, dt, kp, ki, kd):
        P = kp * e_d
        if e_d > 0.05:
            self.integral_v += e_d * dt
            self.integral_v  = float(np.clip(self.integral_v, -1.0, 1.0))
        I = ki * self.integral_v
        D = kd * (e_d - self.prev_ed) / dt if dt > 0 else 0.0
        self.prev_ed = e_d
        return P + I + D

    # ── PID angular ───────────────────────────────────────────────────
    def compute_pid_w(self, e_theta, dt, kp, ki, kd):
        P = kp * e_theta
        if e_theta * self.prev_etheta < 0:
            self.integral_w = 0.0
            self.get_logger().info('Integral angular reseteado — cruzó objetivo')
        if abs(e_theta) > 0.02:
            self.integral_w += e_theta * dt
            self.integral_w  = float(np.clip(self.integral_w, -0.5, 0.5))
        I = ki * self.integral_w
        D = kd * (e_theta - self.prev_etheta) / dt if dt > 0 else 0.0
        self.prev_etheta = e_theta
        return P + I + D

    # ── Timer callback principal ───────────────────────────────────────
    def timer_cb(self):
        # Esperar primera pose
        if not self.pose_received:
            return

        # Esperar primer goal del path_generator
        if not self.goal_received:
            cmd = Twist()
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        # Pausa entre waypoints — robot quieto esperando siguiente goal
        if self.done:
            cmd = Twist()
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        current_time   = self.get_clock().now()
        dt             = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt == 0.0:
            return

        # ── Lectura de parámetros ────────────────────────────────────
        kp_v             = self.get_parameter('kp_v').value
        ki_v             = self.get_parameter('ki_v').value
        kd_v             = self.get_parameter('kd_v').value
        kp_w             = self.get_parameter('kp_w').value
        ki_w             = self.get_parameter('ki_w').value
        kd_w             = self.get_parameter('kd_w').value
        v_max            = self.get_parameter('v_max').value
        w_max            = self.get_parameter('w_max').value
        dist_tol         = self.get_parameter('dist_tolerance').value
        angle_tol        = self.get_parameter('angle_tolerance').value
        zona_frenado     = self.get_parameter('zona_frenado').value
        ramp_rate        = self.get_parameter('ramp_rate').value
        corr_angular     = self.get_parameter('corr_angular').value
        zona_frenado_ang = self.get_parameter('zona_frenado_ang').value
        w_min            = self.get_parameter('w_min').value
        omega_tol        = self.get_parameter('omega_tol').value
        settle_time      = self.get_parameter('settle_time').value
        settle_time_wp   = self.get_parameter('settle_time_wp').value

        # ── Errores ──────────────────────────────────────────────────
        e_x     = self.x_goal - self.x_r
        e_y     = self.y_goal - self.y_r
        e_d     = np.sqrt(e_x**2 + e_y**2)
        e_theta = wrap_to_pi(np.arctan2(e_y, e_x) - self.theta_r)

        # ── Estimación de ω ──────────────────────────────────────────
        d_theta           = wrap_to_pi(self.theta_r - self.prev_theta_r)
        omega_est         = d_theta / dt
        self.prev_theta_r = self.theta_r

        cmd = Twist()

        # ═══════════════════════════════════════════════════════════
        # ESTADO: GIRANDO
        # ═══════════════════════════════════════════════════════════
        if self.state == GIRANDO:
            if abs(e_theta) < angle_tol and abs(omega_est) < omega_tol:
                self.get_logger().info(
                    f'✔ Orientado | e_θ={np.degrees(e_theta):.1f}° → SETTLING'
                )
                self.state        = SETTLING
                self.settle_timer = settle_time
                self.reset_pid()
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
            else:
                W = self.compute_pid_w(e_theta, dt, kp_w, ki_w, kd_w)
                if abs(e_theta) < zona_frenado_ang:
                    w_limite = w_max * (abs(e_theta) / zona_frenado_ang)
                    w_limite = max(w_limite, w_min)
                else:
                    w_limite = w_max
                W = float(np.clip(W, -w_limite, w_limite))
                cmd.linear.x  = 0.0
                cmd.angular.z = W
                self.get_logger().info(
                    f'GIRANDO | e_θ={np.degrees(e_theta):.1f}° '
                    f'W={W:.3f} ω={omega_est:.3f}'
                )

        # ═══════════════════════════════════════════════════════════
        # ESTADO: SETTLING
        # ═══════════════════════════════════════════════════════════
        elif self.state == SETTLING:
            self.settle_timer -= dt
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            if self.settle_timer <= 0.0 and abs(omega_est) < omega_tol:
                self.get_logger().info('✔ Estabilizado → AVANZANDO')
                self.state = AVANZANDO
                self.reset_pid()

        # ═══════════════════════════════════════════════════════════
        # ESTADO: AVANZANDO
        # ═══════════════════════════════════════════════════════════
        elif self.state == AVANZANDO:
            if e_d < dist_tol:
                self.get_logger().info(
                    f'✔ {self.goal_label} alcanzado: '
                    f'x={self.x_r:.3f}  y={self.y_r:.3f}'
                )
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                self.state        = SETTLING_WP
                self.settle_timer = settle_time_wp
                return
            else:
                V = self.compute_pid_v(e_d, dt, kp_v, ki_v, kd_v)
                if e_d < zona_frenado:
                    v_limite = v_max * (e_d / zona_frenado)
                    v_limite = max(v_limite, 0.05)
                else:
                    v_limite = v_max
                self.v_ramp += ramp_rate * dt
                self.v_ramp  = float(np.clip(self.v_ramp, 0.0, v_limite))
                V = float(np.clip(V, 0.0, self.v_ramp))
                W_corr = kp_w * corr_angular * e_theta
                W_corr = float(np.clip(W_corr, -0.3, 0.3))
                cmd.linear.x  = V
                cmd.angular.z = W_corr
                self.get_logger().info(
                    f'AVANZANDO | e_d={e_d:.3f}m  '
                    f'V={V:.3f}  W_corr={W_corr:.3f}'
                )

        # ═══════════════════════════════════════════════════════════
        # ESTADO: SETTLING_WP
        # ═══════════════════════════════════════════════════════════
        elif self.state == SETTLING_WP:
            self.settle_timer -= dt
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            if self.settle_timer <= 0.0 and abs(omega_est) < omega_tol:
                self.get_logger().info('✔ Detenido en WP → notificando')
                self.waypoint_reached()
            return

        # ═══════════════════════════════════════════════════════════
        # ESTADO: LISTO
        # ═══════════════════════════════════════════════════════════
        elif self.state == LISTO:
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
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