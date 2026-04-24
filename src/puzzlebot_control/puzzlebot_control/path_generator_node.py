#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Bool
from puzzlebot_msgs.msg import GoalPose
from geometry_msgs.msg import Pose2D
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class PathGeneratorNode(Node):

    def __init__(self):
        super().__init__('path_generator_node')

        # Posición actual del robot
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Parámetro para elegir parte 1 o parte 2
        self.declare_parameter('parte', 1)
        parte = self.get_parameter('parte').value

        # Parámetros del robot para verificar alcanzabilidad
        self.declare_parameter('v_max', 0.3)
        self.declare_parameter('w_max', 1.0)

        # ── Umbrales de auto-tuning ───────────────────────────────────
        # Si distancia > umbral_lejos → modo rápido
        # Si distancia < umbral_cerca → modo preciso
        self.declare_parameter('umbral_lejos', 1.5)   # metros
        self.declare_parameter('v_max_rapido', 0.30)  # m/s modo lejos
        self.declare_parameter('v_max_preciso', 0.15) # m/s modo cerca

        # Cargar waypoints desde YAML
        pkg_dir   = get_package_share_directory('puzzlebot_control')
        yaml_path = os.path.join(pkg_dir, 'config', 'waypoints.yaml')

        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        key = f'part{parte}'
        self.waypoints   = config[key]['waypoints']
        self.current_idx = 0
        self.goal_active = False

        self.get_logger().info(
            f'PathGenerator iniciado — Parte {parte} '
            f'con {len(self.waypoints)} waypoints'
        )

        # Publisher a /goal
        self.goal_pub = self.create_publisher(GoalPose, '/goal', 10)

        # Suscriptor a /reached
        self.create_subscription(Bool, '/reached', self.reached_cb, 10)

        # Suscriptor a /pose para conocer posición actual del robot
        self.create_subscription(
            Pose2D, '/pose', self.pose_callback,
            qos.qos_profile_sensor_data
        )

        # Cliente para cambiar parámetros del controller_node (auto-tuning)
        self.param_client = self.create_client(
            SetParameters,
            '/controller_node/set_parameters'
        )

        # Timer para publicar el primer goal después de 2 segundos
        # (da tiempo a que el controller_node arranque)
        self.timer = self.create_timer(2.0, self.publish_current_goal)

    # ── Callback de pose ──────────────────────────────────────────────
    def pose_callback(self, msg: Pose2D):
        self.robot_x = msg.x
        self.robot_y = msg.y

    # ── Verificar si el punto es alcanzable ───────────────────────────
    def is_reachable(self, x, y):
        v_max = self.get_parameter('v_max').value
        w_max = self.get_parameter('w_max').value

        # Distancia desde posición actual del robot
        dx       = x - self.robot_x
        dy       = y - self.robot_y
        distance = (dx**2 + dy**2) ** 0.5

        # Punto demasiado cerca — no tiene sentido moverse
        if distance < 0.05:
            return False, "El punto esta demasiado cerca de la posicion actual"

        # Radio mínimo de curvatura del robot
        r_min = v_max / w_max
        self.get_logger().info(
            f'Radio minimo de curvatura: {r_min:.3f}m  '
            f'Distancia al goal: {distance:.3f}m'
        )

        return True, "OK"

    # ── Auto-tuning: ajusta v_max según distancia al goal ─────────────
    def adjust_gains(self, distance):
        umbral_lejos  = self.get_parameter('umbral_lejos').value
        v_max_rapido  = self.get_parameter('v_max_rapido').value
        v_max_preciso = self.get_parameter('v_max_preciso').value

        if distance > umbral_lejos:
            v_max_nuevo = v_max_rapido
            modo        = 'RAPIDO'
        else:
            v_max_nuevo = v_max_preciso
            modo        = 'PRECISO'

        self.get_logger().info(
            f'Auto-tuning: distancia={distance:.2f}m → '
            f'v_max={v_max_nuevo} (modo {modo})'
        )

        # Esperar a que el servicio esté disponible
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                'Controller no disponible para auto-tuning — usando v_max actual'
            )
            return

        # Crear request con el nuevo v_max
        request       = SetParameters.Request()
        param         = Parameter()
        param.name    = 'v_max'
        param.value   = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=float(v_max_nuevo)
        )
        request.parameters = [param]

        # Llamar el servicio de forma asíncrona
        future = self.param_client.call_async(request)
        future.add_done_callback(self.adjust_gains_cb)

    # ── Callback de confirmación del auto-tuning ──────────────────────
    def adjust_gains_cb(self, future):
        try:
            result = future.result()
            if result.results[0].successful:
                self.get_logger().info('Auto-tuning aplicado correctamente ✔')
            else:
                self.get_logger().warn(
                    f'Auto-tuning falló: {result.results[0].reason}'
                )
        except Exception as e:
            self.get_logger().error(f'Error en auto-tuning: {e}')

    # ── Publicar goal actual ──────────────────────────────────────────
    def publish_current_goal(self):
        # Cancelar timer — solo publicar bajo demanda
        self.timer.cancel()

        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('Todos los waypoints completados!')
            return

        wp = self.waypoints[self.current_idx]
        x  = float(wp['x'])
        y  = float(wp['y'])

        # Verificar alcanzabilidad
        reachable, reason = self.is_reachable(x, y)
        if not reachable:
            self.get_logger().error(
                f'WP {wp["label"]} ({x}, {y}) NO ES ALCANZABLE: {reason}'
            )
            # Saltar al siguiente waypoint
            self.current_idx += 1
            self.publish_current_goal()
            return

        # ── Auto-tuning ANTES de publicar ────────────────────────────
        dx       = x - self.robot_x
        dy       = y - self.robot_y
        distance = (dx**2 + dy**2) ** 0.5
        self.adjust_gains(distance)

        # Publicar goal
        msg          = GoalPose()
        msg.x        = x
        msg.y        = y
        msg.theta    = 0.0
        msg.label    = wp['label']
        msg.is_final = (self.current_idx == len(self.waypoints) - 1)

        self.goal_pub.publish(msg)
        self.goal_active = True

        self.get_logger().info(
            f'Goal publicado → {msg.label} '
            f'({msg.x:.2f}, {msg.y:.2f}) '
            f'{"[ULTIMO]" if msg.is_final else ""}'
        )

    # ── Callback de waypoint alcanzado ────────────────────────────────
    def reached_cb(self, msg: Bool):
        if not msg.data or not self.goal_active:
            return

        self.goal_active = False
        wp = self.waypoints[self.current_idx]
        self.get_logger().info(
            f'Waypoint {wp["label"]} alcanzado!'
        )

        self.current_idx += 1

        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('Trayectoria completa!')
            return

        # Publicar siguiente goal
        self.publish_current_goal()


def main(args=None):
    rclpy.init(args=args)
    node = PathGeneratorNode()
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