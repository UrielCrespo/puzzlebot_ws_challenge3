#!/usr/bin/env python3

import rclpy
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import csv
import time

class DataExporter(Node):

    def __init__(self):
        super().__init__('data_exporter')

        # ── Variables de los tópicos ───────────────────────────────────
        self.wr       = 0.0
        self.wl       = 0.0
        self.cmd_v    = 0.0   # velocidad lineal del comando
        self.cmd_w    = 0.0   # velocidad angular del comando

        # ── Tiempo de inicio ──────────────────────────────────────────
        self.start_time = time.time()

        # ── Pedir nombre del experimento ──────────────────────────────
        self.declare_parameter('nombre', 'experimento')
        nombre = self.get_parameter('nombre').value
        filename = f'/home/yuye/{nombre}.csv'

        # ── Abrir archivo CSV ─────────────────────────────────────────
        self.file   = open(filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'time',
            'cmd_linear',
            'cmd_angular',
            'VelocityEncR',
            'VelocityEncL'
        ])

        # ── Suscriptores ──────────────────────────────────────────────
        self.create_subscription(
            Float32,
            '/VelocityEncR',
            self.wr_callback,
            qos.qos_profile_sensor_data
        )
        self.create_subscription(
            Float32,
            '/VelocityEncL',
            self.wl_callback,
            qos.qos_profile_sensor_data
        )
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # ── Timer a 50 Hz ─────────────────────────────────────────────
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        self.get_logger().info(f'Exportando datos a {filename}')

    def wr_callback(self, msg: Float32):
        self.wr = float(msg.data)

    def wl_callback(self, msg: Float32):
        self.wl = float(msg.data)

    def cmd_callback(self, msg: Twist):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z

    def timer_cb(self):
        t = time.time() - self.start_time
        self.writer.writerow([
            f'{t:.4f}',
            f'{self.cmd_v:.4f}',
            f'{self.cmd_w:.4f}',
            f'{self.wr:.4f}',
            f'{self.wl:.4f}'
        ])

    def destroy_node(self):
        self.file.close()
        self.get_logger().info('Archivo CSV guardado correctamente!')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataExporter()
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
