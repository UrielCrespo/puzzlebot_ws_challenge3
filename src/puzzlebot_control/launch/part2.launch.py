from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Nodo 1 — Odometría
        Node(
            package='puzzlebot_control',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[{
                'alpha_filter':     0.3,
                'encoder_deadzone': 0.05,
            }]
        ),

        # Nodo 2 — Controlador PID
        Node(
            package='puzzlebot_control',
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=[{
                'kp_v': 0.5688,
                'ki_v': 1.7818,
                'kd_v': 0.0000,
                'kp_w': 0.5917,
                'ki_w': 1.7246,
                'kd_w': 0.0000,
                'v_max': 0.3,
                'w_max': 1.0,
                'dist_tolerance':    0.1,
                'angle_tolerance':   0.08,
                'zona_frenado':      0.4,
                'ramp_rate':         0.4,
                'corr_angular':      0.3,
                'zona_frenado_ang':  0.35,
                'w_min':             0.12,
                'omega_tol':         0.08,
                'settle_time':       0.20,
                'settle_time_wp':    0.25,
            }]
        ),

        # Nodo 3 — Path Generator Parte 2 (trayectoria arbitraria)
        Node(
            package='puzzlebot_control',
            executable='path_generator_node',
            name='path_generator_node',
            output='screen',
            parameters=[{
                'parte': 2,
                'v_max': 0.3,
                'w_max': 1.0,
            }]
        ),

    ])