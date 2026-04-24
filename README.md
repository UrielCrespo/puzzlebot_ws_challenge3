# 🤖 Puzzlebot PID Controller — Mini Challenge week 8

Control de lazo cerrado PI para navegación autónoma del robot diferencial Puzzlebot en ROS2 Humble. El robot sigue una trayectoria cuadrada de 2x2 metros de forma autónoma usando odometría de encoders y un controlador PI sintonizado con identificación de sistema en MATLAB.

---

## 📋 Requisitos

* Ubuntu 22.04
* ROS2 Humble
* Python 3.10
* Paquetes ROS2: `rclpy`, `geometry_msgs`, `std_msgs`

---

## ⚡ Instalación rápida

```bash
# 1. Clonar el repositorio
git clone https://github.com/UrielCrespo/puzzlebot_ws_challenge3.git
cd puzzlebot_ws_challenge3

# 2. Compilar
colcon build

# 3. Source
source install/setup.bash

# 4. (Opcional) Agregar al bashrc
echo "source ~/puzzlebot_ws_challenge3/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🚀 Uso

### Parte 1 — Cuadrado 2x2 metros

```bash
ros2 launch puzzlebot_control part1.launch.py
```

### Parte 2 — Trayectoria arbitraria (mínimo 3 puntos)

```bash
ros2 launch puzzlebot_control part2.launch.py
```

---

## 🗂️ Estructura del proyecto

```
puzzlebot_ws_challenge3/
└── src/
    ├── puzzlebot_control/
    │   ├── config/
    │   │   ├── waypoints.yaml
    │   │   └── pid_params.yaml
    │   ├── launch/
    │   │   ├── part1.launch.py
    │   │   └── part2.launch.py
    │   └── puzzlebot_control/
    │       ├── odometry_node.py
    │       ├── controller_node.py
    │       ├── path_generator_node.py
    │       └── data_exporter.py
    └── puzzlebot_msgs/
        └── msg/
            └── GoalPose.msg
```

---

## 🏗️ Arquitectura del sistema

```
odometry_node ──→ /pose ──→ controller_node ──→ /cmd_vel ──→ Robot
                                 ↑                    ↓
                           path_generator    /reset_pose
                                 ↑
                            /reached
```

### Tópicos principales

| Tópico        | Tipo     | Descripción             |
| ------------- | -------- | ----------------------- |
| `/pose`       | Pose2D   | Pose estimada del robot |
| `/goal`       | GoalPose | Waypoint actual         |
| `/reached`    | Bool     | Waypoint alcanzado      |
| `/cmd_vel`    | Twist    | Velocidades             |
| `/reset_pose` | Pose2D   | Corrección de drift     |

---

## ⚙️ Parámetros del robot

| Parámetro | Valor      | Descripción                   |
| --------- | ---------- | ----------------------------- |
| `r`       | 0.0556 m   | Radio de ruedas               |
| `l`       | 0.1746 m   | Distancia entre ruedas        |
| `kp_v`    | 0.5013     | Ganancia proporcional lineal  |
| `ki_v`    | 1.3311     | Ganancia integral lineal      |
| `kp_w`    | 0.4775     | Ganancia proporcional angular |
| `ki_w`    | 1.2013     | Ganancia integral angular     |
| `v_max`   | 0.25 m/s   | Velocidad máxima              |
| `w_max`   | 0.15 rad/s | Velocidad angular máxima      |

---

## 📍 Modificar waypoints

```yaml
part1:
  waypoints:
    - label: "esquina_1"
      x: 2.0
      y: 0.0
    - label: "esquina_2"
      x: 2.0
      y: 2.0

part2:
  waypoints:
    - label: "WP1"
      x: 1.0
      y: 0.0
    - label: "WP2"
      x: 1.5
      y: 1.0
    - label: "WP3"
      x: 0.5
      y: 1.5
```

Recompilar:

```bash
colcon build && source install/setup.bash
```

---

## 🔧 Ajustar ganancias por batería

```yaml
battery_100:
  kp_v: 0.5013
  ki_v: 1.3311
  kp_w: 0.4775
  ki_w: 1.2013

battery_70:
  ...

battery_40:
  ...
```

---

## 🛡️ Estrategias de robustez

* Filtro EMA (α=0.3)
* Zona muerta de 0.05 rad/s
* Integración a 20 Hz
* Reset de pose en waypoints
* Anti-windup angular
* Estado SETTLING
* Zona de frenado (0.4 m)
* Rampa de aceleración (0.4 m/s²)
* Corrección angular en línea recta
* Validación de waypoints

---

## 📊 Identificación de sistema

```bash
# Lineal
ros2 run puzzlebot_control data_exporter --ros-args -p nombre:=experimento_lineal
sleep 3 && ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --rate 50 --times 250

# Angular
ros2 run puzzlebot_control data_exporter --ros-args -p nombre:=experimento_angular
sleep 3 && ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" --rate 50 --times 250
```

Luego:

1. Ejecutar `identificacion_pid.m` en MATLAB
2. Copiar ganancias a `controller_node.py`

---

## 🌐 Variables de entorno

```bash
export ROS_DOMAIN_ID=0
```

Persistente:

```bash
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```
