# GoalPose.msg
# Mensaje custom para el topico /goal
# Basado en geometry_msgs/Pose pero adaptado para robot diferencial 2D

float64 x        # posicion objetivo en x (metros)
float64 y        # posicion objetivo en y (metros)
float64 theta   # orientacion objetivo (radianes) — opcional, path generator decide
string  label    # nombre del waypoint ej: "WP1", "esquina_1"
bool    is_final # True si es el ultimo waypoint de la trayectoria