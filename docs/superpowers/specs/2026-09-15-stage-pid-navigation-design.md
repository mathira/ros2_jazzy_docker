# Diseño: navegación PID para Stage ROS 2

## Propósito

Crear un paquete ROS 2 nuevo e independiente, `stage_pid_navigation`, que lleve de forma autónoma un robot diferencial simulado en Stage a una meta plana `(x, y)`. El operador aporta la meta mediante argumentos del launcher. La solución debe ser repetible, detenerse al llegar y evitar colisiones mediante el LiDAR disponible en Stage.

## Alcance y restricciones

- El paquete se creará bajo `ros2_ws/src/stage_pid_navigation`; no se modificará ningún paquete existente.
- Debe ser compatible con ROS 2 Jazzy y estar implementado en Python con `rclpy`.
- El controlador utiliza `geometry_msgs/msg/Twist`; no habilita los modos opcionales TwistStamped ni Ackermann de `stage_ros2`.
- La navegación trabaja en coordenadas métricas del frame publicado por `Odometry`.
- No incluye planificación global ni mapas: combina guiado PID hacia la meta con una reacción local y conservadora al LiDAR.

## Interfaces confirmadas de `stage_ros2`

La implementación local de `stage_ros2` publica y consume los siguientes nombres para un único robot sin prefijos (`enforce_prefixes:=false`, configuración predeterminada):

| Rol | Tópico | Tipo |
| --- | --- | --- |
| Pose estimada | `/odom` | `nav_msgs/msg/Odometry` |
| LiDAR | `/base_scan` | `sensor_msgs/msg/LaserScan` |
| Comandos | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| Referencia global opcional | `/ground_truth` | `nav_msgs/msg/Odometry` |

En mundos con varios vehículos, o con `enforce_prefixes:=true`, el bridge antepone el nombre del modelo. Para el modelo `robot_0`, las interfaces serán `/robot_0/odom`, `/robot_0/base_scan` y `/robot_0/cmd_vel`. Por ello, `odom_topic`, `scan_topic` y `cmd_vel_topic` serán parámetros del nodo y argumentos del launcher.

## Arquitectura

El paquete contendrá un solo ejecutable, `pid_navigator`, y separará la matemática ROS-independiente de la integración con ROS:

- `control.py`: funciones puras y clases pequeñas para normalización angular, PID con límite anti-windup, extracción segura de distancias del láser y generación de un comando de control.
- `pid_navigator.py`: nodo `rclpy` que se suscribe a odometría y LiDAR, mantiene el estado, invoca el controlador a frecuencia fija y publica `Twist`.
- `pid_navigation.launch.py`: declara la meta y todos los parámetros operativos; inicia el nodo sin lanzar Stage, permitiendo usar cualquier escenario que el usuario haya iniciado.

El nodo no comenzará a avanzar hasta haber recibido odometría. Si no hay LiDAR, por diseño permanecerá detenido salvo que se use explícitamente `require_scan:=false`; así el comportamiento predeterminado prioriza no colisionar.

## Flujo de control

1. El callback de odometría obtiene `(x, y, yaw)` del quaternion.
2. El callback de LiDAR conserva las lecturas finitas de un sector frontal configurable y calcula su distancia mínima.
3. El temporizador calcula distancia y ángulo hacia la meta. Un PID angular genera `angular.z`, limitado por `max_angular_speed`.
4. La velocidad lineal depende de la distancia, queda limitada por `max_linear_speed` y se reduce a cero mientras el error angular supere `heading_stop_threshold`.
5. Si el obstáculo frontal está dentro de `slowdown_distance`, se reduce proporcionalmente la velocidad. Dentro de `stop_distance`, se bloquea el avance y se ordena un giro de escape determinista hacia el lado con mayor despeje.
6. Dentro de `goal_tolerance`, se publica `Twist()` nulo, se informa que la meta fue alcanzada y el nodo se mantiene detenido.
7. En apagado del nodo, se publica un comando nulo final.

El PID aplica límite a la integral y usa un `dt` válido derivado de reloj monotónico. El término derivativo se ignora en el primer ciclo y cuando el intervalo no es positivo.

## Parámetros del launcher

Obligatorios desde CLI:

- `goal_x` y `goal_y` (metros).

Configurables con valores seguros por defecto:

- tópicos: `odom_topic:=/odom`, `scan_topic:=/base_scan`, `cmd_vel_topic:=/cmd_vel`;
- tasa: `control_rate:=10.0` Hz;
- PID angular: `kp:=1.8`, `ki:=0.0`, `kd:=0.15`, `integral_limit:=1.0`;
- movimiento: `max_linear_speed:=0.35`, `max_angular_speed:=1.2`, `heading_stop_threshold:=0.7`, `goal_tolerance:=0.15`;
- seguridad: `slowdown_distance:=0.9`, `stop_distance:=0.35`, `front_sector_angle:=0.7`, `require_scan:=true`.

Ejemplo de uso con el mundo `cave` de Stage:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch stage_ros2 stage.launch.py world:=cave
ros2 launch stage_pid_navigation pid_navigation.launch.py goal_x:=2.0 goal_y:=1.5
```

Para `robot_0` en un mundo con prefijos:

```bash
ros2 launch stage_pid_navigation pid_navigation.launch.py \
  goal_x:=2.0 goal_y:=1.5 \
  odom_topic:=/robot_0/odom \
  scan_topic:=/robot_0/base_scan \
  cmd_vel_topic:=/robot_0/cmd_vel
```

## Pruebas y verificación

Las pruebas con `pytest` cubrirán, antes de la implementación correspondiente:

- normalización de errores angulares alrededor de `-pi/pi`;
- salida PID limitada y protección de integral;
- detención al entrar en la tolerancia de meta;
- impedir avance ante gran error de orientación;
- reducción/detención por obstáculo frontal y selección de giro de escape;
- ignorar valores no finitos de `LaserScan`.

La verificación de integración consistirá en compilar únicamente el paquete nuevo con `colcon build --packages-select stage_pid_navigation`, ejecutar su suite y lanzar Stage junto con el launcher hacia una meta libre del mundo `cave`.
