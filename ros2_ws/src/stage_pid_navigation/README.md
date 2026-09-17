# Proyecto de Robots 2

**Estudiante:** Mathias Rodriguez

**Institución:** UTEC - ITR Norte Rivera

**Programa:** PRIA

## Descripción

En este trabajo hice un paquete de ROS 2 para controlar un robot en el
simulador Stage.

El robot recibe su posición y la información del sensor láser. Con esos datos
se mueve solo hacia el objetivo, evita obstáculos y publica sus velocidades.

## Cómo ejecutarlo

Primero entro al contenedor y preparo el workspace:

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select stage_pid_navigation
source /ros2_ws/install/setup.bash
```

Para iniciar el mapa y la navegación con un solo comando uso:

```bash
ros2 launch stage_pid_navigation full.launch.py
```

El robot se dirige al bloque verde del mundo `cave` y se detiene cerca de él
para evitar una colisión.

También puedo hacerlo en dos terminales. Primero redibujo solamente el mapa:

```bash
ros2 launch stage_pid_navigation stage_world.launch.py
```

Después, en otra terminal, arranco solamente la navegación:

```bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch stage_pid_navigation pid_navigation.launch.py
```

## Tópicos principales

- `/ground_truth`: posición del robot.
- `/base_scan`: información del sensor láser.
- `/cmd_vel`: velocidad enviada al robot.

## Qué comprobé

- El robot se mueve sin control manual.
- Usa la odometría y el sensor láser.
- Evita los obstáculos del mapa.
- Llega al objetivo y se detiene.
- El código está organizado en un paquete ROS 2.
