# ROS 2 Jazzy + Stage + TurtleBot3

Workspace ROS 2 Jazzy con simulación Stage, Gazebo Harmonic, Nav2 y exploración autónoma de TurtleBot3.

## Ejecutar con Docker

Docker sigue siendo el entorno de ejecución, pero el proyecto se administra directamente desde esta carpeta y no depende de Dev Containers.

```bash
./setup/setup.sh       # primera vez: imagen, dependencias y build
./setup/up.sh           # usos posteriores
docker compose -f setup/docker-compose.yml exec ros2 bash
```

Dentro del contenedor:

```bash
setup/build.sh
source /ros2_ws/install/setup.bash
```

La interfaz gráfica está disponible en <http://localhost:6080/vnc.html> con contraseña `ros`.

Para detener el entorno:

```bash
docker compose -f setup/docker-compose.yml down
```

## Estructura

```text
setup/       Dockerfile, Compose y scripts de instalación/ejecución
ros2_ws/     paquetes ROS 2, manifiestos de repositorios y modelos
docs/        especificaciones, planes y verificaciones
```

La imagen instala ROS 2 Jazzy, Gazebo Harmonic, `colcon`, `rosdep`, `vcstool`, Nav2, RViz, Stage, TurtleBot3, OpenCV, NumPy, PyYAML y PyTorch CPU. `setup/setup.sh` importa los repositorios definidos en `ros2_ws/stage_nav.repos` y en el manifiesto de TurtleBot3, instala dependencias con `rosdep` y compila el workspace.

Los artefactos `ros2_ws/build`, `ros2_ws/install` y `ros2_ws/log` se guardan en volúmenes Docker del proyecto, fuera del árbol fuente.

Los detalles de navegación Stage, PID, turtlesim y exploración autónoma están en los README de sus paquetes.
