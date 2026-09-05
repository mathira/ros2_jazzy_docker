# Diseño: exploración autónoma TurtleBot3 con mapeo odométrico y DQN

## Objetivo

Crear el paquete Python ROS 2 Jazzy `turtleboot3_autonomous_nav` para que un TurtleBot3 explore autónomamente el escenario `turtlebot3_dqn_stage4`. La misión maximiza el área monitorizada sin teleoperación, sin mapa previo, sin `map_server`, AMCL ni SLAM Toolbox.

El robot forma un mapa de ocupación incremental en el marco `odom` usando su odometría y los escaneos LiDAR. Una política DQN entrenada decide la dirección de exploración; un controlador determinista independiente aplica la decisión con límites de seguridad y es la única entidad que publica `/cmd_vel`.

## Alcance

Se entregarán:

- El paquete Python `turtleboot3_autonomous_nav`.
- Un launcher de misión que incluye el launcher oficial `turtlebot3_dqn_stage4.launch.py`, carga una política entrenada y abre RViz opcionalmente.
- Un launcher de entrenamiento reproducible, sin interfaz gráfica por defecto, que ejecuta episodios, reinicia el mundo y guarda el mejor modelo DQN.
- Mapeo de ocupación propio publicado como `/coverage_map` y métricas de cobertura.
- Política DQN, memoria de repetición, evaluación periódica y persistencia del modelo.
- Control reactivo protegido por LiDAR, recuperación de estancamiento y parada segura.
- Configuraciones, pruebas y README de instalación, entrenamiento, misión y verificación.

No se incluirán Nav2, SLAM Toolbox, AMCL, mapas de ocupación preparados previamente ni comandos manuales durante la misión.

## Dependencias

El workspace debe contener las fuentes compatibles con Jazzy de `turtlebot3`, `turtlebot3_msgs` y `turtlebot3_simulations`. El escenario se arranca mediante el launcher oficial del paquete `turtlebot3_gazebo`.

El paquete declara las dependencias ROS que consume: `rclpy`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `std_msgs`, `tf2_ros`, `ros_gz_interfaces` y `ament_index_python`. La dependencia Python de entrenamiento es PyTorch; se instala de forma explícita en la imagen o entorno de desarrollo y se documenta en el README. No se descarga ninguna dependencia durante el build del paquete.

## Arquitectura

```text
Gazebo / TurtleBot3
  | /scan, /odom, TF                 | /cmd_vel
  v                                  ^
coverage_mapper -> /coverage_map ----+----- safe_motion_controller
  |                                        ^
  +-> observation_builder -> /dqn_observation |
                                          |
                        dqn_explorer -> /exploration_action
```

`coverage_mapper` transforma los rayos de `/scan` con la pose actual de `/odom`. El trazado de rayos marca celdas libres hasta el impacto y ocupadas en el impacto; emplea evidencia acumulada para filtrar mediciones aisladas. El mapa se publica como `nav_msgs/OccupancyGrid` en `odom`. Esto es mapeo odométrico: no corrige la deriva ni realiza cierre de ciclos, por lo que no se presenta como SLAM.

`observation_builder` reduce el mapa a una ventana local y calcula la ganancia de área desconocida en ocho direcciones. Junto con 12 sectores normalizados de LiDAR y el estado de avance/giro, crea una observación numérica de tamaño fijo.

`dqn_explorer` recibe esa observación y publica una acción discreta: avance recto, avance con giro suave a izquierda o derecha, giro marcado a izquierda o derecha, o giro de recuperación. No publica velocidades.

`safe_motion_controller` traduce la acción a una velocidad acotada. Si el LiDAR detecta un obstáculo dentro de la distancia de frenado, anula el avance, selecciona un giro hacia el lado más libre y cuenta una intervención de seguridad. Si no hay avance ni cobertura nueva durante el intervalo configurado, ejecuta una recuperación y reporta estancamiento. Al apagarse o perderse datos de sensor, publica velocidad cero.

## Entrenamiento y misión

`training.launch.py` inicia `turtlebot3_dqn_stage4.launch.py` en modo sin GUI, `coverage_mapper`, `observation_builder`, `safe_motion_controller` y `dqn_trainer`. Cada episodio comienza con el mundo restablecido a su estado inicial y el mapa interno vacío. El entrenador controla el tiempo simulado, termina episodios por límite de pasos, estancamiento o cobertura objetivo y solicita el reinicio del mundo mediante el servicio de control de Gazebo expuesto por `ros_gz_interfaces`.

El DQN usa una red neuronal pequeña totalmente conectada, memoria de repetición, red objetivo y exploración epsilon-greedy decreciente. Cada evaluación usa epsilon cero. Se persiste el modelo con mayor cobertura media de evaluación junto con un archivo JSON de configuración y métricas, para que la misión pueda reproducir exactamente la política entrenada.

`mission.launch.py` inicia el mismo escenario, los nodos de percepción y control, y `dqn_explorer` configurado con el archivo de modelo. No crea experiencias ni actualiza pesos. RViz muestra `/scan`, `/coverage_map`, la trayectoria y métricas de cobertura, pero no interviene en la conducción.

## Recompensa y criterios de episodio

La recompensa de cada paso se compone de:

- Recompensa positiva proporcional al número de celdas antes desconocidas que se observaron en ese paso.
- Bonificación moderada cuando el robot alcanza una región con frontera de exploración.
- Penalización por paso para favorecer trayectorias cortas.
- Penalización por no descubrir área durante una ventana temporal.
- Penalización fuerte por intervención de seguridad, recuperación o colisión detectada.

Los cambios repetidos de una celda ya conocida no generan recompensa positiva. Esta regla evita que el agente obtenga recompensa girando frente al mismo obstáculo. La cobertura es el porcentaje de celdas que han pasado de desconocidas a libres u ocupadas dentro de los límites configurados para el escenario.

## Estructura prevista

```text
ros2_ws/src/turtleboot3_autonomous_nav/
├── config/
│   ├── exploration.yaml
│   └── training.yaml
├── launch/
│   ├── mission.launch.py
│   └── training.launch.py
├── models/
│   └── README.md
├── resource/turtleboot3_autonomous_nav
├── rviz/exploration.rviz
├── test/
├── turtleboot3_autonomous_nav/
│   ├── coverage_mapper.py
│   ├── observation_builder.py
│   ├── safe_motion_controller.py
│   ├── dqn_explorer.py
│   ├── dqn_trainer.py
│   ├── replay_buffer.py
│   └── metrics.py
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## Gestión de fallos

- Si `/scan` o `/odom` queda obsoleto, el controlador publica velocidad cero y expone el fallo en la métrica de misión.
- Una acción cargada que no sea válida o un modelo incompatible impide activar `dqn_explorer`; nunca se sustituyen por comandos aleatorios en misión.
- El reinicio fallido del mundo interrumpe el entrenamiento con un error claro en vez de mezclar episodios.
- Las lecturas LiDAR no finitas se descartan; las lecturas fuera del alcance se usan solo para marcar espacio libre hasta el alcance permitido.
- La misión termina con velocidad cero al alcanzar cobertura objetivo, el límite de pasos configurado o una orden de apagado.

## Verificación

1. Pruebas unitarias comprueban el trazado de rayos, actualización de ocupación, cálculo de cobertura, observación de tamaño fijo, selección de acción y limitaciones del controlador.
2. Pruebas de launch verifican que misión y entrenamiento incluyen el launcher `turtlebot3_dqn_stage4`, usan tiempo simulado y no incluyen SLAM, AMCL, Nav2 ni `map_server`.
3. Una prueba de integración inicia la misión con un modelo de prueba y confirma que se publican `/coverage_map`, `/exploration_action` y `/cmd_vel` sin intervención manual.
4. Un entrenamiento corto verifica reinicio de episodios, creación de métricas y guardado de un checkpoint.
5. La validación manual en RViz confirma que el mapa crece durante el recorrido, el robot cambia de dirección frente a obstáculos y la cobertura aumenta sin teleoperación.

## Criterios de aceptación

Con las dependencias de TurtleBot3 disponibles y el workspace construido, los dos launchers se ejecutan con un único comando. El entrenamiento produce un modelo y métricas reproducibles; la misión carga ese modelo, explora `dqn_stage4` usando `/scan` y `/odom`, publica su propio mapa de cobertura y detiene o evita el robot ante riesgo de colisión. No se usa un mapa previo, SLAM, AMCL ni conducción manual.
