# TurtleBot3 autonomous DQN exploration

ROS 2 Jazzy / Gazebo Harmonic exploration in ROBOTIS's official stage4 world.
The package name intentionally uses `turtleboot3_autonomous_nav` (two o's).
It builds an occupancy grid from LiDAR and odometry and selects six discrete
motion actions with a DQN. It does not launch Nav2, SLAM, AMCL, a map server,
teleoperation, or externally supplied goals.

The mission starts `coverage_mapper`, `observation_builder`, `dqn_explorer`, and
`safe_motion_controller`. Only the controller publishes `/cmd_vel`; its sensor
watchdog, clearance checks, and recovery logic can override policy actions.
Training replaces the explorer with `dqn_trainer`, including world/map resets,
replay updates, target-network synchronization, and epsilon-zero evaluations.

## Relación con la consigna PRIA

La misión parte de la pose inicial del TurtleBot3 Burger en el escenario
`turtlebot3_dqn_stage4`. Una vez iniciado `mission.launch.py`, el robot
monitorea la mayor área observable posible sin comandos de conducción manual.
La arquitectura separa explícitamente adquisición de información, decisión y
actuación:

```text
/scan + /odom
      │
      ├─ coverage_mapper ──> /coverage_map + /coverage_metrics
      ├─ observation_builder ──> /dqn_observation
      ├─ dqn_explorer ──> /exploration_action
      └─ safe_motion_controller ──> /cmd_vel
```

| Criterio de la consigna | Implementación y evidencia |
| --- | --- |
| Entorno compatible con ROS 2 | ROS 2 Jazzy, Gazebo Harmonic y el launcher oficial `turtlebot3_dqn_stage4.launch.py`. |
| Nodos desarrollados por el estudiante | `coverage_mapper`, `observation_builder`, `dqn_explorer`, `safe_motion_controller` y `dqn_trainer`. |
| Percepción efectiva | `/scan` identifica espacio libre/ocupado; `/odom` sitúa los rayos en una grilla. Ambos forman `/coverage_map` y la observación para DQN. |
| Decisión autónoma | La política DQN selecciona la acción exploratoria a partir de LiDAR, parche local del mapa y ganancia de área desconocida. No es una secuencia temporizada. |
| Actuación ROS 2 | Solo `safe_motion_controller` publica `geometry_msgs/Twist` en `/cmd_vel`; limita velocidad, frena ante riesgo y recupera bloqueos. |
| Sin teleoperación | `mission.launch.py` carga un checkpoint obligatorio y no inicia nodos de teleoperación ni espera metas externas. |
| Área monitoreada | `/coverage_metrics` publica el porcentaje de celdas conocidas y `best.metrics.json` guarda la cobertura media de evaluación del modelo. |
| Seguridad y estabilidad | Watchdogs de LiDAR/odometría/acción, parada por datos inválidos, recuperación acotada, límites de pasos y de cobertura, y parada final con velocidad cero. |

Una celda se considera monitoreada cuando pasa de desconocida a libre u
ocupada por una medición LiDAR proyectada con odometría. El porcentaje es la
fracción de celdas conocidas dentro de la grilla configurada de 20 m × 20 m;
incluye celdas ocupadas, no equivale a área libre alcanzable y se informa para
comparar ejecuciones bajo la misma configuración.

### Protocolo de demostración

1. Entrenar o seleccionar un checkpoint generado por el launcher de
   entrenamiento.
2. Lanzar únicamente `mission.launch.py` con ese checkpoint; no iniciar
   `teleop`, ni publicar manualmente en `/cmd_vel`.
3. Mostrar en RViz `/coverage_map`, `/scan` y `/exploration_status` mientras
   el robot navega.
4. Verificar `ros2 topic info /cmd_vel --verbose`: el único publicador debe
   ser `/safe_motion_controller`.
5. Al terminar por cobertura objetivo o `max_steps`, registrar
   `/coverage_metrics`, la grilla final y el archivo `best.metrics.json`.

El video de referencia define la especificación visual de la misión. Antes de
la demostración final se debe contrastar con él la pose inicial, el tiempo
disponible, los eventos visibles y el porcentaje de cobertura esperado. Esos
valores se ajustan sin cambiar la arquitectura mediante `target_coverage`,
`max_steps` y los archivos de configuración.

## Container and source dependencies

From VS Code, use **Dev Containers: Rebuild and Reopen in Container** after
changing `.devcontainer/Dockerfile`. The container needs ROS 2 Jazzy, Gazebo
Harmonic, `ros_gz`, RViz, NumPy, and pytest. The network must be available on the
first launch so Gazebo can cache the upstream ground-plane model.

Install the official simulation sources in a separate dependency workspace
inside the container. The package does not need the upstream DQN training stack.
These commands assume that `/ros2_ws/src/turtleboot3_autonomous_nav` is mounted:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p /home/ros/tb3_dependencies/src
git clone --branch jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git \
  /home/ros/tb3_dependencies/src/turtlebot3_simulations
git -C /home/ros/tb3_dependencies/src/turtlebot3_simulations checkout \
  45633014a14e8f438495b532a723e4ad45cbbd31
sudo apt-get update
sudo apt-get install -y ros-jazzy-ros-gz ros-jazzy-rviz2 python3-numpy python3-pytest
cd /home/ros/tb3_dependencies
colcon build --packages-select turtlebot3_gazebo
source install/setup.bash
cd /ros2_ws
colcon build --packages-select turtleboot3_autonomous_nav
source install/setup.bash
pytest src/turtleboot3_autonomous_nav/test -v
```

Source the dependency workspace and `/ros2_ws/install/setup.bash` in each new
terminal. The launchers select Burger and include the official
`turtlebot3_dqn_stage4.launch.py`. A narrow adapter gates upstream's GUI-only
include, and the launchers add the official obstacle plugin library directory.
The official stamped-velocity subscription is remapped; a one-way bridge
connects this package's `geometry_msgs/Twist` to Gazebo's velocity topic.

## Train and run

The GPU LiDAR still needs a working rendering backend when the GUI is disabled.
In the supplied VNC container, set `DISPLAY=:1` if it is not already configured.
Both launchers default to `use_sim_time:=true`; all commands below run inside
the container after sourcing both workspaces.

```bash
export DISPLAY=:1
cd /ros2_ws
ros2 launch turtleboot3_autonomous_nav training.launch.py \
  episodes:=100 use_gui:=false model_directory:=/ros2_ws/models
```

`episodes` counts training episodes; evaluation episodes are additional. The
default configuration evaluates every 10 training episodes over 3 greedy runs.
Fewer than 10 training episodes with that configuration will not produce a
checkpoint.
At the episode limit the trainer requests an acknowledged controller stop and
exits; stop the remaining simulator launcher with Ctrl-C. Change schedules and
rewards with `training_config:=/absolute/file.yaml`.

For a short integration exercise that performs one training and one evaluation
episode (not a useful trained policy):

```bash
ros2 launch turtleboot3_autonomous_nav training.launch.py episodes:=1 \
  use_gui:=false \
  training_config:=/ros2_ws/src/turtleboot3_autonomous_nav/config/smoke_training.yaml \
  model_directory:=/ros2_ws/models/smoke
```

After training has produced a checkpoint and the training launcher is stopped:

```bash
ros2 launch turtleboot3_autonomous_nav mission.launch.py \
  model_path:=/ros2_ws/models/best.pt use_rviz:=true use_gui:=true
```

For a headless mission use `use_rviz:=false use_gui:=false`. `model_path` is
required so a mission cannot silently substitute untrained weights. Load only
trusted checkpoints: the current `.pt` format is Python pickle, not TorchScript.
The explorer also rejects an absent or empty model when run directly with
`ros2 run`. A mission stops at `max_steps:=500` actions or
`target_coverage:=0.75`, whichever comes first. Override these launch arguments
to set mission limits. On termination it disables the controller, which publishes
zero velocity and rejects later actions until explicitly enabled for a new run.

The controller starts disabled. The policy nodes use the acknowledged
`/safe_motion_controller/enable` (`std_srvs/SetBool`) service to enable motion.
Both disabling and enabling clear cached action, scan, odometry, coverage, and
recovery state, publish zero velocity, and establish a DDS publication cutoff.
Motion requires new scan, odometry, and action publications after that cutoff.
Scan and odometry expire after 0.5 seconds, actions after 1 second, using both
simulated age and a wall-clock watchdog. Negative ages after clock rewind also
stop motion. Automatic stall recovery lasts at most `recovery_duration` (2 seconds
by default), then gives forward policy actions another progress interval.

Each training reset acknowledges controller disable before resetting Gazebo,
then clears the mapper and `/observation_builder/reset` (`std_srvs/Trigger`).
The builder clears all cached inputs, rejects earlier DDS publications, and tags
observations with its new episode epoch. Only after these acknowledgements does
the trainer enable the controller and wait for fresh sensors and a matching
observation. Service discovery, responses, fresh sensors, and subsequent
observations each have a wall-clock `reset_timeout_seconds` deadline (10 seconds
by default). A timeout aborts training and requests a controller stop.

## Observe autonomous operation

RViz uses `odom` as its fixed frame and displays `/coverage_map`, `/scan`, and
`/odom`. A read-only visualizer converts `/cmd_vel` and `/coverage_metrics` into
standard RViz markers on `/exploration_status`: the command arrow and text show
linear/angular commands and current coverage. No custom RViz plugins are needed.

To demonstrate autonomous operation, start only the mission, leave all keyboard
control nodes stopped, and watch the map reveal new cells as the robot moves.
In a second terminal:

```bash
ros2 topic echo /exploration_action
ros2 topic echo /cmd_vel
ros2 topic echo /coverage_metrics
ros2 topic info /cmd_vel --verbose
ros2 node info /safe_motion_controller
ros2 node list
```

Stop each `topic echo` with Ctrl-C before the next command. The velocity topic
must have one publisher named `/safe_motion_controller`. The graph must contain
no navigation, localization, map-server, or teleoperation nodes. Coverage is the
fraction of observed cells in the configured 20 m × 20 m grid, including occupied
cells; it is not the fraction of reachable free space. The default 75% training
target may be unattainable in this bounded world, so step and stall limits also
end episodes. Short smoke results establish integration, not policy quality.

For a bounded automated check while the mission is running:

```bash
python3 /ros2_ws/src/turtleboot3_autonomous_nav/test/integration_probe.py --seconds 30
```

It reports topic counts, first/last known-cell counts, velocity publishers, and
node names, and exits unsuccessfully if coverage does not grow, motion is absent,
or another velocity publisher or excluded node is present.

## Outputs and limits

`model_directory` receives `best.pt` and `best.metrics.json` whenever mean
evaluation coverage improves. The checkpoint preserves online/target weights,
network dimensions, configuration, and evaluation metrics. It does not contain
replay memory or optimizer state for exact training resumption. JSON includes
mean coverage, total episode index, evaluation count, and training step count.

Live topics include `/coverage_map` (OccupancyGrid), `/coverage_metrics`
(Float32), `/dqn_observation` (86 floats), `/exploration_action` (Int32),
`/safety_intervention` and `/recovery_active` (Bool). `/training_metrics` contains
episode, step, coverage, cumulative reward, epsilon, and best mean evaluation
coverage, in that order. Maps are live only. Record the
topics with `ros2 bag record` if persistent demonstration data is needed.

Mapping uses odometry without loop closure, and the policy is environment
specific. Run a full training/evaluation campaign before making claims about
coverage or collision performance.

Training loads Burger into Gazebo's initial world state. Gazebo Sim 8.11.0
removes a Burger spawned after startup when `reset.all` restores that initial
state, which stops odometry even though an old laser sensor can keep publishing.
The training launcher derives a temporary copy of the official stage4 world
with the standard Burger model included at the origin and removes it on launch
shutdown. The upstream world, moving obstacles, robot state publisher, and
bridges are preserved; the later robot creation process is omitted.

Full world reset, controller disable/enable acknowledgements, mapper and
observation reset epochs, and fresh sensor requirements remain enforced. A
Gazebo 8.11.0 smoke run completed two training and two evaluation episodes and
saved a reloadable checkpoint. See the
[reset investigation and verification](../../../docs/verification/2026-09-05-gazebo-training-reset.md)
for the reproduction and partial-reset comparisons. This verifies the episode
integration, not a learned policy's exploration quality.
