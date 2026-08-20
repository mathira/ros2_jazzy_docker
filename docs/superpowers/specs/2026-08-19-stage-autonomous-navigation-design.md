# Diseño: navegación autónoma con Stage y Nav2

## Objetivo

Incorporar al workspace un paquete ROS 2 Jazzy llamado `stage_autonomous_nav` que permita a un robot simulado en el mundo `cave` de `stage_ros2` navegar de forma autónoma desde su pose inicial hasta una meta seleccionada en RViz mediante **2D Goal Pose**.

La primera versión usa la odometría perfecta que publica Stage. No incluye AMCL ni SLAM: un transform estático `map -> odom` sitúa la odometría de la simulación en el mapa conocido. El láser alimenta el coste local de Nav2 y permite esquivar el bloque presente en el mundo aunque este no se represente inicialmente en el mapa estático.

## Alcance

Se incluirán:

- El paquete Python `stage_autonomous_nav`.
- Un launch integrado para Stage, Nav2 y RViz.
- Un mapa de ocupación que corresponda al bitmap de `cave`.
- Parámetros de Nav2 ajustados al robot móvil de Stage.
- Un RViz preconfigurado para enviar metas y observar el estado de navegación.
- Una definición reproducible de dependencias de workspace para `stage_ros2` y el Stage modificado recomendado por ese proyecto.
- Documentación de preparación, ejecución y verificación.

No se incluirán en esta iteración localización probabilística, SLAM, multirobot, ni modificación del mundo `cave`.

## Dependencias

`stage_ros2` se obtendrá de su rama `jazzy` junto con su dependencia Stage modificada. Se proveerá un archivo `.repos` para importar ambas fuentes al workspace. El contenedor instalará las dependencias de sistema y Nav2 para Jazzy. `package.xml` declarará las dependencias ROS de ejecución que consume directamente el paquete; la importación de repositorios externos se documentará y automatizará al nivel del workspace, no como una descarga implícita al construir el paquete.

## Arquitectura

```text
RViz (2D Goal Pose)
        | /goal_pose
        v
Nav2: planner, controller, behavior tree, recoveries
        | /cmd_vel (geometry_msgs/Twist)
        v
stage_ros2 / Stage ----> robot Pioneer simulado
       |                         |
       +-- /odom, /tf, /scan ----+

map_server -> mapa estático cave -> Nav2
static_transform_publisher: map -> odom
```

El launch fija el modo de un solo robot de Stage:

```bash
ros2 launch stage_ros2 stage.launch.py \
  world:=cave enforce_prefixes:=false one_tf_tree:=true
```

Nav2 recibe una meta en `/goal_pose` desde RViz y publica la ruta y comandos de velocidad. Stage ejecuta esos comandos y entrega odometría, TF y láser a Nav2. Los nombres de frames y tópicos se expondrán como argumentos o parámetros de launch para acomodar el contrato exacto publicado por Stage sin editar el código.

## Estructura prevista

```text
ros2_ws/
├── stage_nav.repos
└── src/
    └── stage_autonomous_nav/
        ├── config/nav2_cave.yaml
        ├── launch/cave_navigation.launch.py
        ├── maps/cave.yaml
        ├── maps/cave.pgm
        ├── rviz/cave_navigation.rviz
        ├── resource/stage_autonomous_nav
        ├── stage_autonomous_nav/
        ├── test/
        ├── package.xml
        ├── setup.cfg
        ├── setup.py
        └── README.md
```

El mapa utilizará la misma geometría, origen y resolución que el bitmap de 16 x 16 m del mundo `cave`. El bloque de Stage situado en `(5, 4)` se detectará inicialmente a través del láser y se incorporará al coste local. Se podrá añadir al mapa estático en una iteración posterior si se desea que el planificador global lo considere antes de medirlo.

## Configuración de Nav2

- `use_sim_time: true` en todos los nodos.
- `map` como frame global, `odom` como frame de odometría y frame base parametrizable.
- `map_server` carga `maps/cave.yaml`.
- Costmap global: mapa estático con inflación configurada según el radio del Pioneer.
- Costmap local: observación de láser, obstáculos y capa de inflación.
- Controlador y planificador estándar de Nav2, con límites de velocidad conservadores para Stage.
- Behavior Tree con cancelación, recuperación y parada segura.

## Launch y operación

`cave_navigation.launch.py` realizará, en este orden lógico:

1. Arrancar Stage con el mundo `cave` y TF único.
2. Publicar `map -> odom` estático.
3. Arrancar el stack de navegación de Nav2 y el servidor de mapas.
4. Iniciar RViz con mapa, TF, láser, coste local/global, ruta y la herramienta de metas.

La interfaz de usuario de la primera versión es RViz exclusivamente. El operador establece la meta con **2D Goal Pose**; Nav2 informa el resultado y detiene el robot cuando alcanza, cancela o aborta la tarea.

## Gestión de fallos

- Las comprobaciones de ciclo de vida de Nav2 impedirán activar la navegación si falta mapa, TF, odometría o láser.
- Los timeouts del controlador provocarán recuperación o aborto, según la configuración de Nav2.
- Cancelar una meta desde RViz o finalizar el launch publica velocidad cero antes de detener Stage.
- Los frames y tópicos se validarán al arrancar y se documentarán los remapeos necesarios si la versión de Stage utilizada difiere.

## Verificación

1. Las pruebas de instalación y lint comprueban que el paquete y sus recursos se instalan correctamente.
2. Una prueba de launch valida que la descripción se genera con el mundo `cave` y los argumentos esperados.
3. La comprobación manual de integración arranca el launch, espera a que Nav2 esté activo, establece una meta libre en RViz y confirma que se emiten comandos, el robot llega a la proximidad configurada y Nav2 devuelve resultado exitoso.
4. Una segunda comprobación sitúa una meta cuya trayectoria pasa junto al bloque para confirmar que el coste local usa el láser y evita la colisión.

## Criterio de aceptación

Con las dependencias importadas y el workspace construido, un único comando de launch abre Stage, Nav2 y RViz. Desde la pose inicial del Pioneer en `cave`, una meta libre enviada con RViz produce una ruta ejecutable y el robot llega a la tolerancia configurada sin chocar con paredes ni con el bloque detectado por el láser.
