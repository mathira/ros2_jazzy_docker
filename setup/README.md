# Setup Docker del proyecto

Esta carpeta contiene toda la infraestructura para ejecutar el proyecto en un
contenedor Docker común. No requiere Dev Containers ni extensiones de VS Code.

Desde la raíz del repositorio:

```bash
./setup/setup.sh       # primera vez: imagen, repositorios, rosdep y build
./setup/up.sh          # siguientes veces: build de imagen y arranque
docker compose -f setup/docker-compose.yml exec ros2 bash
```

Dentro del contenedor, el workspace está en `/ros2_ws` y el código se monta
desde `ros2_ws/` del repositorio. Para recompilar después de cambios:

```bash
docker compose -f setup/docker-compose.yml exec ros2 setup/build.sh
```

El contenedor expone RViz/Gazebo mediante noVNC en
<http://localhost:6080/vnc.html>. La contraseña inicial es `ros`.

Al abrir noVNC se inicia automáticamente una terminal Xfce dentro del
contenedor, con ROS 2 y `/ros2_ws/install` cargados. Esa terminal tiene TTY
real, por lo que también sirve para ejecutar `turtle_py/teleop_turtle` y usar
las teclas de movimiento desde el navegador.

Las nuevas ventanas de terminal se abren con Bash y el usuario `ros`; no es
necesario usar `sudo`, `su` ni ingresar una contraseña.

Para detenerlo:

```bash
docker compose -f setup/docker-compose.yml down
```

La imagen instala ROS 2 Jazzy, Gazebo Harmonic, Nav2, Stage, TurtleBot3,
RViz, herramientas de compilación, Python científico y PyTorch CPU. El build
por defecto compila los paquetes usados por las misiones del proyecto; los
componentes opcionales de hardware TurtleBot3/cartographer se pueden compilar
aparte si se agregan sus dependencias. Los
repositorios externos se importan desde los manifiestos versionados en
`ros2_ws/`. Los artefactos `build/install/log` quedan en volúmenes Docker del
proyecto, fuera del árbol fuente, y se regeneran ejecutando nuevamente los
comandos anteriores.
