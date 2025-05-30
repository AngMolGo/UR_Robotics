# Estación de trabajo virtual: Polyscope (browser) y simulador (Rviz)

### Requisitos del sistema:
 - [x] Ubuntu 24.04 lts
 - [x] ROS2 (Humble/Jazzy/Rolling)
 - [x] Docker

## Polyscope (Browser)

La interfaz de usuario de UR Polyscope se puede ejecutar de 2 maneras: directamente por Docker o por ROS2. En ambos casos se creará un contenedor de Docker que expondrá 2 puertos para poder comunicarnos con la interfaz gráfica de Polyscope.

### Opción 1: Docker (recomendado)

Ejecutar el siguiente comando:

``` bash
docker run --rm -it \
    -p 5900:5900 -p 6080:6080 \
    -v "${HOME}/Projects/UR_robotics/ur5_devs:/ursim/programs" \
    -e ROBOT_MODEL=UR5 \
    --network=ursim_net \
    universalrobots/ursim_e-series
```

De esta manera tenemos más control sobre el contenedor y podemos gestionar la red de comunicación hacia otros contenedores y simuladores.

<!-- Opción 2: ROS2 -->
<details>
  <summary>

  ### Opción 2: ROS2

  </summary>
    
  > [!NOTE]
  > 1. Se recomienda tener nociones de desarrollo en ROS2 para usar esta opción.
  > 2. Se necesita tener instalado el paquete Universal_Robots_ROS2_Driver correctamente compilado. En la siguiente sección *"Simulador (Rviz)"* se describe cómo instalar el paquete. 

  ``` bash
  source /opt/ros/{$ROS_DISTRO}/setup.bash  # Source ROS2
  source $COLCON_WS/install/setup.bash      # Source del workspace
  ros2 run ur_client_library start_ursim.sh -m ur20
  ```

De esta manera obtenemos el mismo resultado, un link que tiene acceso a un contenedor donde se estará ejecutando una aplicación web de Polyscope.

</details>

---

### Vista de la interfaz:

![Polyscope in browser](media/readme/polyscope_in_browser.png)


## Simulador (Rviz)

Primero, se necesita instalar el package _**ur_robot_driver**_ y las dependencias necesarias para ejecutar el programa. La documentación de UR nos proporciona el [tutorial para instalar el package](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/installation.html).

> [!WARNING]
> **Siempre siempre siempre siempreeeee** antes de instalar algún programa o paquete en Linux Ubuntu actualizar la lista de paquetes:
> ``` bash
> sudo apt update && apt upgrade
> ```

Con el paquete previamente instalado:

``` bash
cd $COLCON_WS
source install/setup.bash
ros2 launch ur_robot_driver \
     ur_control.launch.py \
     ur_type:=ur5 \
     robot_ip:=<ip_of_polyscope> \
     launch_rviz:=true
```
> [!NOTE]
> Para que el simulador pueda detectar una configuración definida, primero tiene que estar corriendo el contenedor de Docker.

Para mostrar el estado (posición y velocidad) de las articulaciones:
``` bash
ros2 topic echo /joint_states
```

O para una mejor vista:

``` bash
ros2 topic echo /joint_states --filter \
    "print(
    f'\n  Joint\t  Base\tShoulder Elbow\tWrist1\tWrist2\tWrist3\n'
    f'Position {m.position[0]:.3f}\t{m.position[1]:.4f}\t{m.position[2]:.4f}\t{m.position[3]:.4f}\t{m.position[4]:.4f}\t{m.position[5]:.4f}\n'
    f'Velocity {m.velocity[0]:.3f}\t{m.velocity[1]:.4f}\t{m.velocity[2]:.4f}\t{m.velocity[3]:.4f}\t{m.velocity[4]:.4f}\t{m.velocity[5]:.4f}'
    ) or False"
```

### Vista de la interfaz:

![Rviz simulator](media/readme/rviz.png)

## Simulador completo

Al ejecutar ambos programas tenemos una estación robot-teachpendant completamente virtual:

![Simulador completo](media/readme/virtual_station.gif)

## Networking

### Conexión desde una tableta electrónica

Una implementación de esta aplicación es abrir el url del contenedor en una tableta, de esta manera pretendemos tener un teach-pendant en nuestras manos para controlar a nuestro robot simulado. Para esto se necesita crear una red de Docker de tipo `macvlan` con salida a nuestra red local física y realizar un bridge a nuestra interfaz de red.

![Polyscope desde una tableta electrónica](media/readme/teach-pendant_tablet_test.gif)

---

### Conexión Modbus mediante una red virtual

También se puede establecer una comunicación entre dos contenedores vía Modbus a través de la sub-red de Docker.

![Rutina Pick & Place con 2 robots vía modbus](media/readme/rutina_pick_and_place_dos_robots_modbus.gif)

# Simulador en ROS2 Gazebo (adicional)

Adicionalmente podemos conectar la simulación de Rviz con el simulador Gazebo para simular el robot en un entorno 3D dinámico, interactuando con otros robots, objetos y sensores.

1. Crear el workspace:

``` bash
# Creamos el workspace
mkdir ws_ROS2_ursim && cd ws_ROS2_ursim
export COLCON_WS=$PWD
mkdir -p $COLCON_WS/src
# Descargamos el package de URsim
git clone -b ros2 https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git src/ur_simulation_gz
# Instalamos dependencias
source /opt/ros/jazzy/setup.bash #...primero debemos obtener ROS2...
rosdep update && rosdep install --ignore-src --from-paths src -y
# Compilamos el paquete
cd $COLCON_WS
colcon build --symlink-install
```

2. Para ejecutar el paquete:

``` bash
source /opt/ros/jazzy/setup.bash
source $COLCON_WS/install/setup.bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py
```

3. Para realizar un movimiento de prueba (hola mundo):

``` bash
ros2 run ur_robot_driver example_move.py
```

![Gazebo simulator](media/readme/gazebo_and_rviz.png)

---

### Para clonar este repositorio
``` bash
# Debido a que contiene repositorios "anidados" se deben tratar como sub-módulos.
# Para clonar el repositorio con los submódulos:
git clone --recurse-submodules https://github.com/AngMolGo/UR_Robotics
```

---

### Referencias y ligas de interés:
- Análisis comparativo entre RoboDK y ROS: https://www.researchgate.net/publication/348312857_Comparative_Analysis_of_RoboDK_and_Robot_Operating_System_for_Solving_Diagnostics_Tasks_in_Off-Line_Programming
- Documentación DockerHub de contenedor de Universal Robots: https://hub.docker.com/r/universalrobots/ursim_e-series
- Repositorio de Universal_Robots_ROS2_Driver: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
- Guía de instalación de ROS2 Jazzy: https://docs.ros.org/en/jazzy/Installation.html
- Guía de instalación de Docker Engine en Ubuntu: https://docs.docker.com/engine/install/ubuntu/
- Tutorial oficial de ROS2: https://docs.ros.org/en/jazzy/Tutorials.html
- Tutorial oficial de Docker: https://docs.docker.com/get-started/introduction/whats-next/