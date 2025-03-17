(Para clonar este repositorio)
``` bash
git clone --recurse-submodules https://github.com/AngMolGo/UR_Robotics
```

### Simulador en ROS2 Gazebo

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

> [!NOTE]
> Para mostrar el estado de los joints (topic joints_state):
> ``` bash
> ros2 topic "bla bla bla"
> ```
