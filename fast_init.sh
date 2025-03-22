#!/bin/bash

echo "¡Hola, mundo! Este es mi primer script de Bash." 

# Inicializar ROS2
source /opt/ros/jazzy/setup.bash
# Inicializar entorno de la aplicación
source /home/angmolgo/Projects/UR_robotics/ws_ROS2_ursim/install/setup.bash

# Inicializar docker
contenedor=$(docker run --rm -d -p 5900:5900 -p 6080:6080 -v "${HOME}/Projects/UR_robotics/ur5_devs:/ursim/programs" -e ROBOT_MODEL=UR5 --network=ursim_net universalrobots/ursim_e-series)

# Espera 5 segundos:
sleep 3

# Abre el contenedor con Polyscope en el browser
gio open "http://192.168.56.2:6080/vnc.html?host=192.168.56.2&port=6080" &> /dev/null

# Espera a que
echo "¿Ya inicializaste los joints?"
read opcion

# Iniciar simulación:
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.56.2 launch_rviz:=true

# Eliminar contenedor
docker rm -f $contenedor
