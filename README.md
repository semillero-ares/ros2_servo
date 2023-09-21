# Servo hardware for arduino

Creado a partir del [tutorial de Articulated Robotics](https://youtu.be/J02jEKawE5U). 
El cual esta basado en el demo_2 de [ros2_control_demos](https://control.ros.org/humble/doc/ros2_control_demos/example_2/doc/userdoc.html).

Se presenta el paquete de ROS 2 y el código de arduino para controlar un servo desde ROS 2 a traves del puerto de comunicación serial. 

## Requisitos

Se necesita instalar la siguiente libreria para poder compilar el paquete

```bash
sudo apt install libserial-dev
```

## Compilación

Para compilar los dos paquetes usaremos colcon, debemos estar parado en el directorio _workspace_, y ejecutamos

```bash
colcon build
```

## Corriendo el ejemplo del servo

Para correr el ejemplo deberemos instalar unos paquetes:

```bash
sudo apt install ros-humble-moveit ros-humble-moveit-resources -y
```

Con esta instalación, se instalarán también los paquetes adicionales listados a continuación: 

```bash
ros-humble-eigen-stl-containers
ros-humble-geometric-shapes
ros-humble-launch-param-builder
ros-humble-moveit-common
ros-humble-moveit-configs-utils
ros-humble-moveit-core
ros-humble-moveit-kinematics
ros-humble-moveit-msgs
ros-humble-moveit-planners
ros-humble-moveit-planners-ompl
ros-humble-moveit-plugins
ros-humble-moveit-ros
ros-humble-moveit-ros-benchmarks
ros-humble-moveit-ros-move-group
ros-humble-moveit-ros-occupancy-map-monitor
ros-humble-moveit-ros-planning
ros-humble-moveit-ros-planning-interface
ros-humble-moveit-ros-robot-interaction
ros-humble-moveit-ros-visualization
ros-humble-moveit-ros-warehouse
ros-humble-moveit-setup-app-plugins
ros-humble-moveit-setup-assistant
ros-humble-moveit-setup-controllers
ros-humble-moveit-setup-core-plugins
ros-humble-moveit-setup-framework
ros-humble-moveit-setup-srdf-plugins
ros-humble-moveit-simple-controller-manager
ros-humble-object-recognition-msgs
ros-humble-octomap
ros-humble-octomap-msgs
ros-humble-pilz-industrial-motion-planner
ros-humble-random-numbers
ros-humble-ruckig
ros-humble-srdfdom
ros-humble-urdfdom-py
ros-humble-warehouse-ros

ros-humble-moveit-resources-fanuc-description
ros-humble-moveit-resources-fanuc-moveit-config
ros-humble-moveit-resources-panda-description
ros-humble-moveit-resources-panda-moveit-config
ros-humble-moveit-resources-pr2-description
```

Una vez instalado, vamos a correr el ejemplo con el siguiente codigo:

```bash
source install/setup.bash
ros2 launch servo_hardware_moveit_config mock.launch.py
```

Con el servo real correr:

```bash
source install/setup.bash
ros2 launch servo_hardware_moveit_config demo.launch.py
```

y en otro terminal 

```bash
socat -d -d pty,rawer,echo=0,link=$HOME/servo  /dev/ttyACM0,b115200,raw
```