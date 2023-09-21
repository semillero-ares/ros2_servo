# Usando el asistente de Moveit

Para poder usar Moveit2 debemos crear unos archivos de configuración necesarios para correr moveit. Estos archivos se pueden generar en el Moveit-Assitant, lamentablemente este asistente solo está disponible en ROS1, por lo que tendremos que instalarlo. Para hacer la instalación de ROS1 en nuestro Ubuntu 22.04 y no tener problemas con nuestra instalación de ROS2, lo instalaremos en una ambiente virtual creado con mamba (_una versión más rápide de Anaconda_). 

## Instalando mamba para gestionar los ambientes virtuales

Inicialmente tendremos que [instalar mamba]((https://mamba.readthedocs.io/en/latest/mamba-installation.html)). Tomado de [aquí](https://github.com/conda-forge/miniforge#mambaforge), ejecutaremos los siguientes comandos:

```bash
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-Linux-x86_64.sh"
bash Mambaforge-Linux-x86_64.sh
```

Una vez instalado vamos a ir a la carpeta `mambaforge/bin` y ejecutaremos los siguientes comandos para activar mamba:

```bash
./conda init
./conda config --set auto_activate_base false
```

Reiniciamos el terminal y ahora podemos continuar con la instalación de ROS1

# La instalación de ROS1 en un ambiente virtual

_Sección inspirada en la documentación de [RoboStack](https://robostack.github.io/GettingStarted.html)_

Inicialmente vamos a crear el ambiente virtual, como la versión de ROS1 que vamos a instalar es la noetic, creare el ambiente virtual con el siguiente comando:

```bash
mamba create -n noetic_env 
mamba init
```

Luego de reiniciar el terminal, activaremos el ambiente virtual:

```bash
mamba activate noetic_env
```

Prepararemos el ambiente para instalar ROS1 de los repositorios correctos:

```bash
# this adds the conda-forge channel to the new created environment configuration 
conda config --env --add channels conda-forge
# and the robostack channel
conda config --env --add channels robostack-staging
# remove the defaults channel just in case, this might return an error if it is not in the list which is ok
conda config --env --remove channels defaults
```

El último comando puede generar un error por que algunas veces no existe, entonces no puede ser removido. 

Ya tenemos entonces el ambiente listo para instalar ROS1, que instalaremos con el siguiente comando:

```bash
# Install ros-noetic into the environment (ROS1)
mamba install ros-noetic-desktop
# Install tools for local development
mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools
```

Para poder usar ROS1, debemos desactivar ROS2 del archivo `~/.bashrc`, esto lo haremos comentando la linea mostrada a continuación:

```bash
# source /opt/ros/humble/setup.bash
```

y reiniciando el terminal. 

### Probando la instalación

En dos terminales haremos lo siguiente:

1. En el terminal 1, activaremos el ambiente y luego correremos el comando `roscore` que inicia el proceso principal de ROS1:

    ```bash
    mamba activate noetic_env
    roscore
    ```

2. En el terminal 2, activaremos el ambiente y correremos el comando de rviz:

    ```bash
    mamba activate noetic_env
    rviz
    ```  

## Instalando Moveit para ROS1 

Estando dentro del ambiente virtual (*noetic_env*), vamos a instalar moveit con el siguiente comando:

```bash
mamba install ros-noetic-moveit 
```

Una vez instalado podremos correr el assistente para moveit con el siguiente comando: 

```bash
roslaunch moveit_setup_assistant setup_assistant.launch
```

## Usando el asistente de MoveIt

https://www.youtube.com/watch?v=EosEikbZhiM

Para usar el asistente necesitaremos un archivo "URDF" :

```bash
ros2 topic echo /robot_description -l 1000000000 > data.urdf 
```