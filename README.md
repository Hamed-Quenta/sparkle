# Plataforma educativa para la implementación de algoritmos de robótica móivl en entornos reales y simulados
## Configuración del espacio de trabajo de la plataforma
Para poder trabajar correctamente con el presente repositorio es necesario configurar un espacio de trabajo en la computadora del robot. Es necesario clonar este repositorio en tu máquina local. Sigue los pasos detallados a continuación para configurar de forma correcta al robot y a tu máquina local.

### Requisitos
* Sistema operativo Ubuntu 22.04
* Conexión inalámbrica a una red local

### Instalación de ROS 2
Debes asegurarte de tener instalada la versión adecuada de ROS 2 tanto en la computadora del robot como en una máquina local. Se utilizó ROS 2 Humble LTS para el desarrollo de esta plataforma, dicha versión es compatible con Ubuntu 22.04. Las instrucciones detalladas de instalación se encuentran en la [Documentación de Humble](https://docs.ros.org/en/humble/Installation.html).

Se recomienda hacer una instalación completa de `ros-humble-desktop` para la máquina local y una instalación simplificada `ros-humble-ros-base` para la computadora del robot.

### Instalación de dependencias
Existen 4 dependencias principales para el correcto funcionamiento del robot:

#### micro-ROS
Si no tiene este paquete debe instalarlo usando `apt-get`:
```bash
sudo apt-get install ros-humble-micro-ros-setup
```
O puede instalar desde la fuente usando el [repositorio oficial](https://github.com/micro-ROS/micro_ros_setup) (debe asegurarse de usar el branch que corresponde a su distribución de ROS).

Asimismo, una vez instalado `micro-ros-setup` es necesario crear y configurar un agente para comunicarse con la placa de microcontrolador:
```bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
```
Para ejecutar el nodo de comunicación con la placa de microcontrolador se debe asegurar que esta está conectada con la computadora del robot mediante un cable USB.
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

#### RPLiDAR
Para instalar este paquete puede utilizar `apt-get`:
```bash
sudo apt-get install ros-humble-rplidar-ros
```
O puede instalar desde la fuente usando el [repositorio de Slamtec](https://github.com/Slamtec/rplidar_ros/tree/ros2) (debe asegurarse de usar el branch de `ros2`).

Para ejecutar el nodo correspondiente al sensor montado en el robot (RPLiDAR A1) se debe ejecutar:
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py
```
#### BNO055
Esta dependencia es necesaria para publicar los datos obtenidos por el sensor BNO055 montado en el robot. Es posible instalar el paquete usando `apt-get`:
```bash
sudo apt-get install ros-humble-bno055
```
O puede instalar desde la [fuente](https://github.com/flynneva/bno055.git).

Para ejecutar el nodo correspondiente debe asegurarse que el sensor esté conectado al Bus 7 de la computadora del robot mediante los pines I2C:
```bash
ros2 run bno055 bno055 --ros-args --params-file ./src/bno055/bno055/params/bno055_i2c_params.yaml
```

#### Depthai-ROS
Esto es necesario para publicar datos de imagen usando la cámara Luxonis OAK-D del robot. Es posible instalar esta dependencia usando `apt-get`:
```bash
sudo apt-get install ros-humble-depthai-ros
```
En este caso es recomendable instalar desde el [repositorio de Luxonis](https://github.com/luxonis/depthai-ros/tree/humble).

### Creación del workspace
Debe abrir una terminal y ejecutar los siguientes comandos para crear el workspace del robot:
```bash
mkdir -p sparkle_ws/src
cd sparkle_ws/src
git clone https://github.com/Hamed-Quenta/sparkle.git
cd ..
colcon build
source install/setup.bash
```

### Lanzamiento del robot real
Para publicar todos los tópicos relacionados al robot real se debe ejecutar el comando:
```bash
ros2 launch sparkle_bringup robot.launch.py
```

Debería ver los siguientes tópicos publicados:
1. `/left_ticks` y `/right_ticks`: Contiene datos de conteo de los encoders de las ruedas izquierda y derecha, respectivamente. 
2. `/cmd_vel`: Contiene las velocidades lineales y angulares de entrada del robot.
3. `/odom_data_quat` y `/odom_data_euler`: Contienen los datos de odometría de rueda con la orientación expresada en cuaterniones y ángulos de euler, respectivamente.
4. `/scan`: Contiene las lecturas del sensor LiDAR publicadas a una frecuencia de 5.5Hz.
5. `/bno055/imu`: Contiene las lecturas de aceleración y orientación del sensor BNO055.
6. `/oak/rgb/image_raw`: Contiene imágenes de la cámara RGB de la cámara OAK-D.
7. `/oak/depth/image_raw`: Contiene imágenes de la cámara de profundidad de la cámara OAK-D.
8. `/tf`: Contiene las transformaciones de posición y orientación entre todos los links del robot.
