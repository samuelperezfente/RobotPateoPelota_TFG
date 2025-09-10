# RobotPateoPelota_TFG
Un robot cuadrúpedo **Unitree Go2** capaz de **detectar y patear una pelota** utilizando **ROS 2 Humble**, visión artificial con **YOLO**, y un sistema de control en el **robot real**.

---

## Objetivo del proyecto
El propósito de este Trabajo de Fin de Grado es el desarrollo de un sistema autónomo que permita a un robot cuadrúpedo:
- Detectar los objetivos de pateo y de destino mediante visión por computador. 
- Coordinar la alineación, aproximación y ejecución de un disparo controlado.  
- Validar el comportamiento en el robot físico.  

---

## Preparar el entorno

### Clonar el repositorio
```bash
git clone https://github.com/samuelperezfente/RobotPateoPelota_TFG.git
cd RobotPateoPelota_TFG
```

### Construcción del contenedor Docker
```bash
# Construir la imagen
docker build -t unitree-go2-ros2 .

# Ejecutar el contenedor
docker run -it --rm \
    --name unitree-go2 \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="direccion/carpeta/repositorio/RobotPateoPelota_TFG:/ros2_ws" \
    $(for dev in /dev/video*; do echo "--device=$dev"; done) \
    --device=/dev/bus/usb/001/003 \
    --privileged \
    unitree-go2-ros2
```
### Dependencias principales
El entorno de trabajo se encuentra preparado dentro del contenedor Docker, por lo que no es necesario instalarlas manualmente.
Entre las librerías y herramientas ya incluidas se encuentran:
- ROS 2 Humble  
- Gazebo  
- YOLO (Ultralytics)  
- OpenCV, NumPy, PyTorch

---

## Ejecución
### Conexión con el robot real
```bash
export ROBOT_IP="ip do robot"
export CONN_TYPE="webrtc"

cd src/go2_ros2_sdk/
colcon build
source install/setup.bash

ros2 launch go2_robot_sdk robot.launch.py
```

### Ejecución del comportamiento
```bash
ros2 run ball_kicking start_kick.py
```

---

## Documentación
- Canal de YouTube con las pruebas: https://www.youtube.com/@GolpeoBalonUnitreeGo2
- Repositorio de datos utilizados en el proyecto: https://github.com/samuelperezfente/RobotPateoPelota_TFG_Datos
