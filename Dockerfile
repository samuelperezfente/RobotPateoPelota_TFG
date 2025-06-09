FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV YOLO_CONFIG_DIR=/tmp
ENV ROBOT_IP="10.56.43.98"
ENV CONN_TYPE="webrtc"

# 1. Instalar herramientas de red, pip y ROS adicionales
RUN apt update && apt install -y \
    git \
    python3-pip \
    clang \
    portaudio19-dev \
    python3-colcon-common-extensions \
    libcanberra-gtk-module \
    libgl1-mesa-glx \
    libx11-dev \
    net-tools \
    iputils-ping \
    curl \
    gnupg2 \
    lsb-release \
    # Paquetes ROS usados en el proyecto
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro \
    ros-humble-robot-localization \
    ros-humble-ros2-controllers \
    ros-humble-ros2-control \
    ros-humble-velodyne \
    ros-humble-velodyne-gazebo-plugins \
    ros-humble-velodyne-description \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-navigation2 \
    ros-humble-twist-mux \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# 2. Instalar paquetes Python globales
RUN pip3 install --no-cache-dir \
    ultralytics \
    numpy==1.24.4

# 3. Instalar requerimientos específicos del proyecto
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# 4. Crear workspace (será montado por volumen luego)
WORKDIR /ros2_ws

# 5. Comando por defecto al arrancar el contenedor
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && cd /ros2_ws && exec bash"]

