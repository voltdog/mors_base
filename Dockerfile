# Используем базовый образ ROS Noetic с полной десктопной средой
FROM osrf/ros:noetic-desktop-full

# Устанавливаем зависимости для симуляции робота
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y \
    ros-noetic-moveit \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros-control \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    ros-noetic-roboticsgroup-upatras-gazebo-plugins \
    ros-noetic-actionlib-tools \
    ros-noetic-joy \
    python3.10 \
    python3-pip \
    git \
    terminator \
    wget \
    curl \
    libopencv-dev \
    python3-opencv \
    libssl-dev \
    libffi-dev && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libxrandr2 \
    libxinerama1 \
    libxcursor1
#попытка решить проблемы с пайбуллетом
RUN apt-get update && apt-get install -y mesa-utils mesa-utils-extra && \
    rm -rf /var/lib/apt/lists/*
ENV LIBGL_ALWAYS_SOFTWARE=1


# Устанавливаем Python-библиотеки, необходимые для проекта
RUN pip install \
    flask==2.2.5 \
    flask-ask-sdk \
    ask-sdk \
    numpy==1.21.6 \
    control==0.9.4 \
    pyrr==0.10.3 \
    pybullet==3.2.5 \
    pyserial==3.5 \
    pyqtgraph==0.13.3 \
    gymnasium \
    pcl==0.0.0.post1 \
    pyquaternion==0.9.9 \
    evdev \
    pyudev \
    Sphinx==7.1.2 \
    myst-parser==2.0.0 \
    sphinx-rtd-theme==1.3.0 \
    lcm==1.4.4 \
    cryptography==41.0.2     

# Скачиваем и устанавливаем библиотеку LCM для обмена данными между процессами
RUN wget https://github.com/lcm-proj/lcm/archive/refs/tags/v1.5.0.zip && \
    unzip v1.5.0.zip && \
    cd lcm-1.5.0 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install && \
    rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && apt-get install -y python3-tk



# Клонируем и настраиваем проект робота-собаки
WORKDIR /home/user/mors_sim/
#RUN git clone https://github.com/voltbro/mors_dog_simulation.git . && \
 #   rosdep update && \
 #   rosdep install --from-paths src --ignore-src -r -y

# Собираем workspace
#RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Настраиваем рабочее окружение
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc && \
    echo 'source /home/user/mors_dog/mors_ws/devel/setup.bash' >> /root/.bashrc

# Запускаем контейнер в консоли
CMD ["/bin/bash"]

