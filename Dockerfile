FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# 安装系统依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    coinor-libipopt-dev \
    gfortran \
    liblapack-dev \
    pkg-config \
    swig \
    python3-pip \
    vim \
    git \
	libatlas-base-dev \
	python3-scipy \
    && rm -rf /var/lib/apt/lists/*

# install CasADi
WORKDIR /tmp
RUN git clone https://github.com/casadi/casadi.git \
    && cd casadi \
    && mkdir build && cd build \
    && cmake -DWITH_PYTHON=ON -DWITH_IPOPT=ON -DWITH_OPENMP=ON -DWITH_THREAD=ON .. \
    && make -j$(nproc) \
    && make install \
    && cd ../.. \
    && rm -rf casadi \
    && ldconfig

# install Turtlebot3-related packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-turtlebot3-msgs \
    && rm -rf /var/lib/apt/lists/*

# install POT
RUN pip install -i https://pypi.tuna.tsinghua.edu.cn/simple pot

