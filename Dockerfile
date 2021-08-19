FROM arm64v8/ros:melodic
ENV DEBIAN_FRONTEND noninteractive

# install dependencies via apt
ENV DEBCONF_NOWARNINGS yes
RUN set -x && \
  apt-get update -y -qq && \
  apt-get upgrade -y -qq --no-install-recommends && \
  : "basic dependencies" && \
  apt-get install -y -qq \
    build-essential \
    pkg-config \
    cmake \
    git \
    wget \
    curl \
    tar \
    unzip && \
  : "g2o dependencies" && \
  apt-get install -y -qq \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libglew-dev && \
  : "OpenCV dependencies" && \
  apt-get install -y -qq \
    libjpeg-dev \
    libpng++-dev \
    libtiff-dev \
    libopenexr-dev \
    libwebp-dev \
    ffmpeg \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    libavresample-dev && \
  : "other dependencies" && \
  apt-get install -y -qq \
    libyaml-cpp-dev && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

ARG CMAKE_INSTALL_PREFIX=/usr/local
ARG NUM_THREADS=1

ENV CPATH=${CMAKE_INSTALL_PREFIX}/include:${CPATH}
ENV C_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include:${C_INCLUDE_PATH}
ENV CPLUS_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include:${CPLUS_INCLUDE_PATH}
ENV LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:${LIBRARY_PATH}
ENV LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:${LD_LIBRARY_PATH}

# Eigen
ARG EIGEN3_VERSION=3.3.7
WORKDIR /tmp
RUN set -x && \
  wget -q https://gitlab.com/libeigen/eigen/-/archive/${EIGEN3_VERSION}/eigen-${EIGEN3_VERSION}.tar.bz2 && \
  tar xf eigen-${EIGEN3_VERSION}.tar.bz2 && \
  rm -rf eigen-${EIGEN3_VERSION}.tar.bz2 && \
  cd eigen-${EIGEN3_VERSION} && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *
ENV Eigen3_DIR=${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake

# g2o
ARG G2O_COMMIT=9b41a4ea5ade8e1250b9c1b279f3a9c098811b5a
WORKDIR /tmp
RUN set -x && \
  git clone https://github.com/RainerKuemmerle/g2o.git && \
  cd g2o && \
  git checkout ${G2O_COMMIT} && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    -DCMAKE_CXX_FLAGS=-std=c++11 \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_UNITTESTS=OFF \
    -DG2O_USE_CHOLMOD=OFF \
    -DG2O_USE_CSPARSE=ON \
    -DG2O_USE_OPENGL=OFF \
    -DG2O_USE_OPENMP=ON \
    -DG2O_BUILD_APPS=OFF \
    -DG2O_BUILD_EXAMPLES=OFF \
    -DG2O_BUILD_LINKED_APPS=OFF \
    -DDO_SSE_AUTODETECT=OFF \
    -DDISABLE_SSE2=ON \
    -DDISABLE_SSE3=ON \
    -DDISABLE_SSE4_1=ON \
    -DDISABLE_SSE4_2=ON \
    -DDISABLE_SSE4_A=ON \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *
ENV g2o_DIR=${CMAKE_INSTALL_PREFIX}/lib/cmake/g2o

# OpenCV
ARG OPENCV_VERSION=4.1.0
WORKDIR /tmp
RUN set -x && \
  wget -q https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip && \
  unzip -q ${OPENCV_VERSION}.zip && \
  rm -rf ${OPENCV_VERSION}.zip && \
  cd opencv-${OPENCV_VERSION} && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    -DBUILD_DOCS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_JASPER=OFF \
    -DBUILD_OPENEXR=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_opencv_apps=OFF \
    -DBUILD_opencv_dnn=OFF \
    -DBUILD_opencv_ml=OFF \
    -DBUILD_opencv_python_bindings_generator=OFF \
    -DENABLE_CXX11=ON \
    -DENABLE_FAST_MATH=ON \
    -DWITH_EIGEN=ON \
    -DWITH_FFMPEG=ON \
    -DWITH_OPENMP=ON \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *
ENV OpenCV_DIR=${CMAKE_INSTALL_PREFIX}/lib/cmake/opencv4

# socket.io-client-cpp
ARG SIOCLIENT_COMMIT=ff6ef08e45c594e33aa6bc19ebdd07954914efe0
WORKDIR /tmp
RUN set -x && \
  git clone https://github.com/shinsumicco/socket.io-client-cpp.git && \
  cd socket.io-client-cpp && \
  git checkout ${SIOCLIENT_COMMIT} && \
  git submodule init && \
  git submodule update && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    -DBUILD_UNIT_TESTS=OFF \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *
ENV sioclient_DIR=${CMAKE_INSTALL_PREFIX}/lib/cmake/sioclient

# protobuf
WORKDIR /tmp
RUN set -x && \
  apt-get update -y -qq && \
  apt-get upgrade -y -qq --no-install-recommends && \
  apt-get install -y -qq autogen autoconf libtool && \
  wget -q https://github.com/google/protobuf/archive/v3.6.1.tar.gz && \
  tar xf v3.6.1.tar.gz && \
  cd protobuf-3.6.1 && \
  ./autogen.sh && \
  ./configure --prefix=${CMAKE_INSTALL_PREFIX} --enable-static=no && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf * && \
  apt-get purge -y -qq autogen autoconf libtool && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

# OpenVSLAM
ARG OPENVSLAM_COMMIT=0f8f400fe2c488ab4ed7459ac47304c47c415bee
WORKDIR /
RUN set -x && \
  git clone --recursive https://github.com/OpenVSLAM-Community/openvslam.git && \
  cd openvslam && \
  git checkout ${OPENVSLAM_COMMIT} && \
  mkdir -p build && \
  cd build && \
  CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO}/lib/cmake cmake \
    -DUSE_SOCKET_PUBLISHER=ON \
    -DINSTALL_SOCKET_PUBLISHER=ON \
    -DBUILD_EXAMPLES=ON \
    -DUSE_STACK_TRACE_LOGGER=ON \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  rm -rf CMakeCache.txt CMakeFiles Makefile cmake_install.cmake example src && \
  chmod -R 777 ./*

# ROS
RUN set -x && \
  apt-get update -y -qq && \
  : "install ROS packages" && \
  apt-get install -y -qq \
    ros-${ROS_DISTRO}-geometry2 \
    ros-${ROS_DISTRO}-image-transport && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

WORKDIR /root/catkin_ws/src

ARG OPENVSLAM_ROS_COMMIT=8de784454b6a49423110130c2df024b2054cf7be
RUN set -x && \
  : "build ROS packages" && \
  git clone https://github.com/OpenVSLAM-Community/openvslam_ros.git && \
  cd openvslam_ros && \
  git checkout ${OPENVSLAM_ROS_COMMIT} && \
  cd .. && \
  bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; \
  catkin_init_workspace ." && \
  git clone -b opencv4 https://github.com/fizyr-forks/vision_opencv.git && \
  cd .. && \
  bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; \
  catkin_make -j${NUM_THREADS} \
    -DBUILD_WITH_MARCH_NATIVE=OFF \
    -DUSE_PANGOLIN_VIEWER=OFF \
    -DUSE_SOCKET_PUBLISHER=ON \
    -DUSE_STACK_TRACE_LOGGER=ON \
    -DBOW_FRAMEWORK=FBoW"

RUN set -x && \
  sh -c "echo '#'\!'/bin/bash\nset -e\nsource /opt/ros/${ROS_DISTRO}/setup.bash\nsource /root/catkin_ws/devel/setup.bash\nexec \"\$@\"' > /ros_entrypoint.sh" && \
  chmod u+x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
