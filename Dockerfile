FROM docker.io/library/ubuntu:noble

ARG USERNAME="seymour"
ARG HOME_DIR="/home/${USERNAME}"
ARG WORKSPACE="${HOME_DIR}/workspace"
ARG DDS_CONFIG_DIR="/opt/dds/config"
ARG DEBIAN_FRONTEND="noninteractive"
ARG RUN_AS_UID=1000
ARG RUN_AS_GID=1000

ENV LANGUAGE=en
ENV COUNTRY=US
ENV LOCALE=UTF-8
ENV LANG="${LANGUAGE}_${COUNTRY}.${LOCALE}"
ENV LC_ALL="${LANG}"

ENV ROS_DISTRO="jazzy"
ENV CYCLONEDDS_URI="${DDS_CONFIG_DIR}/cyclonedds.xml"
ENV FASTRTPS_DEFAULT_PROFILES_FILE="${DDS_CONFIG_DIR}/fastrtps.xml"

# rmw implementation can be overridden at runtime
# RMW_IMPLEMENTATION -> "rmw_cyclonedds_cpp" | "rmw_fastrtps_cpp"
ENV RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"


################################################################################
# setup utc timeszone
# Enable Ubuntu Universe property
#
#
RUN  apt update \
  && apt install -y -q \
           locales \
           software-properties-common \
           tzdata \
  && apt autoremove -y \
  && add-apt-repository universe \
  && rm -rf /var/lib/apt/lists/* \
  && locale-gen ${LANG}  ${LANG} \
  && update-locale LC_ALL=${LANG} LANG=${LANG}


################################################################################
# install base ubuntu packages
RUN  apt update \
  && apt install -q -y --no-install-recommends \
           bash-completion \
           build-essential \
           curl \
           dirmngr \
           emacs \
           git \
           gnupg2 \
           iproute2 \
           net-tools \
           ssh \
           python-is-python3 \
           python3-pip \
           sudo \
           usbutils \
           wget \
           x11-apps \
           xauth \
  && apt autoremove -y \
  && rm -rf /var/lib/apt/lists/*


################################################################################
# setup ros package overlay & install ros packages
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
           http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt update \
  && apt install -q -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-debugpy \
    python3-rosdep \
    python3-vcstool \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-ros-gz \
  && apt autoremove -y \
  && rm -rf /var/lib/apt/lists/*


################################################################################
# setup colcon mixin and metadata
RUN colcon mixin add default \
        https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
  && colcon mixin update default \
  && colcon metadata add default \
         https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml \
  && colcon metadata update


################################################################################
## Build and Install Dynamixel SDK 
WORKDIR /tmp/sdk
RUN git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git \
  && cd DynamixelSDK/c++/build/linux64 \
  && make \
  && make install

################################################################################
## Build and Install plotjuggler, from source
##
RUN  apt update \
  && apt install -y -q \
                 binutils-dev \
                 cmake \
                 protobuf-compiler \
                 liblua5.1-0-dev \
                 libmosquittopp-dev \
                 liblz4-dev \
                 libprotoc-dev \
                 libzmq3-dev \
                 libqt5opengl5-dev \
                 libqt5svg5-dev \
                 libqt5websockets5-dev \
                 libqt5x11extras5-dev \
                 libzstd-dev \
                 nlohmann-json3-dev \
		 qtbase5-dev \
                 ros-${ROS_DISTRO}-ament-cmake \
                 ros-${ROS_DISTRO}-ament-cmake-core \
                 ros-${ROS_DISTRO}-ament-cmake-export-dependencies \
                 ros-${ROS_DISTRO}-ament-cmake-libraries \
                 ros-${ROS_DISTRO}-ament-cmake-python  \
                 ros-${ROS_DISTRO}-ament-cmake-ros \
                 ros-${ROS_DISTRO}-ament-cmake-target-dependencies \
                 ros-${ROS_DISTRO}-ament-package \
                 ros-${ROS_DISTRO}-rclcpp \
                 ros-${ROS_DISTRO}-rclcpp-components \
                 ros-${ROS_DISTRO}-rosidl-default-generators \
                 ros-${ROS_DISTRO}-rosidl-default-runtime \
     && apt autoremove -y \
     && apt clean  \
     && rm -rf /var/lib/apt/lists/*  \
     && rosdep init \
     && rosdep update --rosdistro ${ROS_DISTRO} \
     && mkdir -p /opt/plotjuggler/src  \
     && cd /opt/plotjuggler/src  \
     && git clone https://github.com/PlotJuggler/plotjuggler_msgs.git  \
     && git clone https://github.com/facontidavide/PlotJuggler.git  \
     && git clone https://github.com/PlotJuggler/plotjuggler-ros-plugins.git  \
     && cd /opt/plotjuggler  \
     && . /opt/ros/${ROS_DISTRO}/setup.sh  \
     && export AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}  \
     && export CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO} \
     && export ROS_VERSION=2  \
     && export ROS_PYTHON_VERSION=3  \
     && rosdep install --from-paths src --ignore-src -y  \
     && colcon build \
            --cmake-args \
            -DCMAKE_BUILD_TYPE=RelWithDebInfo \
            -DROS_DISTRO=${ROS_DISTRO} \
            -DROS_VERSION=2 \
            -DCMAKE_PREFIX_PATH="/opt/ros/${ROS_DISTRO}"


########################################################################
## Install VSCode: (or code-insiders) directly from Microsoft
RUN  wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg \
  && install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg \
  && echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" |sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null \
  && rm -f packages.microsoft.gpg \
  && apt update \
  && apt install code -q -y \
  && apt autoremove -y \
  && apt clean \
  && rm -rf /var/lib/apt/lists/


########################################################################
# Remove default ubuntu user
# create non-root user with given username
# and allow sudo without password
# and setup default users .bashrc
RUN userdel -r ubuntu \
  && groupadd --gid $RUN_AS_GID ${USERNAME} \
  && useradd -rm \
    -d ${HOME_DIR} \
    -s /bin/bash \
    --gid ${RUN_AS_GID} \
    --uid ${RUN_AS_UID} \
    -G users,nogroup,dialout,sudo,adm \
    -m ${USERNAME} \
  && passwd -d ${USERNAME} \
  && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
  && chmod 0440 /etc/sudoers.d/${USERNAME} \
  && sed -i s/"\${debian_chroot:+(\$debian_chroot)}"/"docker-\${debian_chroot:+(\$debian_chroot)}"/g    ${HOME_DIR}/.bashrc \
  && echo "source /opt/ros/${ROS_DISTRO}/setup.bash"           >> ${HOME_DIR}/.bashrc \
  && echo "source /opt/plotjuggler/install/setup.bash"         >> ${HOME_DIR}/.bashrc \
  && echo "SETUP=\"\$(find ${WORKSPACE} -name setup.bash)\""   >> ${HOME_DIR}/.bashrc \
  && echo "[[ -n \"\${SETUP}\" ]] && source \${SETUP}"         >> ${HOME_DIR}/.bashrc \
  && echo "export PATH=\$PATH:/opt/plotjuggler/install/plotjuggler/lib/plotjuggler" >> ${HOME_DIR}/.bashrc \
  && echo "[[ \$LD_LIBRARY_PATH != */usr/local/lib* ]] && export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib"  >> ${HOME_DIR}/.bashrc \
  && echo "[[ \$LD_LIBRARY_PATH != */opt/plotjuggler/install/plotjuggler/lib/plotjuggler* ]] && export LD_LIBRARY_PATH=/opt/plotjuggler/install/plotjuggler/lib/plotjuggler:\$LD_LIBRARY_PATH"  >> ${HOME_DIR}/.bashrc \
  && chown -R ${USERNAME}:${USERNAME}  ${HOME_DIR}

########################################################################
# create workspace and source dir
RUN mkdir -p ${WORKSPACE} \
  && chown  -R  ${USERNAME}:${USERNAME}  ${HOME_DIR}
WORKDIR ${WORKSPACE}

# setup dds config
ADD ./dds_config ${DDS_CONFIG_DIR}

########################################################################
# install deps as non-root user
WORKDIR ${WORKSPACE}
USER ${USERNAME}

# RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
#   && sudo apt update
#   && sudo rosdep init
#   && rosdep update --rosdistro ${ROS_DISTRO} \
#   && rosdep install -y -r -i --from-paths ${WORKSPACE} \
#   && sudo rm -rf /var/lib/apt/lists/*"

# by default hold container open in background
CMD ["tail", "-f", "/dev/null"]
