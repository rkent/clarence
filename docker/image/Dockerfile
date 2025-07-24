ARG DISTRO="rolling"
ARG VARIANT="ros-base"

FROM ros:${DISTRO}-${VARIANT}
ARG DISTRO="rolling"
ARG VARIANT="ros-base"

RUN echo DISTRO:$DISTRO VARIANT:$VARIANT
RUN apt update && apt install -y less git nano sudo python-is-python3 \
  python3-venv python3-pip iproute2 iputils-ping
RUN apt upgrade -y 

# RUN apt install -y ros-$DISTRO-$VARIANT ros-dev-tools
#RUN rosdep init

ARG USERNAME=ros
ARG UID=1000
ARG GID=1000
ARG USER_UID=${UID}
ARG USER_GID=${GID}
ARG GPIO_ID


RUN userdel -r ubuntu || true
RUN groupadd --gid ${USER_GID} $USERNAME && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
&& usermod -aG sudo ${USERNAME} \
&& passwd -d ${USERNAME} || true

RUN mkdir /workspace
RUN chown $USER_UID:$USER_GID /workspace

# Create the gpio group, and add the user to it.
RUN groupadd -g ${GPIO_ID} gpio
RUN usermod -a -G gpio ${USERNAME}

USER ${USERNAME}
COPY docker/image/bashrc /home/${USERNAME}/.bashrc
COPY docker/image/requirements.txt /tmp/requirements.txt
COPY docker/image/ros_entrypoint.sh /
RUN pip install --break-system-packages -r /tmp/requirements.txt
# Get rid of of opening sudo message.
RUN touch ~/.sudo_as_admin_successful
# Git user name and email
COPY docker/.gitconfig /home/${USERNAME}/.gitconfig

RUN rosdep update
# These are needed by RPi to determine Raspberry pi version
ENV RPI_LGPIO_REVISION=0xd04170
ENV RPI_LGPIO_CHIP=0

WORKDIR /workspace
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
