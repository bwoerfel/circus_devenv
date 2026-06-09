# =============================================================================
# CA-1 Motor Controller – Development Image
# Base: osrf/ros:humble-desktop (Ubuntu 22.04 + ROS 2 Humble Hawksbill)
# =============================================================================

FROM osrf/ros:humble-desktop

# -----------------------------------------------------------------------------
# Build arguments
# -----------------------------------------------------------------------------
ARG ROS_DISTRO=humble
ARG USERNAME=newton
ARG USER_UID=1000
ARG USER_GID=1000

# Prevent interactive prompts during apt
ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------------------------------------------------------
# Dev tools
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      python3-venv \
      python3-pip \
      git \
      gdb \
      clang-format \
      clang-tidy \
      bash-completion \
      wget \
      vim \
      sudo \
      doxygen \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Non-root user (matches host user to avoid permission issues)
# -----------------------------------------------------------------------------
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# -----------------------------------------------------------------------------
# Switch to non-root user and set up shell environment
# -----------------------------------------------------------------------------
USER ${USERNAME}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source ~/ros2_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc \
    && echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc \
    && echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc

# -----------------------------------------------------------------------------
# Workspace
# -----------------------------------------------------------------------------
RUN mkdir -p /home/${USERNAME}/ros2_ws/src

WORKDIR /home/${USERNAME}/ros2_ws

ENV ROS_DISTRO=${ROS_DISTRO}

CMD ["/bin/bash"]
