# syntax=docker/dockerfile:1
#
# Native ARM64-compatible Dockerfile for micro-ROS Remote Lab
# Uses ubuntu:22.04 directly instead of crosslab/edrys_pyxtermjs (amd64-only),
# replicating the base image setup inline so everything runs natively on ARM64.
#
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# ============================================================
# 1. System packages (replicate base image + additions)
# ============================================================
RUN apt-get update \
 && apt-get upgrade -y \
 && apt-get install -y \
    locales \
    curl \
    sudo \
    software-properties-common \
    apt-utils \
    python3 \
    python3-pip \
    vim \
    htop \
    zsh \
    git \
    wget \
    tmux \
    nano \
    unzip \
 && add-apt-repository universe \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ============================================================
# 2. Create appuser (UID 1000, GID 1000) — matches base image
# ============================================================
RUN groupadd -g 1000 appuser \
 && useradd -m -u 1000 -g 1000 -s /bin/zsh appuser \
 && mkdir -p /var/www \
 && chown appuser:appuser /var/www

WORKDIR /var/www

# ============================================================
# 3. Install pyxtermjs + dependencies via pip
# ============================================================
RUN pip3 install --no-cache-dir \
    pyxtermjs \
    flask-cors \
    eventlet

# ============================================================
# 4. Setup oh-my-zsh for appuser
# ============================================================
RUN su - appuser -c \
    'sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended' \
 && sed -i 's/^ZSH_THEME=.*/ZSH_THEME="robbyrussell"/' /home/appuser/.zshrc

# ============================================================
# 5. ROS 2 Humble (GPG key + apt repo + desktop + dev-tools)
# ============================================================
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
 && apt-get update \
 && apt-get upgrade -y \
 && apt-get install -y ros-humble-desktop ros-dev-tools \
 && rm -rf /var/lib/apt/lists/*

# ============================================================
# 6. MoveIt2
# ============================================================
RUN apt-get update \
 && apt-get install -y ros-humble-moveit \
 && rm -rf /var/lib/apt/lists/*

# ============================================================
# 7. ros2_control + controllers + topic_tools + xacro
# ============================================================
RUN apt-get update \
 && apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-topic-tools \
    ros-humble-xacro \
 && rm -rf /var/lib/apt/lists/*

# ============================================================
# 8. rosdep, colcon, pip
# ============================================================
RUN apt-get update \
 && apt-get install -y \
    python3-rosdep \
    python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# ============================================================
# 9. rosdep init/update
# ============================================================
RUN rosdep init || true && rosdep update

# ============================================================
# 10. Source ROS 2 in bashrc/zshrc
# ============================================================
RUN echo "source /opt/ros/humble/setup.bash" >> /home/appuser/.bashrc \
 && echo "source /opt/ros/humble/setup.zsh" >> /home/appuser/.zshrc

# ============================================================
# 11. Fix zsh compfix permissions
# ============================================================
SHELL ["/bin/zsh", "-c"]
RUN ZSH_DISABLE_COMPFIX=true \
 && chmod g-w,o-w /home/appuser/.oh-my-zsh/plugins/*

# ============================================================
# 12. Arduino CLI (arch-detect: ARM64 on aarch64, x86_64 elsewhere)
# ============================================================
SHELL ["/bin/bash", "-c"]
RUN ARCH=$(dpkg --print-architecture) \
 && if [ "$ARCH" = "arm64" ]; then \
      ARDUINO_ARCH="ARM64"; \
    else \
      ARDUINO_ARCH="64bit"; \
    fi \
 && curl -fsSL "https://github.com/arduino/arduino-cli/releases/download/v1.3.1/arduino-cli_1.3.1_Linux_${ARDUINO_ARCH}.tar.gz" \
    -o /tmp/arduino-cli.tar.gz \
 && tar -xzf /tmp/arduino-cli.tar.gz -C /tmp \
 && mv /tmp/arduino-cli /usr/local/bin/arduino-cli \
 && chmod +x /usr/local/bin/arduino-cli \
 && rm /tmp/arduino-cli.tar.gz

# ============================================================
# 13. Arduino CLI config + Teensy board manager + core
# ============================================================
RUN arduino-cli config init \
 && arduino-cli config add board_manager.additional_urls \
    https://www.pjrc.com/teensy/package_teensy_index.json \
 && arduino-cli config set network.connection_timeout 300s \
 && arduino-cli core update-index \
 && arduino-cli core install teensy:avr

# ============================================================
# 14. teensy-loader-cli
# ============================================================
RUN apt-get update \
 && apt-get install -y teensy-loader-cli \
 && rm -rf /var/lib/apt/lists/*

# ============================================================
# 15. micro-ROS Arduino library (v2.0.8-humble)
# ============================================================
RUN mkdir -p /root/Arduino/libraries \
 && rm -rf /root/Arduino/libraries/micro_ros_arduino \
 && cd /tmp \
 && wget -O micro_ros_arduino_v2.0.8-humble.zip \
    https://github.com/micro-ROS/micro_ros_arduino/archive/refs/tags/v2.0.8-humble.zip \
 && unzip micro_ros_arduino_v2.0.8-humble.zip \
 && mv micro_ros_arduino-2.0.8-humble /root/Arduino/libraries/micro_ros_arduino \
 && rm -f micro_ros_arduino_v2.0.8-humble.zip

# ============================================================
# 16. SCServo library (for wrist SCS15 servo)
# ============================================================
RUN arduino-cli lib install SCServo

# ============================================================
# 17. Patch Teensy platform.txt for micro-ROS precompiled libs
# ============================================================
RUN TEENSY_AVR_DIR=$(find /root/.arduino15/packages/teensy/hardware/avr -maxdepth 1 -mindepth 1 -type d | head -1) \
 && cp /root/Arduino/libraries/micro_ros_arduino/extras/patching_boards/platform_teensy.txt \
    "${TEENSY_AVR_DIR}/platform.txt"

# ============================================================
# 18. Copy Arduino packages + libraries to appuser
# ============================================================
RUN mkdir -p /home/appuser/.arduino15 \
 && mkdir -p /home/appuser/Arduino \
 && cp -r /root/.arduino15/packages /home/appuser/.arduino15/ \
 && cp -r /root/Arduino/libraries /home/appuser/Arduino/ \
 && chown -R appuser:appuser /home/appuser/.arduino15 \
 && chown -R appuser:appuser /home/appuser/Arduino

# ============================================================
# 19. XPRA + X11 packages
# ============================================================
RUN apt-get update \
 && apt-get install -y \
    xpra \
    xvfb \
    xauth \
    x11-apps \
    dbus-x11 \
    x11-utils \
 && debconf-set-selections <<< "keyboard-configuration keyboard-configuration/layout select us" \
 && debconf-set-selections <<< "keyboard-configuration keyboard-configuration/variant select us" \
 && debconf-set-selections <<< "keyboard-configuration keyboard-configuration/model select pc105" \
 && debconf-set-selections <<< "keyboard-configuration keyboard-configuration/xkb-keymap select us" \
 && apt-get install -y keyboard-configuration \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# ============================================================
# 20. start-xpra.sh script
# ============================================================
RUN cat << 'EOF' > /home/appuser/start-xpra.sh
#!/bin/bash

export XDG_RUNTIME_DIR="$HOME/.run"
mkdir -p "$XDG_RUNTIME_DIR"

export DISPLAY=:100

# --- Clean previous XPRA/Xvfb sessions ---
pkill -f "xpra.*:100" 2>/dev/null || true
pkill -f "Xvfb :100" 2>/dev/null || true
rm -rf "$XDG_RUNTIME_DIR/xpra"
rm -f /tmp/.X11-unix/X100
rm -f /tmp/.X100-lock

# Start XPRA clean every time
xpra start :100 --bind-tcp=0.0.0.0:14500 --html=on --daemon=yes --xvfb="Xvfb :100 -screen 0 1280x800x24"

exec "$@"
EOF

RUN chmod +x /home/appuser/start-xpra.sh

# ============================================================
# 21. User permissions (dialout, video, sudo, password)
# ============================================================
RUN usermod -aG dialout appuser \
 && usermod -aG video appuser \
 && echo "appuser ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/appuser \
 && echo "appuser:a" | chpasswd

# ============================================================
# 22. /workspace directory
# ============================================================
RUN mkdir -p /workspace \
 && chown -R appuser:appuser /workspace \
 && chmod -R 755 /workspace

# ============================================================
# 23. microros_install helper script
# ============================================================
RUN cat << 'EOF' > /usr/local/bin/microros_install
#!/bin/bash
set -e

MICROROS_WS="$HOME/workspace/microros_ws"

echo "=== micro-ROS Build System Installation ==="

source /opt/ros/humble/setup.bash

mkdir -p "$MICROROS_WS/src"

if [ ! -d "$MICROROS_WS/src/micro_ros_setup" ]; then
    echo "Cloning micro_ros_setup..."
    git clone -b humble \
        https://github.com/micro-ROS/micro_ros_setup.git \
        "$MICROROS_WS/src/micro_ros_setup"
else
    echo "micro_ros_setup already exists."
fi

cd "$MICROROS_WS"
sudo apt update
echo "Updating rosdep..."
rosdep update

echo "Installing dependencies..."
rosdep install --from-paths src --ignore-src -y

echo "Building workspace..."
colcon build

if ! grep -q "microros_ws/install/local_setup.bash" "$HOME/.bashrc"; then
    echo "" >> "$HOME/.bashrc"
    echo "# micro-ROS workspace" >> "$HOME/.bashrc"
    echo "source $MICROROS_WS/install/local_setup.bash" >> "$HOME/.bashrc"
fi

source install/local_setup.bash

echo ""
echo "======================================"
echo "micro-ROS build system installed."
echo "Workspace:"
echo "$MICROROS_WS"
echo "======================================"
EOF

RUN chmod +x /usr/local/bin/microros_install

# ============================================================
# 24. microros_clean helper script
# ============================================================
RUN cat << 'EOF' > /usr/local/bin/microros_clean
#!/bin/bash
set -e

MICROROS_WS="$HOME/workspace/microros_ws"

echo "Removing micro-ROS workspace..."

rm -rf "$MICROROS_WS"

sed -i '\|microros_ws/install/local_setup.bash|d' "$HOME/.bashrc"

echo "Done."
echo "Run:"
echo "  microros_install"
echo "to create a fresh workspace."
EOF

RUN chmod +x /usr/local/bin/microros_clean

# ============================================================
# 25. Switch to appuser
# ============================================================
USER appuser

# ============================================================
# 26. Expose ports
# ============================================================
EXPOSE 5000
EXPOSE 11311
EXPOSE 7400
EXPOSE 7410
EXPOSE 7420

# ============================================================
# 27. Entrypoint: start-xpra.sh -> pyxtermjs with zsh
# ============================================================
ENTRYPOINT ["/home/appuser/start-xpra.sh", "python3", "-m", "pyxtermjs", "--host", "0.0.0.0", "--command", "zsh"]
