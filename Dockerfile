FROM crosslab/edrys_pyxtermjs:latest

# switch to root to install packages
USER root

# Basic system setup
RUN DEBIAN_FRONTEND=noninteractive apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y locales curl sudo software-properties-common \
 && DEBIAN_FRONTEND=noninteractive add-apt-repository universe \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && export LANG=en_US.UTF-8

# ROS2 setup
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
 && DEBIAN_FRONTEND=noninteractive apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y ros-humble-desktop ros-dev-tools

# Install MoveIt2 (binary, stable)
RUN DEBIAN_FRONTEND=noninteractive apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-moveit \
 && rm -rf /var/lib/apt/lists/*

# Install ros2_control + controllers
RUN DEBIAN_FRONTEND=noninteractive apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
 && rm -rf /var/lib/apt/lists/*

# micro-ROS dependency management
RUN DEBIAN_FRONTEND=noninteractive apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-pip \
 && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update


RUN apt update && apt install -y ros-humble-xacro


# Source ROS2 in bash and zsh
RUN echo "source /opt/ros/humble/setup.bash" >> /home/appuser/.bashrc \
 && echo "source /opt/ros/humble/setup.zsh" >> /home/appuser/.zshrc

# Fix zsh warnings
SHELL ["/bin/zsh", "-c"]
RUN ZSH_DISABLE_COMPFIX=true \
 && chmod g-w,o-w /home/appuser/.oh-my-zsh/plugins/*

# Install tmux
RUN apt-get update && apt-get install -y tmux

# Install Arduino CLI
RUN curl -fsSL https://github.com/arduino/arduino-cli/releases/download/v1.3.1/arduino-cli_1.3.1_Linux_64bit.tar.gz -o /tmp/arduino-cli.tar.gz \
 && tar -xzf /tmp/arduino-cli.tar.gz -C /tmp \
 && mv /tmp/arduino-cli /usr/local/bin/arduino-cli \
 && chmod +x /usr/local/bin/arduino-cli \
 && rm /tmp/arduino-cli.tar.gz


# Configure Arduino CLI
RUN arduino-cli config init

# Add Teensy board manager
RUN arduino-cli config add board_manager.additional_urls \
    https://www.pjrc.com/teensy/package_teensy_index.json

# Install Teensy support (increase timeout for large downloads)
RUN arduino-cli config set network.connection_timeout 300s \
 && arduino-cli core update-index \
 && arduino-cli core install teensy:avr

# Install teensy_loader_cli for headless firmware uploads
RUN apt-get update && apt-get install -y teensy-loader-cli && rm -rf /var/lib/apt/lists/*

# Install micro-ROS Arduino library (ROS 2 Humble)
# Install unzip (needed for GitHub release package)
RUN apt-get update && apt-get install -y unzip

# Install micro-ROS Arduino library (ROS 2 Humble)
RUN mkdir -p /root/Arduino/libraries \
 && rm -rf /root/Arduino/libraries/micro_ros_arduino \
 && cd /tmp \
 && wget -O micro_ros_arduino_v2.0.8-humble.zip \
    https://github.com/micro-ROS/micro_ros_arduino/archive/refs/tags/v2.0.8-humble.zip \
 && unzip micro_ros_arduino_v2.0.8-humble.zip \
 && mv micro_ros_arduino-2.0.8-humble /root/Arduino/libraries/micro_ros_arduino \
 && rm -f micro_ros_arduino_v2.0.8-humble.zip

# Patch Teensy platform for micro-ROS precompiled libraries
RUN TEENSY_AVR_DIR=$(find /root/.arduino15/packages/teensy/hardware/avr -maxdepth 1 -mindepth 1 -type d | head -1) \
 && cp /root/Arduino/libraries/micro_ros_arduino/extras/patching_boards/platform_teensy.txt \
    "${TEENSY_AVR_DIR}/platform.txt"
    
# Make Arduino packages available to appuser
RUN mkdir -p /home/appuser/.arduino15 \
 && mkdir -p /home/appuser/Arduino \
 && cp -r /root/.arduino15/packages /home/appuser/.arduino15/ \
 && cp -r /root/Arduino/libraries /home/appuser/Arduino/ \
 && chown -R appuser:appuser /home/appuser/.arduino15 \
 && chown -R appuser:appuser /home/appuser/Arduino


 # Install nano editor
RUN apt-get update && apt-get install -y nano

# Install Xpra (headless GUI streaming) + X11 support
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
      xpra \
      xvfb \
      xauth \
      x11-apps \
      dbus-x11 \
      x11-utils && \
    # Pre-configure keyboard to avoid dialog prompts
    debconf-set-selections <<< "keyboard-configuration keyboard-configuration/layout select us" && \
    debconf-set-selections <<< "keyboard-configuration keyboard-configuration/variant select us" && \
    debconf-set-selections <<< "keyboard-configuration keyboard-configuration/model select pc105" && \
    debconf-set-selections <<< "keyboard-configuration keyboard-configuration/xkb-keymap select us" && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration && \
    # Clean cache
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Create startup script for XPRA (ensuring display :100 is always free)
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

# Start XPRA clean every time
xpra start :100 --bind-tcp=0.0.0.0:14500 --html=on --daemon=yes --xvfb="Xvfb :100 -screen 0 1280x800x24"

exec "$@"
EOF

RUN chmod +x /home/appuser/start-xpra.sh



# Give appuser access to serial & video devices
RUN usermod -aG dialout appuser && usermod -aG video appuser

# Give appuser sudo privileges with password "a"
RUN echo "appuser ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/appuser \
 && echo "appuser:a" | chpasswd

# Give appuser access to serial & video devices
RUN usermod -aG dialout appuser && usermod -aG video appuser

# Create a persistent workspace for students
RUN mkdir -p /workspace \
 && chown -R appuser:appuser /workspace \
 && chmod -R 755 /workspace


# micro-ROS helper script
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

#microros clean script
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

# get back to the default appuser
USER appuser

# Expose ports
EXPOSE 5000
EXPOSE 11311
EXPOSE 7400
EXPOSE 7410
EXPOSE 7420

# Start pyxtermjs with zsh
#ENTRYPOINT python3 -m pyxtermjs --cors True --host 0.0.0.0 --command zsh

ENTRYPOINT ["/home/appuser/start-xpra.sh", "python3", "-m", "pyxtermjs", "--cors", "True", "--host", "0.0.0.0", "--command", "zsh"]

