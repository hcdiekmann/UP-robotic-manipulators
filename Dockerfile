# To build this Dockerfile use command:
#   docker build -t amr-interbotix-arm .

ARG HUMBLE_DIGEST=sha256:f05e713cd384afe798581f9bbf312a249102fb37a184049b9457e9fef90d137e

FROM ros:humble@${HUMBLE_DIGEST}

SHELL ["/bin/bash", "-c"]

# install essential packages
RUN apt-get update && apt-get install -y \
    nano \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# intall required python packages
RUN apt-get update && pip3 install \
    transforms3d \
    setuptools==58.2.0 \
    modern_robotics 

# install AprilTag ROS 2 Wrapper (for camera-to-arm calibration)
RUN mkdir -p /home/apriltag_ws/src && \
    cd /home/apriltag_ws/src && \
    git clone https://github.com/Interbotix/apriltag_ros.git -b ros2-port && \
    cd /home/apriltag_ws && \
    source /opt/ros/humble/setup.bash && \
    apt-get update && \
    rosdep update --include-eol-distros && \
    rosdep install --from-paths src --ignore-src -r -y

# build AptilTag package
RUN source /opt/ros/humble/setup.bash && \
    cd /home/apriltag_ws && \
    colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w"

# install Interbotix ROS 2 manipulators packages
RUN mkdir -p /home/interbotix_ws/src && \
    cd /home/interbotix_ws/src && \
    git clone https://github.com/hcdiekmann/UP-robotic-manipulators.git -b humble && \
    cd UP-robotic-manipulators/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk && \
    cd /home/interbotix_ws && \
    source /opt/ros/humble/setup.bash && \
    rosdep update --include-eol-distros && \
    rosdep install --from-paths src --ignore-src -r -y

# build Interbotix manipulators packages
RUN source /opt/ros/humble/setup.bash && \
    cd /home/interbotix_ws && \
    colcon build

# setup env variables
RUN echo "# AAEON Interbotix configurations" >> ~/.bashrc && \
    echo 'export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)' >> ~/.bashrc && \
    echo -e 'if [ -z "$ROS_IP" ]; then\n\texport ROS_IP=127.0.0.1\nfi' >> ~/.bashrc && \
    echo "source /home/interbotix_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source /home/apriltag_ws/install/setup.bash" >> ~/.bashrc

