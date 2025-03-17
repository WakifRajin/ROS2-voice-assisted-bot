# Use ROS 2 Humble as base image
FROM ros:humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-rclpy \
    ros-humble-ros2launch \
    portaudio19-dev && \  
    rm -rf /var/lib/apt/lists/*  # ✅ Make sure this is on the same line as &&

# Set up the workspace
WORKDIR /ros2_2025

# Copy the package source code
COPY . /ros2_2025

# Install Python dependencies (EXCEPT rclpy)
RUN grep -v "rclpy" requirements.txt > temp_requirements.txt && \
    pip3 install --no-cache-dir -r temp_requirements.txt

# Ensure ROS 2 environment is sourced
SHELL ["/bin/bash", "-c"]

# Build the ROS 2 package
RUN source /opt/ros/humble/setup.bash && colcon build

# Source the environment when container starts
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_2025/install/setup.bash" >> /root/.bashrc

# Default command when container starts
CMD ["/bin/bash"]

