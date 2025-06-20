# Voice Assisted TurtleBot3

## 📌 Overview
Voice Assisted TurtleBot3 is a ROS2-based project that enables controlling a TurtleBot3 in Gazebo simulation using voice commands. The robot responds to spoken commands for movement and energy management, making it a unique interactive experience.

## 🚀 Features
- 🎙️ **Voice-Controlled Navigation**: Move the robot using voice commands like "forward", "left", "right".
- 🔋 **Energy Management System**: The robot consumes energy while moving and can replenish energy using specific voice triggers like "heal" or "kaboom".
- 📡 **Real-time ROS2 Communication**: Uses `rclpy` to process commands and send movement instructions to `/cmd_vel`.
- 🐳 **Dockerized Deployment**: The entire project can be built and run inside a Docker container.

## 📁 Repository Structure
```
📂 voice_assist_bot
 ├── 📂 src/                     # ROS2 package source code
 ├── 📄 Dockerfile               # Dockerfile for containerization
 ├── 📄 requirements.txt         # Python dependencies
 ├── 📄 README.md                # Project documentation
 ├── 📄 setup.py                 # ROS2 package setup
```

## 🔧 Installation
### **1️⃣ Clone the repository**
```bash
git clone https://github.com/YOUR_USERNAME/voice_assist_bot.git
cd voice_assist_bot
```
### **2️⃣ Build the ROS2 Package**
```bash
colcon build
source install/setup.bash
```

## 🐳 Running with Docker

Dockerhub : https://hub.docker.com/repository/docker/wakifrajin/voice_assist_bot

### **1️⃣ Pull the Docker Image**
```bash
docker pull wakifrajin/voice_assist_bot
```
### **2️⃣ Run the Container**
```bash
docker run --device /dev/snd:/dev/snd -it wakifrajin/voice_assist_bot
```

## 🤓 Use the package

1. Source the package
   ```bash
   cd ~/ros2_2025
   source install/setup.bash
   ```
   or, if you use .zsh
   ```bash
   cd ~/ros2_2025
   source install/setup.zsh
   ```
2. Startup gazebo turtlebot3
   ```bash
   export TURTLEBOT3_MODEL=burger  # or waffle
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```
3. Run the launch file to run speaker and control nodes
   ```bash
   ros2 launch voice_assist_bot voice_assist_launch.py
   ```

## 🎤 Voice Commands
| Command      | Action |
|-------------|--------|
| "forward"    | Moves forward for 2 seconds |
| "forward 5s" | Moves forward for 5 seconds |
| "left"       | Rotates left |
| "right"      | Rotates right |
| "heal" / "kaboom" | Increases energy |

## 🤝 Contributing
Pull requests are welcome! Feel free to open an issue if you have any questions or suggestions.

## 📜 License
This project is licensed under the MIT License. See `LICENSE` for details.

