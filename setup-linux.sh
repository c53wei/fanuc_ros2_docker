#!/bin/bash
# FANUC ROS2 Docker Setup for Linux (Ubuntu 24.04)

set -e

echo "=========================================="
echo "FANUC ROS2 Docker Setup for Linux"
echo "=========================================="
echo ""

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Error: Docker is not running!"
    echo "Please start Docker and try again."
    exit 1
fi

echo "✅ Docker is running"
echo ""

# Allow Docker to access X server
echo "🔧 Configuring X11 permissions..."
xhost +local:docker > /dev/null 2>&1
echo "✅ X11 access granted to Docker"
echo ""

# Build the image
echo "🏗️  Building Docker image (this may take ~10 minutes)..."
docker-compose -f docker-compose.linux.yml build

if [ $? -eq 0 ]; then
    echo "✅ Docker image built successfully"
else
    echo "❌ Error building Docker image"
    exit 1
fi
echo ""

# Start the container
echo "🚀 Starting container..."
docker-compose -f docker-compose.linux.yml up -d

if [ $? -eq 0 ]; then
    echo "✅ Container started successfully"
else
    echo "❌ Error starting container"
    exit 1
fi
echo ""

echo "=========================================="
echo "✅ Setup Complete!"
echo "=========================================="
echo ""
echo "📋 Quick Start:"
echo ""
echo "  Access the container:"
echo "    docker exec -it fanuc-ros2-humble bash"
echo ""
echo "  Test RViz2 (inside container):"
echo "    source /opt/ros/humble/setup.bash"
echo "    source /root/ws_fanuc/install/setup.bash"
echo "    ros2 run rviz2 rviz2"
echo ""
echo "  Launch FANUC robot with MoveIt (inside container):"
echo "    ros2 launch fanuc_moveit_config fanuc_moveit.launch.py \\"
echo "      robot_model:=crx10ia use_mock:=true"
echo ""
echo "  Stop the container:"
echo "    docker-compose -f docker-compose.linux.yml down"
echo ""
echo "  Launch with visualization service:"
echo "    docker-compose -f docker-compose.linux.yml --profile visualization up"
echo ""
echo "📚 For more information, see README.md"
echo ""

