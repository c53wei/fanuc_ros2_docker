# FANUC ROS 2 Humble Docker Environment

This Docker setup provides a complete environment for running the FANUC ROS 2 Driver on Ubuntu 22.04 with ROS 2 Humble Hawksbill.

**✅ macOS Compatible** - This configuration has been tested and optimized for macOS (Apple Silicon and Intel).

## What's Included

- Ubuntu 22.04 (Jammy)
- ROS 2 Humble Desktop (full installation)
- FANUC Description packages (CRX, LR Mate, M-series, R-series)
- FANUC Driver package (with submodules)
- FANUC MoveIt configuration
- All necessary dependencies and build tools
- Pre-compiled and ready to use (~5GB image)

## Prerequisites

- Docker Desktop installed and running on your system
- Docker Compose (included with Docker Desktop)

## Building and Running with Docker Compose (Recommended)

### 1. Build the Docker Image

```bash
cd ~/fanuc_ros2_docker
docker-compose build
```

**Note:** This will take approximately 10 minutes as it downloads and builds all ROS 2 dependencies and FANUC packages.

### 2. Start the Container

```bash
docker-compose up -d
```

### 3. Access the Container

```bash
docker exec -it fanuc-ros2-humble bash
```

### 4. Stop the Container

```bash
docker-compose down
```

## Running with Plain Docker (Alternative)

### Basic Usage (Terminal Only)

```bash
docker run -it --rm fanuc-ros2-humble
```

### Linux with Display Support

```bash
xhost +local:docker

docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --network host \
  fanuc-ros2-humble
```

### With Network Access (for Physical Robot)

**Note:** `--network host` does not work properly on macOS (Docker runs in a VM). Use port mapping instead if needed.

**Linux:**
```bash
docker run -it --rm \
  --network host \
  fanuc-ros2-humble
```

## Quick Start Commands

Once inside the container, the ROS 2 environment is already sourced. Try these commands:

### List Available FANUC Packages

```bash
ros2 pkg list | grep fanuc
```

Available packages include:
- `fanuc_crx_description` - CRX robot URDF models
- `fanuc_lrmate_description` - LR Mate robot URDF models  
- `fanuc_m10_description`, `fanuc_m20_description` - M-series robot URDF models
- `fanuc_r1000ia_description`, `fanuc_r2000_description` - R-series robot URDF models
- `fanuc_moveit_config` - MoveIt configuration for motion planning
- `fanuc_hardware_interface` - Hardware interface for real robots
- `fanuc_controllers` - Custom controllers for FANUC robots
- `fanuc_msgs` - Custom ROS 2 messages for FANUC

### Visualize URDF Model (Requires Display)

```bash
ros2 launch fanuc_crx_description view_crx.launch.py robot_model:=crx10ia
```

### Launch with Mock Hardware (Requires Display for RViz)

```bash
ros2 launch fanuc_moveit_config fanuc_moveit.launch.py robot_model:=crx10ia use_mock:=true
```

### Launch with Physical Hardware

```bash
ros2 launch fanuc_moveit_config fanuc_moveit.launch.py robot_model:=crx10ia robot_ip:="192.168.1.100"
```

### Test ROS 2 Installation (Terminal Only)

```bash
# Check ROS 2 version
ros2 --version

# List available topics
ros2 topic list

# Run a simple node
ros2 run demo_nodes_cpp talker
```

## Supported Robot Models

- CRX-5iA
- CRX-10iA
- CRX-10iA/L
- CRX-20iA/L
- CRX-30iA
- CRX/30-18A

## Additional Resources

- [FANUC ROS 2 Driver Documentation](https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/quick_start/quick_start.html)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)

## Troubleshooting

### General Issues

1. **Docker daemon not running**
   - Ensure Docker Desktop is running before building or starting containers
   - Check the Docker Desktop icon in your system tray/menu bar

2. **Build takes too long or fails**
   - Ensure Docker has sufficient resources (RAM: 8GB+, CPU: 4+ cores)
   - Check your internet connection (downloads ~3GB of packages)
   - Try building again: `docker-compose build --no-cache`

3. **Container won't start**
   - Check logs: `docker-compose logs fanuc-ros2`
   - Verify the image built successfully: `docker images | grep fanuc`


### Getting Help

- Check that all git submodules were cloned correctly
- Verify network configuration for physical hardware connections
- Consult the official FANUC documentation linked above

## Support

Contact your local FANUC sales representative for hardware-specific support.

