#!/bin/bash
set -e

# 1. Clone franka_description
cd ./melon_ws/src
if [ ! -d "franka_description" ]; then
  git clone https://github.com/frankaemika/franka_description.git
fi

cd ../../

# 2. build docker image
build_docker_image(){
  if docker images | grep -q "melon_ros2_app.*latest"; then
    echo "Docker image 'melon_ros2_app:latest' already exists."
    read -p "Do you want to rebuild it? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
      echo "Starting container with existing image..."
      return
    fi
  fi

  echo "Building Docker image..."
  docker build -t melon_ros2_app:latest .

  if docker images | grep -q "melon_ros2_app.*latest"; then
    echo "Docker image 'melon_ros2_app:latest' built successfully."
  else
    echo "Failed to build Docker image 'melon_ros2_app:latest'."
    exit 1
  fi
}

# 3. run docker container
run_docker_container(){
  if [ -f .env ]; then
    export $(grep -v '^#' .env | xargs)
  fi

  # if docker ps -a | grep -q "${CONTAINER_NAME:-melon_ros2}"; then
  if docker ps -a --filter "name=^${CONTAINER_NAME:-melon_ros2}$" --format "{{.Names}}" | grep -q "^${CONTAINER_NAME:-melon_ros2}$"; then
    echo "Docker container '${CONTAINER_NAME:-melon_ros2}' is already created."
    return
  fi

  echo "Starting Docker container..."
  docker run -d \
    --name ${CONTAINER_NAME:-melon_ros2} \
    --network host \
    --ipc host \
    -e DISPLAY=${DISPLAY} \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-80} \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/root/.Xauthority:ro \
    -v $(pwd)/melon_ws:/root/melon_ws:rw \
    -v $(pwd)/pytwb_ws:/root/pytwb_ws:rw \
    -w /root/melon_ws \
    --tty \
    --gpus all \
    melon_ros2_app:latest \
    tail -f /dev/null
}

connect_to_container(){
  if [ -f .env ]; then
    export $(grep -v '^#' .env | xargs)
  fi

  if ! docker ps --filter "name=^${CONTAINER_NAME:-melon_ros2}$" --format "{{.Names}}" | grep -q "^${CONTAINER_NAME:-melon_ros2}$"; then
    echo "Docker container '${CONTAINER_NAME:-melon_ros2}' is not running."
    docker start ${CONTAINER_NAME:-melon_ros2}
  fi

  echo "Connecting to Docker container..."
  echo ""
  echo "==============================="
  echo ""
  echo "If this is your first time inside a container, first run the following command:"
  echo ""
  echo "For Melon ROS2 workspace:"
  echo "1. $ colcon build"
  echo "2. $ source install/setup.bash"
  echo ""
  echo "For Behavior Tree and ros_actor applications:"
  echo "1. $ cd /root/pytwb_ws"
  echo "2. $ pytwb"
  echo "3. > create cm1"
  echo "4. > Y"
  echo "5. > exit"
  echo "From now on, you can run the application by typing the command:"
  echo "$ actor"
  echo ""
  echo "==============================="
  docker exec -it ${CONTAINER_NAME:-melon_ros2} bash
}

main(){
  build_docker_image
  run_docker_container
  connect_to_container
}

main "$@"

