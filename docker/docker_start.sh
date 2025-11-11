#!/bin/bash

COMPOSE_FILE="docker-compose.yml"
SERVICE_NAME="gantry-sim"

xhost +local:docker

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    HOST_DISPLAY_VAR="$DISPLAY"
elif [[ "$OSTYPE" == "darwin"* || "$OSTYPE" == "msys"* || "$OSTYPE" == "win32"* ]]; then
    HOST_DISPLAY_VAR="host.docker.internal:0"
fi

# --- Build the Docker Compose services ---
echo "Building Docker Compose services..."
UID=$(id -u) GID=$(id -g) docker-compose -f "${COMPOSE_FILE}" build \
    --build-arg HOST_UID=$(id -u) \
    --build-arg HOST_GID=$(id -g) \
    "${SERVICE_NAME}"
if [ $? -ne 0 ]; then
    echo "Docker Compose build failed! Aborting."
    exit 1
fi
echo "Docker Compose build successful."

# --- Check and Stop/Remove existing container ---
echo "Running docker compose down to ensure a clean start..."
UID=$(id -u) GID=$(id -g) docker-compose -f "${COMPOSE_FILE}" down --remove-orphans > /dev/null 2>&1 || true
echo "Docker Compose cleanup completed."

# --- Run the Docker Compose service interactively ---
echo "Starting Docker Compose service '${SERVICE_NAME}' in interactive mode..."

UID=$(id -u) GID=$(id -g) docker-compose -f "${COMPOSE_FILE}" run \
    -e "HOST_DISPLAY_VAR=${HOST_DISPLAY_VAR}" \
    -e QT_X11_NO_MITSHM=1 \
    -e "USERNAME=ros" \
    "${SERVICE_NAME}" \
    bash

echo "Docker container session ended."
