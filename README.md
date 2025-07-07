# Devastator

This repository contains the ROS2 workspace of the Devastator platform, together with the thesis documenting its capabilities and its extended abstract.

## How to Run (with Docker)

This repository includes a pre-configured Dockerfile to build and run the Devastator platform inside a containerized environment.

---

### 1. Clone the Repository:

```bash
git clone https://github.com/mppinhoo/Devastator.git
cd Devastator

### 2. Build the Docker Image and run the Docker Container:

docker build -t devastator_ws -f Dockerfile .

docker run -it \
  --network=host \
  --ipc=host \
  -v $PWD:/home/devastator/proj_ws \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --env=DISPLAY \
  -v /dev/input:/dev/input \
  --device-cgroup-rule='c 13:* rmw' \
  --device=/dev/ttyUSB0 \
  --device=/dev/ttyACM0 \
  devastator_ws


