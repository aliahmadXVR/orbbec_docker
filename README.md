That's a great idea for a README. Based on your setup and commands, here is the complete Markdown file.

-----

# Orbbec Femto Mega ROS Noetic Docker Container for Jetson AGX Orin

This repository contains the necessary files to build and run a **ROS Noetic** containerized environment specifically for the **Orbbec Femto Mega** depth camera on a **Jetson AGX Orin** device.

## 🚀 Overview

This container facilitates using the Orbbec Femto Mega camera on a Jetson AGX Orin (client/slave device) by publishing its ROS topics directly to a remote **ROS Master**.

| Device | Role | IP Address | ROS Installation | Camera |
| :--- | :--- | :--- | :--- | :--- |
| **Jetson Orin NX** | **ROS Master** | `192.168.8.1` | Direct | N/A |
| **Jetson AGX Orin** | **ROS Slave** | `192.168.8.2` | **Inside Docker** | Orbbec Femto Mega |

The container uses **ROS Noetic** and is built for **JetPack 5.1.2** with the necessary configurations to communicate with the remote ROS Master at `http://192.168.8.1:11311`.

## 🛠️ Building the Docker Image

### Prerequisites

Ensure you have **Docker** and **NVIDIA Container Toolkit** (or `nvidia-docker`) installed on your Jetson AGX Orin device.

### Build Command

Assuming your `Dockerfile` is in the current directory, use the following command to build the image:

```bash
sudo docker build -t aliahmadxvr/orbbec-noetic:jetpack5.1.2 .
```

> **Note:** The final dot (`.`) in the command indicates the current directory is the build context. This command is correct.

-----

## 🏃 Running the Container

### Manual Run (Interactive/Debugging)

Use this command to run the container in an interactive mode. The `--rm` flag ensures the container is automatically removed when you stop it.

```bash
sudo docker run -it --rm \
  --name orbbec-container \
  --network=host \
  -e ROS_MASTER_URI=http://192.168.8.1:11311 \
  -e ROS_HOSTNAME=$(hostname -I | awk '{print $1}') \
  -e ROS_IP=$(hostname -I | awk '{print $1}') \
  --runtime nvidia \
  --device /dev/bus/usb:/dev/bus/usb \
  --privileged \
  aliahmadxvr/orbbec-noetic:jetpack5.1.2
```

| Option | Purpose |
| :--- | :--- |
| `--network=host` | Allows the container to use the host's network, essential for **ROS networking**. |
| `-e ROS_MASTER_URI=...` | Sets the address for the remote **ROS Master** on the Orin NX. |
| `-e ROS_HOSTNAME/ROS_IP` | Dynamically sets the container's hostname/IP to the host's primary IP, crucial for the Master to communicate back. |
| `--runtime nvidia` | Enables use of the **NVIDIA GPU** inside the container. |
| `--device /dev/bus/usb:/dev/bus/usb` | Maps the host's USB bus into the container, allowing access to the **Orbbec Femto Mega**. |
| `--privileged` | Grants the container extended privileges, often needed for deep hardware access like USB devices. |

-----

### Run on Boot (Automatic Startup)

To ensure the camera driver and ROS topics are published automatically every time your Jetson AGX Orin boots up, use the following command. The `--restart=always` flag makes Docker manage the automatic startup.

```bash
sudo docker run -d \
  --name orbbec-container \
  --network=host \
  -e ROS_MASTER_URI=http://192.168.8.1:11311 \
  -e ROS_HOSTNAME=$(hostname -I | awk '{print $1}') \
  -e ROS_IP=$(hostname -I | awk '{print $1}') \
  --runtime nvidia \
  --device /dev/bus/usb:/dev/bus/usb \
  --privileged \
  --restart=always \
  aliahmadxvr/orbbec-noetic:jetpack5.1.2
```

> **Note:** The `-d` flag runs the container in **detached** (background) mode.

-----

## 🐛 Debugging and Management

Use these common Docker commands for monitoring and managing the running container:

### View Logs (Real-time)

Check the output of the running ROS node:

```bash
sudo docker logs -f orbbec-container
```

### Stop the Container

Gracefully stop the background service:

```bash
sudo docker stop orbbec-container
```

### Enter the Container

Access a shell inside the running container for troubleshooting:

```bash
sudo docker exec -it orbbec-container bash
```