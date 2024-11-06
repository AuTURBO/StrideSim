# StrideSim Docker Installation Guide

## Graphic Driver Installation

To install the NVIDIA graphic driver, execute the following commands:

```bash
sudo apt-get update
sudo apt install build-essential -y
wget https://us.download.nvidia.com/XFree86/Linux-x86_64/535.129.03/NVIDIA-Linux-x86_64-535.129.03.run
chmod +x NVIDIA-Linux-x86_64-535.129.03.run
sudo ./NVIDIA-Linux-x86_64-535.129.03.run
```

![alt text](Asset/docker_install/image-2.png)
> I tried to install *nvidia driver-550*, and it also works.

## Docker Installation

To install Docker, execute the following commands:

```bash
# Docker installation using the convenience script
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Post-install steps for Docker
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker

# Verify Docker
docker run hello-world
```

## Nvidia Container Toolkit Installation

```bash
#Configure the repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
  && \
    sudo apt-get update

#Install the NVIDIA Container Toolkit packages
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

#Configure the container runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

#Verify NVIDIA Container Toolkit
docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
```

## Get NGC Key
You can follow this [official documentation](https://docs.nvidia.com/ngc/gpu-cloud/ngc-user-guide/index.html#generating-api-key) to get the NGC key.

1. Go to [NGC](https://ngc.nvidia.com/signin)

2. Click on the **Setup** button on the top right.

    ![alt text](Asset/docker_install/image.png)

3. Click on the **Generate API Key** button.

    ![alt text](Asset/docker_install/image-1.png)

4. Copy the API Key.

```bash
docker login nvcr.io
```

```bash
Username: $oauthtoken
Password: API키
```

## Get Isaac Sim Docker Image

```bash
docker pull nvcr.io/nvidia/isaac-sim:4.0.0
```

## Run StrideSim Docker Container

1. Build Docker Image

```bash
docker build -t stride-sim:v0.0.1 docker
```

2. Run Docker Container

```bash
docker run --name stride-sim-0.0.1 --entrypoint bash -it --runtime=nvidia --gpus all -e "ACCEPT_EULA=Y" --network=host --privileged \
    --user $(id -u):$(id -g) \
    -e "PRIVACY_CONSENT=Y" \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/documents:/root/Documents:rw \
    -v /dev/shm:/dev/shm \
    stride-sim:v0.0.1
```

Now you can run StrideSim Docker Container.

It contains the Isaac Sim, StrideSim, IsaacLab and ROS2 Humble!!

You can run the following command to run isaac-sim.

```bash
cd /isaac-sim
./isaac-sim.sh --allow-root
```

ENJOY!

## File Structure

- /isaac-sim
- /StrideSim
- /IsaacLab
- /opt/ros/humble

## Reference

- [Container Installation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html)
- [NGC Key](https://docs.nvidia.com/ngc/gpu-cloud/ngc-user-guide/index.html#generating-api-key)

*Back to [README](README.md)*