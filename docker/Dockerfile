FROM nvcr.io/nvidia/isaac-sim:4.0.0

# Set working directory to root
WORKDIR /

# Update apt-get and install necessary packages
RUN apt-get update && \
    apt-get install -y git curl git-lfs cmake vim gedit && \
    git lfs install

# Clone the required repositories
RUN git clone https://github.com/AuTURBO/StrideSim.git && \
    git clone https://github.com/isaac-sim/IsaacLab.git -b v1.0.0

WORKDIR /StrideSim

# update git lfs
RUN git lfs fetch && \
    git lfs pull

# Install ROS 2 Humble
# RUN apt-get update && apt-get install -y locales software-properties-common && \
#     locale-gen en_US en_US.UTF-8 && \
#     update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
#     add-apt-repository universe && \
#     curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
#     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list && \
#     apt-get update && \
#     apt-get install -y ros-humble-desktop

# Install Miniconda (latest version)
WORKDIR /

RUN curl -sS https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -o miniconda.sh && \
    bash miniconda.sh -b -p /opt/miniconda && \
    rm miniconda.sh && \
    /opt/miniconda/bin/conda init

# Set environment variables for conda
ENV PATH="/opt/miniconda/bin:$PATH"

# Set up IsaacLab
WORKDIR /IsaacLab

# Create symbolic link
RUN ln -s /isaac-sim _isaac_sim

# Initialize conda
RUN /bin/bash -c "source /opt/miniconda/etc/profile.d/conda.sh"

# Run conda setup
RUN /bin/bash -c "./isaaclab.sh --conda"

# Install IsaacLab
RUN /bin/bash -c "./isaaclab.sh --install"

# Set up StrideSim
WORKDIR /StrideSim/rl
RUN python -m pip install -e .
