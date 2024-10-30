FROM nvcr.io/nvidia/isaac-sim:4.0.0

# Set working directory to root
WORKDIR /

# Update apt-get and install necessary packages
RUN apt-get update && \
    apt-get install -y git curl git-lfs cmake vim gedit && \
    git lfs install

# install build tools
RUN apt-get install -y build-essential gcc g++ sudo wget unzip software-properties-common

# Clone the required repositories
RUN git clone https://github.com/AuTURBO/StrideSim.git && \
    git clone https://github.com/isaac-sim/IsaacLab.git -b v1.0.0

# pull lfs files in StrideSim
WORKDIR /StrideSim

RUN git lfs fetch && \
    git lfs pull

# Install ROS 2 Humble
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata && \
    apt-get install -y locales software-properties-common && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-humble-desktop

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

# Run conda setup
RUN /bin/bash -c "./isaaclab.sh --conda"

# Initialize conda and activate environment in the same RUN command
RUN /bin/bash -c "source /opt/miniconda/etc/profile.d/conda.sh && \
    conda init bash && \
    conda activate isaaclab"

# Install IsaacLab
RUN /bin/bash -c "./isaaclab.sh --install"

# Set up StrideSim
WORKDIR /StrideSim/rl
RUN /bin/bash -c "source /opt/miniconda/etc/profile.d/conda.sh && \
    conda activate isaaclab && \
    python -m pip install -e ."

WORKDIR /isaac-sim

# Add conda environment activation to bashrc
RUN echo "source /opt/miniconda/etc/profile.d/conda.sh && conda activate isaaclab" >> ~/.bashrc

# Set default shell to bash
SHELL ["/bin/bash", "-c"]
