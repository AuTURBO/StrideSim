FROM isaac-sim-ros2:humble-4.0.0

# Set working directory to root
WORKDIR /

# Update apt-get and install necessary packages
RUN apt-get update && \
    apt-get install -y git curl git-lfs cmake && \
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

# Set up IsaacLab
WORKDIR /IsaacLab

# Create symbolic link
RUN ln -s /isaac-sim _isaac_sim

# Install IsaacLab
ENV TERM=xterm-256color
RUN /bin/bash -c "./isaaclab.sh --install"

# Set up StrideSim
WORKDIR /StrideSim/rl

RUN apt-get install -y python3-pip

# Create symbolic link for python3 to python
RUN ln -s /usr/bin/python3 /usr/bin/python

RUN /bin/bash -c "python -m pip install -e ."

WORKDIR /isaac-sim

# Create a non-root admin user
RUN useradd -m -s /bin/bash -G sudo user && \
    echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    chown -R user:user /isaac-sim /StrideSim /IsaacLab

# Switch to user
USER user

# Set default shell to bash
SHELL ["/bin/bash", "-c"]