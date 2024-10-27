# Base image
FROM ros:foxy

# Set the working directory
WORKDIR /root

# Copy files from the host to the container
# COPY <source> <destination>

# Install dependencies
RUN apt-get update && \
    apt-get install -y \ 
    git \
    neovim \
    tmux \
    python3-pip \ 
    ros-foxy-ackermann-msgs

# Startup
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc

# Expose ports
# EXPOSE <port>

# Set environment variables
# ENV <key>=<value>

# Run commands when the container starts
# CMD <command>

# BUILD COMMAND
# docker build -f f1tenth_bt.Dockerfile -t bentjh01:f1tenth-foxy .
