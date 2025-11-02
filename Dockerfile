FROM gitlab-registry.oit.duke.edu/introtorobotics/mems-robotics-toolkit:mems-robotics-latest

# (A) Add any extra system packages you need
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev \
    ros-jazzy-rqt-tf-tree \
    # Add more apt packages here, one per line
 && rm -rf /var/lib/apt/lists/*

# (B) Add any extra Python packages into the course's virtual environment
RUN . /opt/ros_python/bin/activate && \
    pip install --no-cache-dir \
    -r requirements.txt