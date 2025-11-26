FROM gitlab-registry.oit.duke.edu/introtorobotics/mems-robotics-toolkit:kinova-jazzy-latest

# (A) Add any extra system packages you need
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev \
    ros-jazzy-rqt-tf-tree \
    python3-venv \
    # Add more apt packages here, one per line
 && rm -rf /var/lib/apt/lists/*

 COPY requirements.txt .

RUN python3 -m venv /opt/ros_python


# (B) Add any extra Python packages into the course's virtual environment
RUN . /opt/ros_python/bin/activate && \
    pip install --no-cache-dir \
    -r requirements.txt