FROM ros:foxy
ARG USERNAME=ws_av
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# Copy code over
WORKDIR /home/ws_av/src/mike_av_stack/
COPY  . .

# [Optional] Set the default user. Omit if you want to keep the default as root.
# USER $USERNAME

RUN sudo apt-get install -y ros-foxy-vision-msgs ros-foxy-derived-object-msgs python3-opencv git
RUN pip install -r requirements.txt

WORKDIR /home/ws_av/src/
# RUN git clone git@github.com:Box-Robotics/ros2_numpy.git
WORKDIR /home/ws_av/
# RUN source /opt/ros/foxy/setup.bash
# RUN rosdep update
# RUN rosdep install --from-paths src -y --ignore-src --rosdistro foxy
# RUN colcon build

CMD ["/bin/bash"]

