# Get ros-melodic-desktop-full
FROM osrf/ros:melodic-desktop-full

# Notify the docker initialization
RUN echo "Welcome to ROS melodic on docker linux!!"

# Maintainer
LABEL maintainer="Young Hwan Shin <shinyh99@gmail.com>"

# Do linux update
RUN apt-get update
RUN apt-get install -y curl wget vim sudo unzip whois gedit

# Install zsh
RUN apt-get update
RUN apt-get install -y terminator zsh

# Make zsh defualt shell for docker RUN
SHELL ["/usr/bin/zsh", "-c"]
# Maker zsh default shell for docker image
RUN chsh -s $(which zsh)
CMD ["zsh"]

# Install oh-my-zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/robbyrussell/oh-my-zsh/master/tools/install.sh)"
RUN source $HOME/.zshrc && omz update

# ROS Environment setup
RUN echo "source /opt/ros/melodic/setup.zsh" >> $HOME/.zshrc

ENV ROS_DISTRO melodic
ENV CATKIN_WS /root/catkin_ws
RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}/src

RUN apt-get install net-tools iputils-ping ros-melodic-rosbridge-server -y

# update python3 to higher version
RUN sudo apt install -y python3.8
RUN sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1

RUN sudo apt install -y python3-pip
RUN python3 -m pip install --upgrade pip
# RUN python3 -m pip3 install --upgrade pip

RUN pip3 install catkin_pkg empy pyyaml rospkg numpy opencv-contrib-python matplotlib

# Update all
RUN apt-get update -y
RUN apt-get dist-upgrade -y

RUN echo "source /opt/ros/melodic/setup.zsh" >> $HOME/.zshrc
RUN echo "source ~/catkin_ws/devel/setup.zsh" >> $HOME/.zshrc
RUN echo "alias cw='cd ~/catkin_ws'" >> $HOME/.zshrc
RUN echo "alias cs='cd ~/catkin_ws/src'" >> $HOME/.zshrc
# RUN echo "alias cm='cd ~/catkin_ws && catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so && rospack profile'" >> $HOME/.zshrc
RUN echo "alias cm='cd ~/catkin_ws && catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m && rospack profile && source ~/catkin_ws/devel/setup.zsh'" >> $HOME/.zshrc
# RUN echo "export ROS_HOSTNAME=localhost" >> $HOME/.zshrc
# RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> $HOME/.zshrc




# RUN sudo apt install -y \
#     python-rosdep python-rosinstall \
#     python-rosinstall-generator \
#     python-wstool build-essential \



# RUN rosdep init && rosdep update

# RUN echo "export ROS_HOSTNAME=localhost" >> $HOME/.bashrc
# RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> $HOME/.bashrc

# # RUN echo $SHELL
# # RUN PATH="$PATH:/usr/bin/zsh"
# # RUN zsh -command source $HOME/.zshrc && omz update
# RUN ["zsh", "-command", "source $HOME/.zshrc && omz update"]
# # RUN source $HOME/.zshrc && omz update
# CMD ["zsh"]

# RUN source $HOME/.zshrc
# RUN upgrade_oh_my_zsh
# RUN omz update
# RUN upgrade_oh_my_zsh
# CMD ["upgrade_oh_my_zsh"]

# # ubuntu install 
# FROM ubuntu 
# MAINTAINER 9r00t 
# RUN apt-get -y update && \ apt-get install -y git\ wget 
# # zsh install & change shell 
# RUN apt-get -y install zsh 
# RUN chsh -s /usr/bin/zsh 
# # oh-my-zsh install 
# RUN sh -c "$(wget -O- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" 
# # powerline install 
# RUN apt-get -y install fonts-powerline 
# # hack font install 
# COPY ttf /ttf 
# ENV TERM xterm-256color 
# # bullet-train theme install 
# RUN git clone https://github.com/caiogondim/bullet-train.zsh.git ~/.oh-my-zsh/custom/theme/bullet-train 
# ENV ZSH_THEME bullet-train/bullet-train COPY .zshrc /root/.zshrc 
# CMD ["zsh"]

# Clean up large trash files
# RUN apt-get autoremove --purge --yes \
#     && rm -rf /var/lib/apt/lists/* \
#     && rm -rf /etc/ros/rosdep/sources.list.d/20-default.list