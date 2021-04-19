# On windows side
## Docker setting
- install docker

## GUI Forwarding
- install vcxsrv on windows https://sourceforge.net/projects/vcxsrv/
    - check: clipboard, primary selection, disable access control
    - uncheck: native opengl

# Build docker image
- move to dockerfile location
- `docker build -t <image_name>[:<tag>] .`
- `docker build -t ros-melodic .`

# Run docker image
- `docker run -it --name <container_name> -p 9090:9090 -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 <image_name>[:<tag>]`
    - --net=host does not work in windows docker
    - use -p 9090:9090 instead, 9090 is port for morai network
- `docker run -it --name ros-test -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 ros-melodic`
- `docker run -d -it --name ros-test -p 9090:9090 -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 ros-melodic`

# Run new terminal
- `docker exec -it <container_name> <command>`
- `docker exec -it ros-test zsh`

# Continue
- `docker start -it ros-test zsh`
- `docker attach -it ros-test zsh`

# git clone
- `cd ~/catkin_ws/src`
- `git clone https://github.com/shinyh99/gwu_robot.git .`
- `git submodule update --init --recursive`
- `cd geometry2 && git checkout melodic-devel`