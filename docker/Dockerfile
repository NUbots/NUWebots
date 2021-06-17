ARG BASE_IMAGE=ubuntu:focal
FROM ${BASE_IMAGE}
# Can use nivida's version for gpu acceleration
# nvidia/cudagl:11.0-devel-ubuntu20.04

# Disable dpkg/gdebi interactive dialogs
ENV DEBIAN_FRONTEND=noninteractive

# Dependencies, some of theme may not be needed
RUN apt update && apt install --yes git xvfb cmake-curses-gui libprotobuf-dev protobuf-compiler libeigen3-dev libyaml-cpp-dev ninja-build clang-tidy python3-dev xserver-xorg-video-dummy xpra xorg-dev libgl1-mesa-dev mesa-utils libgl1-mesa-glx  

WORKDIR /usr/local
RUN git clone --branch nubots-changes --single-branch --recurse-submodules -j$(nproc) https://github.com/NUbots/webots.git

# Install Webots
WORKDIR /usr/local/webots

RUN chmod +x scripts/install/linux_compilation_dependencies.sh && ./scripts/install/linux_compilation_dependencies.sh
RUN chmod +x scripts/install/linux_optional_compilation_dependencies.sh && ./scripts/install/linux_optional_compilation_dependencies.sh
RUN chmod +x scripts/install/linux_runtime_dependencies.sh && ./scripts/install/linux_runtime_dependencies.sh

ENV JAVA_HOME=/usr/lib/jvm/java-14-openjdk-amd64

RUN make -j$(nproc)

ENV QTWEBENGINE_DISABLE_SANDBOX=1
ENV WEBOTS_HOME /usr/local/webots
ENV PATH /usr/local/webots:${PATH}


# Finally open a bash command to let the user interact
CMD ["/bin/bash"] 
