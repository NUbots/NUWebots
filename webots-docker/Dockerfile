ARG BASE_IMAGE=nvidia/cuda:11.8.0-base-ubuntu22.04
FROM ${BASE_IMAGE} AS downloader

# Determine Webots version to be used and set default argument
ARG WEBOTS_VERSION=R2022b
ARG WEBOTS_PACKAGE_PREFIX=

# Disable dpkg/gdebi interactive dialogs
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install --yes \
  wget\
  bzip2

RUN rm -rf /var/lib/apt/lists/
RUN wget https://github.com/cyberbotics/webots/releases/download/$WEBOTS_VERSION/webots-$WEBOTS_VERSION-x86-64$WEBOTS_PACKAGE_PREFIX.tar.bz2
RUN tar xjf webots-*.tar.bz2 
RUN rm webots-*.tar.bz2

FROM ${BASE_IMAGE}

# Disable dpkg/gdebi interactive dialogs
ENV DEBIAN_FRONTEND=noninteractive


# Install Webots runtime dependencies
RUN apt-get update
RUN apt-get install --yes \
  wget \
  xvfb \
  git \
  python3-pip 
  
RUN rm -rf /var/lib/apt/lists/
RUN wget https://raw.githubusercontent.com/cyberbotics/webots/master/scripts/install/linux_runtime_dependencies.sh 
RUN chmod +x linux_runtime_dependencies.sh
RUN ./linux_runtime_dependencies.sh
RUN rm ./linux_runtime_dependencies.sh

# Install NUWebots dependencies
RUN wget https://raw.githubusercontent.com/NUbots/NUWebots/williamson/webots-docker/webots-docker/dependencies.sh
RUN chmod +x dependencies.sh
RUN ./dependencies.sh 
RUN rm ./dependencies.sh
RUN rm -rf /var/lib/apt/lists/

# Install Webots
WORKDIR /usr/local
COPY --from=downloader /webots /usr/local/webots/
ENV QTWEBENGINE_DISABLE_SANDBOX=1
ENV WEBOTS_HOME /usr/local/webots
ENV PATH /usr/local/webots:${PATH}

# Enable OpenGL capabilities
ENV NVIDIA_DRIVER_CAPABILITIES=graphics

# RUN useradd -ms /bin/bash webots && usermod -aG sudo webots && echo "webots ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers
# ENV HOME=/home/webots
# USER webots

# WORKDIR $HOME

# Build the latest version of the RoboCup Humanoid TC fork of GameController
RUN git clone https://github.com/RoboCup-Humanoid-TC/GameController ./GameController 
RUN cd GameController && ant

# Build the robocup controllers
RUN git clone https://github.com/RoboCup-Humanoid-TC/hlvs_webots.git ./hlvs_webots
RUN pip3 install -r ./hlvs_webots/controllers/referee/requirements.txt
RUN make -j$(nproc) -C ./hlvs_webots

# Set environment variables for GameController
ENV GAME_CONTROLLER_HOME=/GameController JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64

# Install the NUbots-developed environment
RUN git clone https://github.com/NUbots/NUWebots /NUWebots
RUN pip3 install -r /NUWebots/requirements.txt
RUN /NUWebots/b configure -- -DENABLE_CLANG_TIDY=OFF
RUN /NUWebots/b build

RUN wget https://raw.githubusercontent.com/NUbots/NUWebots/williamson/webots-docker/webots-docker/entrypoint.sh
RUN wget https://raw.githubusercontent.com/NUbots/NUWebots/williamson/webots-docker/webots-docker/webots-config.py
RUN chmod +x entrypoint.sh

# Configure robocup game.json file based on environment variables
ENTRYPOINT ["./entrypoint.sh"]
CMD ["bash"]
