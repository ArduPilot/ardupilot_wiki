FROM python:3.11-slim-bookworm

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=ardupilot_wiki
ARG USER_UID=1000
ARG USER_GID=1000
ARG WORKDIRECTORY=/${USER_NAME}

WORKDIR ${WORKDIRECTORY}
RUN groupadd ${USER_NAME} --gid ${USER_GID}\
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash


RUN apt-get update && apt-get install --no-install-recommends -y \
    bash-completion \
    git \
    software-properties-common \
    lsb-release \
    sudo \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ENV USER=${USER_NAME}

RUN echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME} \
    && chown -R ${USER_NAME}:${USER_NAME} ${WORKDIRECTORY}

USER ${USER_NAME}

COPY Sphinxsetup.sh ${WORKDIRECTORY}/Sphinxsetup.sh
RUN bash -c "${WORKDIRECTORY}/Sphinxsetup.sh" && rm Sphinxsetup.sh
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ARG PARAMVERSIONING=0
ENV PARAMVERSIONING=$PARAMVERSIONING
WORKDIR /
RUN if [ "$PARAMVERSIONING" -ne 1 ]; then echo "Not building paramversionning"; \
     else echo 'Building paramversionning' \
    && sudo git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git \
    && sudo mkdir -p new_params_mversion && sudo mkdir -p old_params_mversion \
    && sudo chown -R ${USER_NAME}:${USER_NAME} /ardupilot \
    && sudo chown -R ${USER_NAME}:${USER_NAME} /new_params_mversion \
    && sudo chown -R ${USER_NAME}:${USER_NAME} /old_params_mversion; fi

WORKDIR ${WORKDIRECTORY}

ENV PATH="/home/${USER_NAME}/.local/bin:${PATH}"
