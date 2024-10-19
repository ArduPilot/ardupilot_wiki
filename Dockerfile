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
    software-properties-common \
    lsb-release \
    sudo

ENV USER=${USER_NAME}

RUN echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME} \
    && chown -R ${USER_NAME}:${USER_NAME} ${WORKDIRECTORY}

USER ${USER_NAME}

RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=bind,source=Sphinxsetup.sh,target=${WORKDIRECTORY}/Sphinxsetup.sh \
    --mount=type=bind,source=requirements.txt,target=${WORKDIRECTORY}/requirements.txt \
    bash -c "./Sphinxsetup.sh"

RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ENV PATH="/home/${USER_NAME}/.local/bin:${PATH}"
