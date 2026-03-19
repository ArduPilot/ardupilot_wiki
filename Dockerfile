FROM python:3.12-slim-trixie

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=ardupilot
ARG USER_UID=1000
ARG USER_GID=1000
ARG WORKDIRECTORY=/ardupilot_wiki

RUN apt-get update && apt-get install --no-install-recommends -y \
    bash-completion \
    git \
    lsb-release \
    sudo \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

WORKDIR ${WORKDIRECTORY}

RUN groupadd ${USER_NAME} --gid ${USER_GID}\
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME} \
    && chown -R ${USER_NAME}:${USER_NAME} ${WORKDIRECTORY}

USER ${USER_NAME}

RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=bind,source=Sphinxsetup.sh,target=${WORKDIRECTORY}/Sphinxsetup.sh \
    --mount=type=bind,source=requirements.txt,target=${WORKDIRECTORY}/requirements.txt \
    bash -c "./Sphinxsetup.sh" \
    && sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ARG PARAMVERSIONING=0
ENV PARAMVERSIONING=$PARAMVERSIONING
WORKDIR /
RUN if [ "$PARAMVERSIONING" -ne 1 ]; then echo "Not building paramversionning"; \
     else echo 'Building paramversionning' \
    && sudo git clone https://github.com/ArduPilot/ardupilot.git \
    && sudo chown -R ${USER_NAME}:${USER_NAME} /ardupilot \
    && mkdir -p /${WORKDIRECTORY}/new_params_mversion /${WORKDIRECTORY}/old_params_mversion \
    && sudo chown -R ${USER_NAME}:${USER_NAME} /${WORKDIRECTORY}/new_params_mversion /${WORKDIRECTORY}/old_params_mversion \
    && sudo ln -s /${WORKDIRECTORY}/new_params_mversion /new_params_mversion \
    && sudo ln -s /${WORKDIRECTORY}/old_params_mversion /old_params_mversion; fi
WORKDIR ${WORKDIRECTORY}

ENV PATH="/home/${USER_NAME}/.local/bin:${PATH}"
