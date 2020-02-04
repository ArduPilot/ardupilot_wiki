FROM python:3.7-slim-buster

# Say that we won't anwser question
ENV DEBIAN_FRONTEND noninteractive

# Make a working directory
WORKDIR /ardupilot_wiki

RUN apt-get update && apt-get install --no-install-recommends -y \
        software-properties-common \
        lsb-release

COPY Sphinxsetup.sh /ardupilot_wiki/Sphinxsetup.sh
RUN bash -c "/ardupilot_wiki/Sphinxsetup.sh" && rm Sphinxsetup.sh

RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
