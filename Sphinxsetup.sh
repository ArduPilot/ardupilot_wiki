#!/bin/bash
set -e
set -x

if [ "$UID" -gt 0 ]; then
     echo "Sorry, this script must be run as ROOT!"
     exit 1
fi

DISTRIBUTION_CODENAME=$(lsb_release -i -s)
if [ ${DISTRIBUTION_CODENAME} == 'Ubuntu' ]; then
  add-apt-repository universe
fi
apt-get -y update
apt-get install -y unzip git imagemagick curl wget make python

# Get pip through the official website to get the lastest release
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python get-pip.py && rm -f get-pip.py

# Install sphinx
pip install -U sphinx==1.8.3

# Install sphinx theme from ArduPilot repository
pip install git+https://github.com/ArduPilot/sphinx_rtd_theme.git -UI

# and a youtube plugin:
pip install git+https://github.com/sphinx-contrib/youtube.git -UI

# and a vimeo plugin:
pip install git+https://github.com/ArduPilot/sphinxcontrib.vimeo.git -UI

# Say that we finish
echo "Setup completed successfully !"
