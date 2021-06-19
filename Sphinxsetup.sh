#!/bin/bash
set -e
set -x

if [ "$UID" -gt 0 ]; then
     echo "Sorry, this script must be run as ROOT!"
     exit 1
fi

DISTRIBUTION_ID=$(lsb_release -i -s)
if [ ${DISTRIBUTION_ID} == 'Ubuntu' ]; then
  DISTRIBUTION_CODENAME=$(lsb_release -c -s)
  if [ ${DISTRIBUTION_CODENAME} == 'focal' ] || [ ${DISTRIBUTION_CODENAME} == 'bionic' ]; then
    add-apt-repository universe
  fi
fi

apt-get -y update
apt-get install -y unzip git imagemagick curl wget make python3

# Install packages release specific
if [ ${DISTRIBUTION_CODENAME} == 'bionic' ]; then
  apt-get install -y python3-distutils
elif [ ${DISTRIBUTION_CODENAME} == 'focal' ]; then
  apt-get install -y python-is-python3
else
    if [ ${DISTRIBUTION_ID} == 'Ubuntu' ]; then
        apt-get install -y python-is-python3
    fi
fi

# Get pip through the official website to get the lastest release
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python3 get-pip.py
rm -f get-pip.py

# Install sphinx
python3 -m pip install --upgrade sphinx==1.8.3

# install a specific version of docutils to avoid __name__ attribute error
python3 -m pip install --upgrade docutils==0.16

# Install sphinx theme from ArduPilot repository
python3 -m pip install git+https://github.com/ArduPilot/sphinx_rtd_theme.git --upgrade

# and a youtube plugin:
python3 -m pip install git+https://github.com/sphinx-contrib/youtube.git --upgrade

# and a vimeo plugin:
python3 -m pip install git+https://github.com/ArduPilot/sphinxcontrib.vimeo.git --upgrade

# Say that we finish
echo "Setup completed successfully !"
