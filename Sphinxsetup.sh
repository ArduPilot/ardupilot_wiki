#!/bin/bash
set -e
set -x

if [ "$UID" -eq 0 ]; then
     echo "Sorry, this script must NOT be run as ROOT!"
     exit 1
fi

DISTRIBUTION_ID=$(lsb_release -i -s)
if [ ${DISTRIBUTION_ID} = 'Ubuntu' ]; then
  DISTRIBUTION_CODENAME=$(lsb_release -c -s)
  if [ ${DISTRIBUTION_CODENAME} = 'focal' ] || [ ${DISTRIBUTION_CODENAME} = 'bionic' ]; then
    sudo add-apt-repository universe
  fi
fi

sudo apt-get -y update
sudo apt-get install -y unzip git imagemagick curl wget make python3

# Install packages release specific
if [ ${DISTRIBUTION_CODENAME} = 'bionic' ]; then
  sudo apt-get install -y python3-distutils
elif [ ${DISTRIBUTION_CODENAME} = 'focal' ]; then
  sudo apt-get install -y python-is-python3
else
    if [ ${DISTRIBUTION_ID} = 'Ubuntu' ]; then
        sudo apt-get install -y python-is-python3
    fi
fi

# For WSL (esp. version 2) make DISPLAY empty to allow pip to run
STORED_DISPLAY_VALUE=$DISPLAY
if grep -qi -E '(Microsoft|WSL)' /proc/version; then
  echo "Temporarily clearing the DISPLAY variable because this is WSL"
  export DISPLAY=
fi

# Get pip through the official website to get the latest release
GET_PIP_URL="https://bootstrap.pypa.io/get-pip.py"

# accomodate default Python on bionic:
if [ "$(python --version)" == "Python 3.6.9" ]; then
    GET_PIP_URL="https://bootstrap.pypa.io/pip/3.6/get-pip.py"
fi

curl "$GET_PIP_URL" -o get-pip.py
python3 get-pip.py
rm -f get-pip.py

# Install python packages using known working versions
# Install sphinx with a specific docutils version
# Docutils version is for correct bullet point rendering. Can be rolled forward after theme is updated to >=0.5.1
# See https://stackoverflow.com/a/68685753/2578171
python3 -m pip install --user --upgrade sphinx==5.1.1 docutils==0.16

# lxml for parameter parsing:
python3 -m pip install --user --upgrade lxml

# Install sphinx theme from ArduPilot repository
python3 -m pip install --user --upgrade git+https://github.com/ArduPilot/sphinx_rtd_theme.git

# and youtube and video plugins:
# This command might require a --force option if you have and older extension installed
# Rerun Sphinxsetup.sh after doing that
python3 -m pip install --user --upgrade git+https://github.com/ArduPilot/sphinxcontrib-youtube.git

# and a parser to use getting posts from Discourse (forum) and insert in FrontEnd
python3 -m pip install --user --upgrade beautifulsoup4

# Install flake8
python3 -m pip install --user --upgrade flake8==3.7.9

# Reset the value of DISPLAY
if grep -qi -E '(Microsoft|WSL)' /proc/version; then
  echo "Returning DISPLAY to the previous value in WSL"
  export DISPLAY=$STORED_DISPLAY_VALUE
fi

echo "Setup completed successfully!"
