#!/bin/bash
set -e
set -x

if [ "$UID" -eq 0 ]; then
     echo "Sorry, this script must NOT be run as ROOT!"
     exit 1
fi

# Check for lsb_release and set distribution variables safely
if command -v lsb_release &> /dev/null; then
    DISTRIBUTION_ID=$(lsb_release -i -s)
    DISTRIBUTION_CODENAME=$(lsb_release -c -s)
else
    DISTRIBUTION_ID=""
    DISTRIBUTION_CODENAME=""
fi

if [ ${DISTRIBUTION_ID} = 'Ubuntu' ]; then
  if [ ${DISTRIBUTION_CODENAME} = 'focal' ] || [ ${DISTRIBUTION_CODENAME} = 'bionic' ]; then
    sudo add-apt-repository universe
  fi
fi

# Check for macOS
if [[ "$(uname)" == "Darwin" ]]; then
    echo "Detected macOS. Installing dependencies using Homebrew."
    # Check for Homebrew and install if missing
    if ! command -v brew &> /dev/null; then
        echo "Homebrew not found. Installing Homebrew."
        /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
        export PATH="/usr/local/bin:/opt/homebrew/bin:$PATH"
    fi
    brew update
    brew install git imagemagick curl wget make python3 unzip
    # Ensure pip is available
    if ! command -v pip3 &> /dev/null; then
        python3 -m ensurepip --upgrade || true
    fi
    SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
    python3 -m pip install --user --upgrade -r "$SCRIPT_DIR"/requirements.txt
    echo "Setup completed successfully for macOS!"
    exit 0
fi

sudo apt-get -y update
sudo apt-get install -y unzip git imagemagick curl wget make python3

# Install packages release specific
if [ ${DISTRIBUTION_CODENAME} = 'bionic' ]; then
  sudo apt-get install -y python3-distutils
fi

# For WSL (esp. version 2) make DISPLAY empty to allow pip to run
STORED_DISPLAY_VALUE=$DISPLAY
if grep -qi -E '(Microsoft|WSL)' /proc/version; then
  echo "Temporarily clearing the DISPLAY variable because this is WSL"
  export DISPLAY=
fi

sudo apt-get install -y python3-pip

# Install required python packages
SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
python3 -m pip install --user --upgrade -r "$SCRIPT_DIR"/requirements.txt

# Reset the value of DISPLAY
if grep -qi -E '(Microsoft|WSL)' /proc/version; then
  echo "Returning DISPLAY to the previous value in WSL"
  export DISPLAY=$STORED_DISPLAY_VALUE
fi

echo "Setup completed successfully!"
