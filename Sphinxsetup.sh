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
  if [ ${DISTRIBUTION_CODENAME} = 'jammy' ] || [ ${DISTRIBUTION_CODENAME} = 'noble' ] || [ ${DISTRIBUTION_CODENAME} = 'resolute' ]; then
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

sudo apt-get --yes update
# Migrate to Python virtual environments on Ubuntu 24.04 and later.
if [ ${DISTRIBUTION_CODENAME} = 'noble' ] || [ ${DISTRIBUTION_CODENAME} = 'resolute' ]; then
  sudo apt-get install --yes curl git imagemagick make unzip wget python3-venv
else
  sudo apt-get install --yes curl git imagemagick make unzip wget python3-pip
fi

# For WSL (esp. version 2) make DISPLAY empty to allow pip to run
STORED_DISPLAY_VALUE=$DISPLAY
if grep -qi -E '(Microsoft|WSL)' /proc/version; then
  echo "Temporarily clearing the DISPLAY variable because this is WSL"
  export DISPLAY=
fi

# Python package requirements are in the same directory as this script.
SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))

# Activate the Python venv with the latest pip and setuptools on Ubuntu 24.04 and later.
if [ ${DISTRIBUTION_CODENAME} = 'noble' ] || [ ${DISTRIBUTION_CODENAME} = 'resolute' ]; then
  python3 -m venv --upgrade-deps .venv
  # shellcheck source=/dev/null
  source .venv/bin/activate
  python3 -m pip install --upgrade -r "$SCRIPT_DIR"/requirements.txt
else
  python3 -m pip install --upgrade --user -r "$SCRIPT_DIR"/requirements.txt
fi

# Reset the value of DISPLAY
if grep -qi -E '(Microsoft|WSL)' /proc/version; then
  echo "Returning DISPLAY to the previous value in WSL"
  export DISPLAY=$STORED_DISPLAY_VALUE
fi

echo "Setup completed successfully!"
