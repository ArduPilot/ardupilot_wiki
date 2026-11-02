#!/bin/bash
set -e
set -x

if [ "$UID" -eq 0 ]; then
    echo "Sorry, this script must NOT be run as ROOT!"
    exit 1
fi

# Detect Arch Linux
if ! grep -qi "arch" /etc/os-release; then
    echo "This script is intended for Arch Linux."
    exit 1
fi

#echo "Updating system..."
#sudo pacman -Syu --noconfirm

echo "Installing base dependencies..."
sudo pacman -S --needed --noconfirm \
    git \
    imagemagick \
    curl \
    wget \
    make \
    python \
    python-pip \
    unzip \
    base-devel

# Install yay (AUR helper) if not present
if ! command -v yay &> /dev/null; then
    echo "Installing yay (AUR helper)..."
    TEMP_DIR=$(mktemp -d)
    git clone https://aur.archlinux.org/yay.git "$TEMP_DIR/yay"
    cd "$TEMP_DIR/yay"
    makepkg -si --noconfirm
    cd ~
    rm -rf "$TEMP_DIR"
fi

# WSL handling (same as original script)
STORED_DISPLAY_VALUE=$DISPLAY
if grep -qi -E '(Microsoft|WSL)' /proc/version; then
    echo "Temporarily clearing the DISPLAY variable because this is WSL"
    export DISPLAY=
fi

# Ensure pip is available
if ! command -v pip &> /dev/null; then
    echo "Ensuring pip is installed..."
    python -m ensurepip --upgrade || true
fi

# Upgrade pip safely (user install)
python -m pip install --user --upgrade pip --break-system-packages

# Install required python packages
SCRIPT_DIR=$(dirname "$(realpath "${BASH_SOURCE[0]}")")

python -m pip install --user --upgrade -r "$SCRIPT_DIR"/requirements.txt --break-system-packages

# Restore DISPLAY if WSL
if grep -qi -E '(Microsoft|WSL)' /proc/version; then
    echo "Returning DISPLAY to the previous value in WSL"
    export DISPLAY=$STORED_DISPLAY_VALUE
fi

echo "Setup completed successfully for Arch Linux!"
