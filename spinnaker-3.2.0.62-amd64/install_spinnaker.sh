#!/bin/bash

set -o errexit

# Automatically confirm installation
CONFIRM_INSTALL=${CONFIRM_INSTALL:-"Y"}

if [ "$CONFIRM_INSTALL" != "Y" ]; then
    echo "Installation aborted."
    exit 0
fi

echo "This is a script to assist with installation of the Spinnaker SDK."
echo "Installing Spinnaker packages..."

# Install the packages without checking for confirmations
sudo dpkg -i libgentl_*.deb
sudo dpkg -i libspinnaker_*.deb
sudo dpkg -i libspinnaker-dev_*.deb
sudo dpkg -i libspinnaker-c_*.deb
sudo dpkg -i libspinnaker-c-dev_*.deb
sudo dpkg -i libspinvideo_*.deb
sudo dpkg -i libspinvideo-dev_*.deb
sudo dpkg -i libspinvideo-c_*.deb
sudo dpkg -i libspinvideo-c-dev_*.deb
# sudo apt-get install -y qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools ./spinview-qt_*.deb
sudo dpkg -i spinview-qt-dev_*.deb
sudo dpkg -i spinupdate_*.deb
sudo dpkg -i spinupdate-dev_*.deb
sudo dpkg -i spinnaker_*.deb
sudo dpkg -i spinnaker-doc_*.deb

# Automatically allow USB hardware access
echo "Adding udev entry to allow access to USB hardware..."
sudo sh configure_spinnaker.sh

# Automatically set USB-FS memory size to 1000 MB at startup
echo "Setting USB-FS memory size to 1000 MB..."
sudo sh configure_usbfs.sh

# Automatically add Spinnaker prebuilt examples to system path
echo "Adding Spinnaker prebuilt examples to system path..."
sudo sh configure_spinnaker_paths.sh

ARCH=$(ls libspinnaker_* | grep -oP '[0-9]_\K.*(?=.deb)' || [[ $? == 1 ]])
if [ "$ARCH" = "amd64" ]; then
    BITS=64
elif [ "$ARCH" = "i386" ]; then
    BITS=32
fi

if [ -z "$BITS" ]; then
    echo "Could not automatically add the FLIR GenTL Producer to the GenTL environment variable."
    echo "To use the FLIR GenTL Producer, please follow the GenTL Setup notes in the included README."
else
    echo "Adding FLIR GenTL Producer to GENICAM_GENTL${BITS}_PATH..."
    sudo sh configure_gentl_paths.sh $BITS
fi

echo "Installation complete."
echo "Thank you for installing the Spinnaker SDK."
exit 0
