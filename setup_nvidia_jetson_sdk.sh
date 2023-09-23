#!/bin/bash
#---SETUP PATHS---
set -e

export DEVICE_FOLDER='Jetson_Linux_R35.2.1_aarch64'
export NVIDIA_WORKSPACE=$HOME/nvidia/nvidia_sdk
export JETPACK=$NVIDIA_WORKSPACE/$DEVICE_FOLDER/Linux_for_Tegra

echo -e "\e[7mThis scripts creates nvidia sdk flashing ecosystem and applies airvolute bsp.\e[0m"
if [[ -z "${DOWNLOAD_PATH}" ]]; then
    echo  "Using default download PATH\n"
    DOWNLOAD_PATH_SCRIPT=${HOME}"/Downloads/airvolute_bsp_image_orinnx"
else
    DOWNLOAD_PATH_SCRIPT=${DOWNLOAD_PATH}"/airvolute_bsp_image_orinnx"
fi

echo -e "\e[7mDownloading the nvidia sdk bsp folder.\e[0m"
if [ -d $DOWNLOAD_PATH_SCRIPT ]; then
rm -rf $DOWNLOAD_PATH_SCRIPT
fi
mkdir -p $DOWNLOAD_PATH_SCRIPT
wget "https://developer.nvidia.com/downloads/jetson-linux-r3521-aarch64tbz2" -P $DOWNLOAD_PATH_SCRIPT
echo -e "\e[7mDownloading the rootfs.\e[0m"

wget "https://developer.nvidia.com/downloads/linux-sample-root-filesystem-r3521aarch64tbz2" -P $DOWNLOAD_PATH_SCRIPT


echo -e "\e[7mCreating flashing environment with bsp.\e[0m"
if [ -d $NVIDIA_WORKSPACE"/"$DEVICE_FOLDER ]; then
    echo "$NVIDIA_WORKSPACE/$DEVICE_FOLDER this workspace already exists, do you want to delete it?"
    read -p "Do you want to proceed? (yes/no) " yn
    case $yn in
        yes ) echo ok, we will proceed;;
        no ) echo exiting...;
            exit;;
        * ) echo invalid response;
            exit 1;;
    esac
    sudo rm -rf $NVIDIA_WORKSPACE"/"$DEVICE_FOLDER
fi
mkdir -p $NVIDIA_WORKSPACE"/"$DEVICE_FOLDER

cd $NVIDIA_WORKSPACE"/"$DEVICE_FOLDER

echo -e "\e[7mUntar nvidia bsp: jetson-linux-r3521-aarch64tbz2.\e[0m"

sudo tar -xf $DOWNLOAD_PATH_SCRIPT/jetson-linux-r3521-aarch64tbz2

cd $JETPACK/rootfs
echo -e "\e[7mUntar rootfs jetson-linux-r3521-aarch64tbz2.\e[0m"

sudo tar -xf $DOWNLOAD_PATH_SCRIPT/linux-sample-root-filesystem-r3521aarch64tbz2

cd $JETPACK 

echo -e "\e[7mApply binaries.\e[0m"
sudo ./apply_binaries.sh
