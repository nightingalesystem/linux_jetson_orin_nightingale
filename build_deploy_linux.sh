#!/bin/bash

#---SETUP PATHS---
set -e

echo -e "\e[7mSetting exports...\e[0m"

export VER='JP51 Custom'

export DEVICE_FOLDER='Jetson_Linux_R35.2.1_aarch64' 
export JETPACK_TOP=$HOME/nvidia/nvidia_sdk/$DEVICE_FOLDER
export JETPACK=$HOME/nvidia/nvidia_sdk/$DEVICE_FOLDER/Linux_for_Tegra
export TEGRA_KERNEL_OUT=$JETPACK/linux_jetson/kernel_out
export CROSS_COMPILE_AARCH64_PATH=$HOME/l4t-gcc
export CROSS_COMPILE_AARCH64=$HOME/l4t-gcc/bin/aarch64-buildroot-linux-gnu-
export CROSS_COMPILE=$HOME/l4t-gcc/bin/aarch64-buildroot-linux-gnu-
export INSTALL_MOD_STRIP=--strip-debug
export SOURCE_FOLDER=$JETPACK/linux_jetson
export TEGRA_EXTERNAL_MODULE=$SOURCE_FOLDER/tegra/kernel-src/nv-kernel-display-driver/NVIDIA-kernel-module-source-TempVersion
export AIRVOLUTE_OVERLAY=$SOURCE_FOLDER/resources/airvolute_overlay 

SCRIPT_DIR="$(dirname $(readlink -f "${0}"))"

echo -e "\e[7mRunning build...\e[0m"

if [ -d "$TEGRA_KERNEL_OUT" ]; then
    echo "$TEGRA_KERNEL_OUT exists."
    ./nvbuild.sh -o $TEGRA_KERNEL_OUT
else 
   mkdir -p $TEGRA_KERNEL_OUT
  ./nvbuild.sh -o $TEGRA_KERNEL_OUT
fi

echo -e  "\e[7mDONE compiling exiting for now\e[0m "

cd $JETPACK
sudo ./apply_binaries.sh 

echo -e  "\e[7mCopying dtb files and kernel image from kernel output to overlay directory  ...\e[0m "
cd
cd $AIRVOLUTE_OVERLAY/Linux_for_Tegra/kernel
if [ -f "Image" ]; then
    rm Image
fi

cd
sudo cp $TEGRA_KERNEL_OUT/arch/arm64/boot/Image $AIRVOLUTE_OVERLAY/Linux_for_Tegra/kernel
sudo cp -a $TEGRA_KERNEL_OUT/arch/arm64/boot/dts/nvidia/. $AIRVOLUTE_OVERLAY/Linux_for_Tegra/kernel/dtb/

cd $JETPACK

if [ -d "${JETPACK}/rootfs/usr/lib/modules/5.10.104-tegra/extra" ]; then
  sudo  rm -rf ${JETPACK}/rootfs/usr/lib/modules/5.10.104-tegra/extra
fi

if [ -d "images" ]; then
    rm -r images
else
    mkdir images
fi

cd $SOURCE_FOLDER/kernel/kernel-5.10

echo -e  "\e[7mInstalling modules ...\e[0m "
echo ""

make ARCH=arm64 O=$TEGRA_KERNEL_OUT modules_install CROSS_COMPILE=$CROSS_COMPILE INSTALL_MOD_PATH=$JETPACK/images -j"${NPROC}"

echo -e  "\e[7mBuild external modules\e[0m "
source_dir="${SCRIPT_DIR}/kernel/kernel-5.10/" 

cd  ${source_dir}

cd $TEGRA_EXTERNAL_MODULE
#make clean
make \
      modules \
      SYSSRC=${source_dir} \
      SYSOUT=${TEGRA_KERNEL_OUT} \
      O=${TEGRA_KERNEL_OUT} \
      CC=${CROSS_COMPILE_AARCH64}gcc \
      LD=${CROSS_COMPILE_AARCH64}ld.bfd \
      AR=${CROSS_COMPILE_AARCH64}ar \
      CXX=${CROSS_COMPILE_AARCH64}g++ \
      OBJCOPY=${CROSS_COMPILE_AARCH64}objcopy \
      TARGET_ARCH=aarch64 \
      LOCALVERSION="-tegra" \
      ARCH=arm64 \
      -j"${NPROC}"
      
echo -e  "\e[7mInstalling external modules ...\e[0m "

make ARCH=arm64 O=$TEGRA_KERNEL_OUT modules_install CROSS_COMPILE=$CROSS_COMPILE INSTALL_MOD_PATH=$JETPACK/images M=$PWD -j"${NPROC}"

cd $JETPACK/images

tar --owner root --group root -cjf kernel_supplements.tbz2 lib/modules
mv kernel_supplements.tbz2 $AIRVOLUTE_OVERLAY/Linux_for_Tegra/kernel/

echo -e  "\e[7mCreating airvolute overlay ...\e[0m "

cd  $AIRVOLUTE_OVERLAY/Linux_for_Tegra

if [ -f "av_revision.txt" ]; then
    rm "av_revision.txt"
fi

touch av_revision.txt
echo $(LANG=en_us_88591; date) >> av_revision.txt
echo $VER >> av_revision.txt

cd  $AIRVOLUTE_OVERLAY

tar -cvjSf airvolute_overlay.tbz2 Linux_for_Tegra/

cd

echo -e  "\e[7mApply airvolute overlay ...\e[0m "

cd $JETPACK_TOP
sudo tar -xf $SOURCE_FOLDER/resources/airvolute_overlay/airvolute_overlay.tbz2 

echo -e  "\e[7mAplying binaries ...\e[0m "
echo ""
cd $JETPACK

if [ -f "kernel/kernel_display_supplements.tbz2" ]; then
   sudo rm -r "kernel/kernel_display_supplements.tbz2"
fi

sudo ./apply_binaries.sh -t False
echo -e  "\e[7mDONE\e[0m "

exit 0
done
