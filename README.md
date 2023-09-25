# linux_jetson_for_dcs
This repository contains Nvidia linux kernel sources with modifications targeted at DroneCore.Suite by Airvolute.
## Setup
1. Prepare jetson workspace on host PC using `setup_nvidia_jetson_sdk.sh` script.
2. Clone this repository into `...nvidia/nvidia_sdk/Jetson_Linux_R35.2.1_aarch64/Linux_for_Tegra`
3. Apply your modifications into the kernel sources.
4. Setup Jetson Linux Toolchain: [Nvidia Developer Documentation - Jetson Linux Toolchain](https://docs.nvidia.com/jetson/archives/r35.2.1/DeveloperGuide/text/AT/JetsonLinuxToolchain.html)
5. Install the Jetson Linux build utilities and other modules. 
```
sudo apt install build-essential bc libncurses-dev libssl-dev flex bison pkg-config
```
6. To build the kernel refer to Nvidia Developer forum or use `./build_deploy_linux.sh` from the `linux_jetson` folder.
7. Flash the device with available configurations with appropriate DCS revision and Jetson device. Or apply appropriate modules/image/dtb into the device.

### Example command to flash orin nx with DroneCore.Suite 1.2 using external nvme
`sudo ./tools/kernel_flash/l4t_initrd_flash.sh --external-device nvme0n1p1   -c tools/kernel_flash/flash_l4t_external_custom.xml -p "-c bootloader/t186ref/cfg/flash_t234_qspi.xml"   --showlogs --network usb0 airvolute-dcs1.2+p3767-0000 internal`
