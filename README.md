# linux_jetson_for_dcs
This repository contains Nvidia linux kernel sources with modifications targeted at DroneCore.Suite by Airvolute.
## Setup
1. Prepare jetson workspace on host PC using `setup_nvidia_jetson_sdk.sh` script.
2. Clone this repository into `...nvidia/nvidia_sdk/Jetson_Linux_R35.2.1_aarch64/Linux_for_Tegra`
3. Apply your modifications into the kernel sources.
4. To build the kernel refer to Nvidia Developer forum or use `./build_deploy_linux.sh` from the `linux_jetson` folder.
5. Flash the device with available configurations with appropriate DCS revision and Jetson device. Or apply appropriate modules/image/dtb into the device.


