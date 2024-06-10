# linux_jetson_for_dcs
This repository contains Nvidia linux kernel sources with modifications targeted at DroneCore.Suite 1.2 by Airvolute.
## Setup
1. Prepare jetson workspace on host PC using `setup_nvidia_jetson_sdk.sh` script.
2. Clone this repository into `...nvidia/nvidia_sdk/Jetson_Linux_R35.4.1_aarch64/Linux_for_Tegra`
3. Apply your modifications into the kernel sources.
4. Setup Jetson Linux Toolchain: [Nvidia Developer Documentation - Jetson Linux Toolchain](https://docs.nvidia.com/jetson/archives/r35.4.1/DeveloperGuide/text/AT/JetsonLinuxToolchain.html)
5. Install the Jetson Linux build utilities and other modules. 
```
sudo apt install build-essential bc libncurses-dev libssl-dev flex bison pkg-config
```
6. To build the kernel refer to Nvidia Developer forum or use `./build_deploy_linux.sh` from the `linux_jetson` folder.
7. Flash the device with available configurations with appropriate DCS revision and Jetson device. Or apply appropriate modules/image/dtb into the device.

### Example command to flash orin nx with DroneCore.Suite 2.0 using external nvme
`sudo ./tools/kernel_flash/l4t_initrd_flash.sh --external-device nvme0n1p1   -c tools/kernel_flash/flash_l4t_external_custom.xml -p "-c bootloader/t186ref/cfg/flash_t234_qspi.xml"   --showlogs --network usb0 airvolute-dcs2.0+p3767-0000 internal`

### Additional Peripherals Configuration

To make correct use of DroneCore.Suite 2.0, there needs to be some additional system services set up. All of these services can be found here: https://github.com/airvolute/dcs-deploy/tree/main/resources

We recommend setting up at least `ethernet_switch_control` and `usb_hub_control`, which are responsible for correctly handling the USB hub and Ethernet switch state after reset and power-up sequences.

### Camera Modifications
If you want to use a different camera configuration, you can refer to `hardware/nvidia/platform/t23x/p3768/kernel-dts/cvb` for supported combinations. To apply a different camera configuration, a relevant include must be added in the file `tegra234-p3509-a02.dtsi`. Files ending with `dcs 2.0` are compatible with DroneCore.Suite 2.0. It is also possible to create your own configuration based on the other files.

After the compilation of the device tree, you can reflash the device or update the boot configuration on the device itself.

1. The reflash can be done with the above command.

2. To update the device boot configuration, you can follow these steps:
    1. Locate this file in the build folder: `kernel_out/tegra234-p3767-0000-p3509-a02.dtb` and copy it to the device.
    2. SSH into the device and make a local backup copy of `/boot/dtb/kernel_tegra234-p3767-0000-p3509-a02.dtb`.
    3. Replace the original device tree `/boot/dtb/kernel_tegra234-p3767-0000-p3509-a02.dtb` with your compiled one (`scp compiled.dtb /boot/dtb/kernel_tegra234-p3767-0000-p3509-a02.dtb`).
    4. Restart the device. The new device tree should be applied. If the behavior is undesired, you can restore the backup.
