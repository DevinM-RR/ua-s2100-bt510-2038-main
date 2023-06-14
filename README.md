# Regal Beloit BT510 Sensor Device Firmware Repo
<p>
## Cloning Firmware

This is a Zephyr based repository `west` manifest repository with included 'apps'. To clone this repository properly use the `west` tool. To install `west` you will first need to install Python3 (not detailed here).

After installing Python3, install `west` using `pip3`:

### Linux
	pip3 install --user -U west

### macOS (Terminal) and Windows (cmd.exe)
	pip3 install -U west


Once `west` is installed, clone this repository using `west init` and `west update`:


### Checkout the latest manifest on main
	west init -m git@github.com:LairdCP/regal_rexnord_bt510_2038.git

### OR checkout v0.0.7 tag
	west init -m git@github.com:LairdCP/regal_rexnord_bt510_2038.git -mr v0.0.7

### OR checkout temperaturesampling branch
	west init -m git@github.com:LairdCP/regal_rexnord_bt510_2038.git -mr temperaturesampling

Now, pull all the source described in the manifest using:

	west update

## Preparing to Build

If this is your first time working with a Zephyr project on your computer you should follow the [Zephyr getting started guide](https://docs.zephyrproject.org/latest/getting_started/index.html#) to install all the tools.

It is recommended to build this firmware with the [GNU Arm Embedded Toolchain: 8-2019-q3-update](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads).

This project has only been tested building under Windows 10 using cmd.exe shell.

**NOTE:** The current version assumes imgtool.py is installed and in your path in order to sign images for use with mcuboot. To install imgtool.py
```
python3 -m pip install imgtool.py
```

### Building the Firmware

From the directory where you issued the `west init` and `west update` commands you can use the following command to build the firmware:

# Windows
	west build -b bt510 -d build\apps\bt510 apps\bt510 -- -DBOARD_DIR=boards\arm\bt510 -DCONF_FILE=prj.bt510.conf

# Linux and macOS
	west build -b bt510 -d build/apps/bt510 apps/bt510 -- -DBOARD_DIR=boards/arm/bt510 -DCONF_FILE=prj.bt510.conf

### VS Code

This project includes a .vscode settings directory with several configurations that add integration features with the VS Code editor. Use the Terminal->Run Task menu to see the tasks already defined for this project.

For example, here is a sequence of build task steps to build and flash mcuboot, build, sign and flash the application to a bt510 board.

	build app
	flash app

To create a release, run these tasks:

	build app
	create release

The following files will be generated in "Releases\<version\>\bt510:

- bt510-app_update-<version\>.bin - Application DFU image
- bt510-app-merged-<version\>.hex - MCUBoot+App combined image for JTAG programming