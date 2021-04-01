# Short guide for generating U-Boot-SPL image which enables FPGA-SDRAM interface for VIP

# Create HPS settings first
${INTELFPGA_PATH}/20.1/embedded/embedded_command_shell.sh bsp-create-settings --type spl --bsp-dir software/bootloader --preloader-settings-dir hps_isw_handoff/sys_ddr3_0_hps --settings software/bootloader/settings.bsp

# Edit settings if needed
cd software/bootloader
${INTELFPGA_PATH}/20.1/embedded/embedded_command_shell.sh bsp-editor

# Copy sources for U-Boot
cd software/bootloader/u-boot-socfpga
./arch/arm/mach-socfpga/qts-filter.sh cyclone5 ../../../ ../ ./board/altera/cyclone5-socdk/qts/

# Configure and build U-Boot
cd software/bootloader/u-boot-socfpga
export CROSS_COMPILE=arm-linux-gnueabihf-
make socfpga_de10_nano_defconfig
make -j 48

# Generate SD card image
cd software/bootloader/sd_image
su-to-root -c "python3 ./make_sdimage_p3.py -f -P ../u-boot-socfpga/u-boot-with-spl.sfp,num=3,format=raw,size=2M,type=A2 -s 4M -n sdcard_cv.img"