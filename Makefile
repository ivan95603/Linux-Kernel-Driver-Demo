# Kernel module name
obj-m += scaleKernel_driver.o

# Kernel source directory (adjust as needed)
KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
# DRIVERS_DIR ?= /lib/modules/$(shell uname -r)/build/kernel/drivers

# Device tree overlay
DTS_FILE = scale_overlay.dts
DTB_FILE = $(DTS_FILE:.dts=.dtbo)

# Default target
all: module dtbo

# Build kernel module
module:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules

# Build device tree overlay
dtbo: $(DTB_FILE)

$(DTB_FILE): $(DTS_FILE)
	dtc -@ -I dts -O dtb -o $@ $<

# Clean everything
clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
	rm -f $(DTB_FILE)

# Install module and overlay
install: all
	sudo cp scaleKernel_driver.ko $(DRIVERS_DIR)/
	sudo depmod -a
	sudo cp $(DTB_FILE) /boot/overlays/
	@echo "Add 'dtoverlay=scale_overlay' to /boot/config.txt to enable"

# UnInstall everything
uninstall:
	rm -f /boot/overlays/$(DTB_FILE)
	rm -f $(DRIVERS_DIR)/scaleKernel_driver.ko

# Load dtbo
dtinsert:
	sudo dtoverlay scale_overlay.dtbo

# Load module
load:
	sudo modprobe scaleKernel_driver

# Unload module
unload:
	sudo modprobe -r scaleKernel_driver

# Display module info
info:
	modinfo scaleKernel_driver.ko

.PHONY: all module dtbo clean install uninstall dtinsert load unload info


# obj-m += scaleKernelDriver.o
 

# KDIR = /lib/modules/$(shell uname -r)/build

# all: driver dt
# 	echo Building driver and DTBO

# dt: scale_overlay.dts
# 	dtc -@ -I dts -O dtb -o scale_overlay.dtbo scale_overlay.dts

# driver:
# 	make -C $(KDIR)  M=$(shell pwd) modules

# clean:
# 	make -C $(KDIR)  M=$(shell pwd) clean
# 	rm -rf scale_overlay.dtbo