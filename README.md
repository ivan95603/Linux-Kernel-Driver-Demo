# Linux Kernel Driver for Scale Device
    make                                                    // Build kernel module
    sudo insmod scaleKernelDriver.ko                        // Load kernel module
    sudo rmmod scaleKernelDriver                            // Unload kernel module
    /sys/module/scaleKernelDriver/parameters/scale_value    // Userspace interface for reading data from scale device
    dmesg -xw                                               // While loop read kernel log messages