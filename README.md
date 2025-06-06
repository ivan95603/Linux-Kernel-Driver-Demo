# Linux Kernel Driver for Scale Device with IIO interface support
```bash
    make				    # Build kernel module + dtbo
    make dtapply		    # Apply DT overlay
    make load				# Load kernel module
    sudo unload			    # Un load kernel module
    dmesg -xw			    # While loop read kernel log messages
```

```bash
/sys/module/scaleKernel_driver/parameters/scale_value 
# Userspace interface for reading data from scale device
```

## ****** IIO INTERFACE ******
```bash
    cat /sys/bus/iio/devices/iio\:device0/name
    cat /sys/bus/iio/devices/iio\:device0/in_pressure_raw
```
## ****** EOF IIO INTERFACE ******
