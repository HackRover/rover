# Ardiuno Communication

Basic templates and examples on how to comunicated between arduino and ROS. Uses the ROSserial library for arduino. 

This has not been tested yet and assumes that serial communication works through a USB cable between arduino and ROS system.

# Connection Between Arduion and ROS

## ROS On Jetson Nano

    The Jetson Nano has a USB port that can be used to connect to the arduino. The arduino will be powered by the Jetson Nano. This does require setup on the Jeston Nano side.

    Resource Link - https://elinux.org/Jetson/Tutorials/Program_An_Arduino

### FTDI Kernel Module

    Arduino uses an FTDI serial-to-USB converter, which isn't set in the kernel configuration by default. To enable it, run the following commands:

```bash
    ~$ zcat /proc/config.gz | grep FTDI
    tar xvjf kernel_src.tar.bz2
    zcat /proc/config.gz > ~/kernel/.config
    sudo apt-get install ncurses-bin libncurses5-dev
    make menuconfig
```

Navigate to Device Drivers -> USB Support -> USB Serial Converter Support

Choose 'M'odule for USB FTDI Single Port Serial Driver

Save changes and exit.

Verify that the FTDI component is now set to build as module:
```
~/kernel$ cat .config | grep FTDI
```

### Building Modules

```
    make prepare
    make modules_prepare

    make M=drivers/usb/serial/
```

### Installing the FTDI Module

```
    sudo cp drivers/usb/serial/ftdi_sio.ko /lib/modules/`uname -r`/kernel/drivers/usb/serial/
    sudo cp drivers/usb/serial/usbserial.ko /lib/modules/`uname -r`/kernel/drivers/usb/serial/
    sudo depmod -a
```

Verify Installation 
```
    ~/kernal$ dmesg | grep usb
```

Port has been assigned to /dev/ttyUSB0

### Arudino IDE

    The Arduino IDE is not installed by default on the Jetson Nano. To install it, run the following commands:

```
    sudo apt-get update
    sudo apt-get install arduino
```

Under tools > Serial Port is the option to select the port for the arduino to connect to the Jetson Nano with.

# Package Setup

## ROS Workstation

    Doing work with arduinos does require the ROS Workstation, so follow traditional ways to install it if you haven't already.

## Installing ROS Package to Arduino IDE

```
  cd <sketchbook>/libraries
  rm -rf ros_lib
  rosrun rosserial_arduino make_libraries.py .
```

For windows:

```
  cd <some_empty_directory>
  rosrun rosserial_arduino make_libraries.py .
```

If it worked, you should be able to see Rosserial Arduino Library in the Examples section of the Arduino IDE.