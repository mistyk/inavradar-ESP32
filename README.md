![Logo](https://github.com/mistyk/inavradar-ESP32/raw/master/docs/logo.png)

# LoRa based inter UAV communication

INAV-Radar is an addition to the [INAV](https://github.com/iNavFlight/inav) flight control software, it relays information about UAVs in the area to the flight controller for display on the OSD. INAV-Radar does this by using [LoRa](https://en.wikipedia.org/wiki/LoRa) radio to broadcast position, altitude, speed and plane name. It also listens for other UAVs so INAV OSD  can display this information as a radar style map.

## Hardware
Current development is done using these cheap ESP32 LoRa modules.

There are different variants for 433MHz and 868/915MHz:

[Banggood: ESP32 Lora 868/915MHz (2 Pcs)](https://www.banggood.com/de/2Pcs-Wemos-TTGO-LORA32-868915Mhz-ESP32-LoRa-OLED-0_96-Inch-Blue-Display-p-1239769.html?rmmds=search&cur_warehouse=CN)

[Banggood: ESP32 Lora 433MHz](https://www.banggood.com/de/Wemos-TTGO-LORA-SX1278-ESP32-0_96OLED-16-Mt-Bytes-128-Mt-bit-433Mhz-For-Arduino-p-1205930.html?rmmds=search&cur_warehouse=CN)

Other variants (e.g. Heltec) or without OLED display and different antenna connectors should also work.

Also please keep track of your countries regulations regarding radio transmissions.

## Software
Everything here is WORK IN PROGRESS!

The software is based on two components:
- ESP32 LoRa part is found in this repo.
It's developed using [PlatformIO](https://platformio.org/) plugin for [Atom](https://atom.io/) editor.
- INAV OSD part is found [here](https://github.com/mistyk/inav).
It's a fork from the INAV repo and instructions how to build can be found [here](https://github.com/iNavFlight/inav/blob/master/docs/development/Building%20in%20Docker.md).

INAV-Radar is currently no part of the INAV flight control software. INAV can be found [here](https://github.com/iNavFlight/inav).

## Testing communication between ESP32 Lora modules and FCs
For testing there is no need to install Atom and PlatformIO, just use the [esptool](https://github.com/espressif/esptool) for flashing.

Your system needs the driver for the USB UART bridge:
[Windows+MacOS](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers)
 or [Alternative MacOS](https://github.com/adrianmihalko/ch340g-ch34g-ch34x-mac-os-x-driver)

Also you will need [Python 3.4 or newer](https://www.python.org/downloads/) installed on your system.

Be sure to check 'Add Python to PATH':
![Python Setup](https://github.com/mistyk/inavradar-ESP32/raw/master/docs/python.png)


The latest stable esptool.py release can be installed via pip in your command prompt:

Windows:
```
c:\> pip install esptool
```

MacOS:
```
$ pip3 install esptool
```

Download the air-to-air test firmware from the [testing folder](https://github.com/mistyk/inavradar-ESP32/tree/master/testing)
and extract it. Run this command to flash it onto your ESP32 Lora module (Windows and MacOS):

You may change the --port to match your operating system. If you are using Windows check the [device manager](https://github.com/mistyk/inavradar-ESP32/raw/master/docs/devManager.PNG).

Windows:
```
c:\> cd (your air-to-air directory here)
c:\> esptool.py --port COM11 write_flash -z --flash_mode dio 0x1000 bootloader_dio_40m.bin 0x8000 default.bin 0xe000 boot_app0.bin 0x10000 firmware.bin 0x291000 fs.bin
```

MacOS:
```
$ cd (your air-to-air directory here)
$ esptool.py --port /dev/tty.SLAB_USBtoUART write_flash -z --flash_mode dio 0x1000 bootloader_dio_40m.bin 0x8000 default.bin 0xe000 boot_app0.bin 0x10000 firmware.bin 0x291000 fs.bin

```

The output should look something like this:
![Windows CMD output](https://github.com/mistyk/inavradar-ESP32/raw/master/docs/cmd.PNG)

After the firmware is flashed on the devices you should see 'No FC' (or the name of the UAV) on the opposing display.
If only the bottom line with TX and RX is showing, something is not working correctly, please open a serial terminal (Windows e.g. [PuTTy](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html)). Reset the device connected to the terminal and send us a copy of the output.

There is also a Android app in the [testing folder](https://github.com/mistyk/inavradar-ESP32/tree/master/testing) to control debugging modes like sending fake planes to FC or via radio. If your ESP is powered up you can pair it to your phone via Bluetooth. Also note that Bluetooth is disabled on arming.

Via USB serial you get a small CLI with debug output.

Also there are some commands:

```
config                           shows current config
config loraFreq 433000000        sets lora frequency
debug                            toggle debug output
radiofakeplanes                  makes the module send a fakeplane (somewhere in germany)
localfakeplanes                  makes the module send a fakeplanes to connected FC
movefakeplanes                   moves the fakeplane
```

Attention !!! On Windows with Putty you must use [CRTL]-[J] insted of the [ENTER] key. 

A [testing version](https://github.com/mistyk/inavradar-ESP32/tree/master/testing) of INAV needs to be installed on your FC.
To connect the ESP32 to the FC:
- wire up +5V and GND
- TX from FC to ESP RX pin 17
- RX from FC to ESP TX pin 23

Thanks for testing! 😄 👍

## Contact
[Facebook Group](https://www.facebook.com/groups/360607501179901/)

[Patreon](https://www.patreon.com/inavradar)
