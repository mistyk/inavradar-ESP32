# INAV - Radar
LoRa based inter UAV communication

INAV-Radar is an addition to the [INAV](https://github.com/iNavFlight/inav) flight control software, it relays information about UAVs in the area to the flight controller for display on the OSD. INAV-Radar does this by using [LoRa](https://en.wikipedia.org/wiki/LoRa) radio to broadcast position, altitude, speed and plane name. It also listens for other UAVs so INAV OSD  can display this information as a radar style map.

## Hardware
Current development is done using these cheap ESP32 LoRa modules.

There are different variants for 433MHz and 868/915MHz:

[Banggood: ESP32 Lora 868/915MHz (2 Pcs)](https://www.banggood.com/de/2Pcs-Wemos-TTGO-LORA32-868915Mhz-ESP32-LoRa-OLED-0_96-Inch-Blue-Display-p-1239769.html?rmmds=search&cur_warehouse=CN)

[Banggood: ESP32 Lora 433MHz](https://www.banggood.com/de/Wemos-TTGO-LORA-SX1278-ESP32-0_96OLED-16-Mt-Bytes-128-Mt-bit-433Mhz-For-Arduino-p-1205930.html?rmmds=search&cur_warehouse=CN)

Other variants without OLED display and different antenna connectors should also work.

For those who want a smaller form factor and the [world record](https://www.youtube.com/watch?v=adhWIo-7gr4) RFM95 radio:

[EXP-Tech: ATmega168 Grove Lora 868 MHz](https://www.exp-tech.de/module/wireless/funk/8022/seeed-studio-grove-lora-radio-868-mhz-rfm95)

[EXP-Tech: ATmega168 Grove Lora 433 MHz](https://www.exp-tech.de/module/wireless/funk/8024/seeed-studio-grove-lora-radio-433-mhz-rfm95)

Also please keep track of your countries regulations regarding radio transmissions.

## Software
Everything here is WORK IN PROGRESS!

The software is based on two components:
- ESP32 LoRa part is found in this repo.
It's developed using [PlatformIO](https://platformio.org/) plugin for [Atom](https://atom.io/) editor.
- INAV OSD part is found [here](https://github.com/mistyk/inav).
It's a fork from the INAV repo and instructions how to build can be found [here](https://github.com/iNavFlight/inav/blob/master/docs/development/Building%20in%20Docker.md).

INAV-Radar is currently no part of the INAV flight control software. INAV can be found [here](https://github.com/iNavFlight/inav).

## Testing
For testing there is no need to install Atom and PlatformIO, just use the [esptool](https://github.com/espressif/esptool) for flashing.

Your system needs the [driver for the USB UART bridge](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers).

Also you will need either [Python 2.7 or Python 3.4 or newer](https://www.python.org/downloads/) installed on your system.

The latest stable esptool.py release can be installed via pip:
```
$ pip install esptool
```

Download the air-to-air test firmware from the [testing folder](https://github.com/mistyk/inavradar-ESP32/testing/)
and extract it. Run this command to flash it onto your ESP32 LoRa module:
```
$ esptool.py --port /dev/tty.SLAB_USBtoUART write_flash -z --flash_mode dio 0x1000 bootloader_dio_40m.bin 0x8000 default.bin 0xe000 boot_app0.bin 0x10000 firmware.bin
```
You may change the --port to match your operating system. If you are using Windows check the [device manager](https://github.com/mistyk/inavradar-ESP32/raw/master/docs/devManager.PNG).

The output should look something like this:
![Windows CMD output](https://github.com/mistyk/inavradar-ESP32/raw/master/docs/cmd.PNG)

## Contact
[Facebook Group](https://www.facebook.com/groups/360607501179901/)
