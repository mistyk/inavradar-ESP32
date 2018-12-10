# INAV - Radar
LoRa based inter UAV communication

INAV-Radar is an addition to the [INAV](https://github.com/iNavFlight/inav) flight control software, it relays information about UAVs in the area to the flight controller for display on the OSD. INAV-Radar does this by using [LoRa](https://en.wikipedia.org/wiki/LoRa) radio to broadcast position, altitude, speed and plane name. It also listens for other UAVs so INAV OSD  can display this information as a radar style map.

## Hardware
Current development is done using these cheap ESP32 LoRa modules.
There are different variants for 433MHz and 868/915MHz:
[Banggood: ESP32 Lora 868/915MHz (2 Pcs)](https://www.banggood.com/de/2Pcs-Wemos-TTGO-LORA32-868915Mhz-ESP32-LoRa-OLED-0_96-Inch-Blue-Display-p-1239769.html?rmmds=search&cur_warehouse=CN)
[Banggood: ESP32 Lora 433MHz](https://www.banggood.com/de/Wemos-TTGO-LORA-SX1278-ESP32-0_96OLED-16-Mt-Bytes-128-Mt-bit-433Mhz-For-Arduino-p-1205930.html?rmmds=search&cur_warehouse=CN)
Other variants without OLED display and different antenna connectors should also work.
Also please keep track of your countries regulations regarding radio transmissions.

## Software
Everything here is WORK IN PROGRESS!

The software is based on two components:
- ESP32 LoRa part is found in this repo.
It's developed using [PlatformIO](https://platformio.org/) plugin for [Atom](https://atom.io/) editor.
- INAV OSD part is found [here](https://github.com/mistyk/inav).
It's a fork from the INAV repo and instructions how to build can be fount [here](https://github.com/iNavFlight/inav/blob/master/docs/development/Building%20in%20Docker.md)

INAV-Radar is currently no part of the INAV flight control software. INAV can be found [here](https://github.com/iNavFlight/inav).

## Contact
[Facebook Group](https://www.facebook.com/groups/360607501179901/)
