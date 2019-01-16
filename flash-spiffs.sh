tools/mkspiffs -c spiffs/ -b 4096 -p 256 -s 0x16F000 testing/fs.bin
esptool.py --port /dev/tty.SLAB_USBtoUART write_flash -z 0x291000 testing/fs.bin
