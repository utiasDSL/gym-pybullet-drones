#!/bin/bash

# USE
# 1. ..

echo 'TBD'

git clone https://github.com/betaflight/betaflight
cd betaflight/
# comment out line `delayMicroseconds_real(50); // max rate 20kHz` in ./src/main/main.c
# edit ports in ./src/main/target/SITL/sitl.c, ./src/main/drivers/serial_tcp.c
make arm_sdk_install 
make TARGET=SITL 
cp ~/gym-pybullet-drones/gym_pybullet_drones/assets/eeprom.bin ~/betaflight/
./obj/main/betaflight_SITL.elf