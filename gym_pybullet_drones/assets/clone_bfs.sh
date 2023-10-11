#!/bin/bash

# Clone, edit, build, configure, multiple Betaflight SITL executables

# Check for the correct number of command-line arguments
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <gym_pybullet_drones full path> <num_iterations>"
    exit 1
fi

# Extract command-line arguments
gpd_base_path="$1"
num_iterations="$2"

# Create directory along gym-pybullet-donres
cd $gpd_base_path
cd ..
mkdir betaflights/
cd betaflights/

pattern0="delayMicroseconds_real(50); // max rate 20kHz"
pattern1="#define PORT_PWM_RAW    9001    // Out"
pattern2="#define PORT_PWM        9002    // Out"
pattern3="#define PORT_STATE      9003    // In"
pattern4="#define PORT_RC         9004    // In"
pattern5="#define BASE_PORT 5760"

replacement0="// delayMicroseconds_real(50); // max rate 20kHz"

for ((i = 1; i <= num_iterations; i++)); do

    # Clone
    git clone https://github.com/betaflight/betaflight "bf${i}"
    cd "bf${i}/"

    # Edit
    sed -i "s/$pattern0/$replacement0/g" ./src/main/target/SITL/sitl.c

    replacement1="#define PORT_PWM_RAW    90${i}1    // Out"
    sed -i "s/$pattern1/$replacement1/g" ./src/main/target/SITL/sitl.c

    replacement2="#define PORT_PWM    90${i}2    // Out"
    sed -i "s/$pattern2/$replacement2/g" ./src/main/target/SITL/sitl.c

    replacement3="#define PORT_STATE    90${i}3    // In"
    sed -i "s/$pattern3/$replacement3/g" ./src/main/target/SITL/sitl.c

    replacement4="#define PORT_PWPORT_RCM_RAW    90${i}4    // In"
    sed -i "s/$pattern4/$replacement4/g" ./src/main/target/SITL/sitl.c

    replacement5="#define BASE_PORT 57${i}0"
    sed -i "s/$pattern5/$replacement5/g" ./src/main/drivers/serial_tcp.c

    # Build
    make arm_sdk_install 
    make TARGET=SITL 

    # Copy configured memory
    cp "${gpd_base_path}/gym_pybullet_drones/assets/eeprom.bin" .

done
