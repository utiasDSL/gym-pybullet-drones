#!/bin/bash

# USE
# 1. ..

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
mkdir betaflights/
cd betaflights/

for ((i = 1; i <= num_iterations; i++)); do

    # Clone
    git clone https://github.com/betaflight/betaflight "bf${i}"
    cd "bf${i}/"

    # Edit
    # comment out line `delayMicroseconds_real(50); // max rate 20kHz` in ./src/main/main.c
    # edit ports in ./src/main/target/SITL/sitl.c, ./src/main/drivers/serial_tcp.c

    # Build
    make arm_sdk_install 
    make TARGET=SITL 

    # Copy configured memory
    cp "${gpd_base_path}/gym_pybullet_drones/assets/eeprom.bin" .

done

# # Define the file you want to modify
# file="your_source_code.c"

# # Define the base pattern to search for
# pattern1="#define PORT_PWM_RAW    9001    // Out"
# pattern2="#define PORT_PWM        9002    // Out"
# pattern3="#define PORT_STATE      9003    // In"
# pattern4="#define PORT_RC         9004    // In"

# # Use a for loop to perform replacements
# for ((i = 1; i <= 10; i++)); do
#     current_pattern="${base_pattern}${i}"
#     replacement="new_pattern_${i}"
#     sed -i "s/$current_pattern/$replacement/g" "$file"
# done
