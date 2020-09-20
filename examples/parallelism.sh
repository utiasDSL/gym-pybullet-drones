#!/bin/bash

# USE
# 1. make the script executable: $ chmod +x ./parallelism.sh
# 2. run the script: $ ./parallelism.sh

for i in {1..25}; do                        # for 25 rounds
  echo -e "\nROUND $i\n"
  for j in {1..4}; do                       # run the fly.py script on the following line 4 times
    python fly.py --num_drones 20 --gui False --aggregate True --plot False --duration_sec 30 &
  done
  wait                                      # wait for the 4 parallel simulations to complete
done 2>/dev/null                            # redirect STDERR to /dev/null

