#!/bin/sh

# USE
# 1. make the script executable: $ chmod +x ./parallelism.sh
# 2. run the script: $ ./parallelism.sh

for i in {1..5}; do                         # for 5 rounds
  echo -e "\nROUND $i\n"
  for j in {1..4}; do                       # run script.py 4 times
    python script.py --num_drones 20 --gui False --aggregate True --plot False --duration_sec 30 &
  done
  wait                                      # wait for the 4 parallel simulations to complete
done 2>/dev/null                            # redirect STDERR to /dev/null






