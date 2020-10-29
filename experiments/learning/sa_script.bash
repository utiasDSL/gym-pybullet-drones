#!/bin/bash

# Note to self: 3*5*2*2=60 jobs; do not launch at once

# declare -a env_list=( 'takeoff' 'hover' 'flythrugate' )
# declare -a algo_list=( 'a2c' 'ppo' 'sac' 'td3' 'ddpg' )
# declare -a pol_list=( 'mlp' 'cnn' )
# declare -a input_list=( 'rpm' 'dyn' )

declare -a env_list=( 'takeoff' )
declare -a algo_list=( 'a2c' 'ppo' 'sac' 'td3' 'ddpg' )
declare -a pol_list=( 'mlp' )
declare -a input_list=( 'rpm' 'dyn' )

for env in ${env_list[@]}; do
    for algo in ${algo_list[@]}; do
        for pol in ${pol_list[@]}; do
            for input in ${input_list[@]}; do
                sbatch --export=env=$env,algo=$algo,pol=$pol,input=$input sa_script.slrm
            done
        done
    done
done
