#!/bin/bash

# declare -a env_list=( 'takeoff' 'hover' 'flythrugate' )
# declare -a algo_list=( 'a2c' 'ppo' 'sac' 'td3' 'ddpg' )
# declare -a obs_list=( 'kin' 'rgb' )
# declare -a act_list=( 'rpm' 'dyn' 'pid' 'one_d_rpm' 'one_d_dyn' 'one_d_pid' )

# Note: 3*5*2*2=60 jobs; do not launch at once

declare -a env_list=( 'hover' )
declare -a algo_list=( 'ppo' 'sac' 'ddpg' )
declare -a obs_list=( 'kin' 'rgb' )
declare -a act_list=( 'one_d_rpm' )

for env in ${env_list[@]}; do
    for algo in ${algo_list[@]}; do
        for obs in ${obs_list[@]}; do
            for act in ${act_list[@]}; do
                sbatch --export=env=$env,algo=$algo,obs=$obs,act=$act sa_script.slrm
            done
        done
    done
done
