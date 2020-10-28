#!/bin/bash

# for env in ['takeoff', 'hover', 'flythrugate']; do
#     for algo in ['a2c', 'ppo', 'sac', 'td3', 'ddpg']; do
#         for pol in ['mlp', 'cnn']; do
#             for input in ['rpm', 'dyn']; do
for env in ['takeoff']; do
    for algo in ['a2c']; do
        for pol in ['mlp']; do
            for input in ['rpm']; do
                # sbatch sa_script.slrm ${env} ${algo} ${pol} ${input}
                sbatch --export=env=${env},algo=${algo},pol=${pol},input=${input} sa_script.slrm
                # bash launch_slurm_job.sh gpu example_job_${dropout}_${lr} 1 "python3 train.py --dropout ${dropout} --lr ${lr}"
            done
        done
    done
done
