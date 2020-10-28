#!/bin/bash

# for env in ['takeoff', 'hover', 'flythrugate']; do
#     for algo in ['a2c', 'ppo', 'sac', 'td3', 'ddpg']; do
for env in ['takeoff', 'hover']; do
    for algo in ['a2c', 'ppo']; do
        for pol in ['mlp', 'cnn']; do
            for input in ['rpm', 'dyn']; do
                sbatch script.slrm ${env} ${algo} ${pol} ${input}
                # bash launch_slurm_job.sh gpu example_job_${dropout}_${lr} 1 "python3 train.py --dropout ${dropout} --lr ${lr}"
            done
        done
    done
done
