#!/bin/sh
#$ -cwd
#$ -l q_node=1
#$ -j y
#$ -l h_rt=24:00:00
#$ -o output/o.$JOB_ID

. /etc/profile.d/modules.sh



module purge
#module load python/3.8.3
module load intel openmpi
module load cuda/11.2.146
module load cudnn/8.1
module load nccl/2.4.2


source ~/.bashrc
conda activate pydrone37

python3 -m pip install --user --upgrade pip
python3 -m pip install --user laspy
#python3 -m pip install --user scikit-learn
python3 -m pip install --user matplotlib
python3 -m pip install --user gym
#python3 -m pip install --user tensorflow
python3 -m pip install --user keras
python3 -m pip install --user keras_preprocessing
python3 -m pip install --user opencv-python
#python3 -m pip install --user pydot

python3 -m pip install --user packaging
python3 -m pip install --user tensorflow_probability
python3 -m pip install --user scikit-image
python3 -m pip install --user pybullet
#python3 -m pip install --user pickle5
python3 -m pip install --user "ray[rllib]==2.3"
#python3 -m pip install --user "ray[rllib] @ LINK_TO_WHEEL.whl"
#python3 -m pip install --user torchvision torchaudio
#python3 -m pip install --user torch 
python3 -m HOROVOD_GPU_ALLREDUCE=NCCL pip install --no-cache-dir horovod
#python3 -m pip install --user https://download.pytorch.org/whl/cpu/torch-1.0.1-cp36-cp36m-win_amd64.whl
#python3 -m pip install --user torch==1.12.1+cu113 torchvision==0.13.1+cu113 torchaudio==0.12.1 --extra-index-url https://download.pytorch.org/whl/cu113
python3 -m pip install --user torch==1.12.1+cu113 torchvision==0.13.1+cu113 torchaudio==0.12.1 --extra-index-url https://download.pytorch.org/whl/cu113

python3 experiments/learning/multiagent.py --act dyn --env leaderfollower
