#!/bin/bash

# test pretrained policy for mario
time python train_ppo_ale.py --env SuperMarioBros-1-1-v0 --demo --load /home/computing/Desktop/pfrl/results-ppo-supermario/20220416T153806.685398-run3/4415616_except --eval-n-runs 100 --render

