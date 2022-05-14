#!/bin/bash


# test pretrained policy for breakout
time python train_ppo_ale.py --env BreakoutNoFrameskip-v4 --demo --load /home/computing/Desktop/pfrl/results-ppo-breakout/20220419T102547.103901-run1/2010480_except --eval-n-runs 100 --render

echo testing breakout
