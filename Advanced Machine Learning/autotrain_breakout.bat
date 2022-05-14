Rem @echo off 
Rem This is for automating training of machine learning breakoutgame
python train_dqn_ale.py --env BreakoutNoFrameskip-v4 --arch doubledqn --final-exploration-frames 300000 --steps 330000 --gpu -1
echo "The program has completed"