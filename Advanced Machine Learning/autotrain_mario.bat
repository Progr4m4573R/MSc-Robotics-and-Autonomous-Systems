Rem @echo off 
Rem This is for automating training of machine learning mario game
python train_dqn_ale.py --env SuperMarioBros-1-1-v0 --arch doubledqn --final-exploration-frames 300000 --steps 330000 --gpu -1
echo "The program has completed"