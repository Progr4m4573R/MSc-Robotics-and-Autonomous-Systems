Rem @echo off 
	
Rem This is for automating machine learning assignment

pip install torch
pip install gym
pip install ale-py
pip install nes-py
pip install gym-super-mario-bros
pip install gym[accept-rom-license]
pip install OpenCV-Python
pip install filelock

python train_dqn_ale.py --env BreakoutNoFrameskip-v4 --arch doubledqn --final-exploration-frames 300000 --steps 330000
echo "The program has completed"