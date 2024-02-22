# Introduction
this project contains a demo simulation of Kuka Youbot manipulator. 
It is implemented making use of the graphic card with open gl. A game controller like Logitech Gamepad F310 is recommended to inspect the scenery.
This simulation is intendet to provide the neccessary info to train RL agents, specifically Multi Agent Adversarial Reinforcement Learning. However, to provide a self-contained example, these communication interfaces are commented out for the sake of simplicity.

# Setup
This simulation is written (and tested) for Linux but should also work for Windows.
 - sudo apt-get install libsdl2-dev  
 - sudo apt-get install freeglut3-dev libglfw3-dev libglm-dev libglew-dev 
 - make clean
 - make

 # Run
 ./visualization