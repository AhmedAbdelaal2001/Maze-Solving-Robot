# Maze-Solving-Robot
## Overview
The main purpose of this project was to build a robotic car that can enter into a maze, explore it fully to find the shortest path to the exit, and then follow that path in subsequent rounds to solve the maze as quickly as possible. The maze is in the form of black lines that the car must follow, and the line following software uses PID control to ensure stability.
## Needed Components
* An Arduino Board (Nano, Uno, or Mega)
* 5 Line Tracking Sensors (TCRT5000 Modules are preferred)
* 2 L298N Dual H-Bridge Driver Boards
* 4 DC Motors
* A Suitable Car Body
### Note
The speeds of every motor, as well as the PID parameters (kp, kd, and ki) all need to be tuned according to the specific details of the car being used.
