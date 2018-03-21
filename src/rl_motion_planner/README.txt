
Goal: 
Train a motion planner with RL so it can:
- follow a specified trajectory
- avoid obstacles

Input:
- trajectory (poses in egocentric ref, velocity and phi)
- last commands (to deal with latency)
- robot's current velocy and phi 
- distance to obstacles around the car

Output:
- list of commands (velocity and phi)

Desired behavior that should be encouraged using rewards:
- move in direction to the goal
- achieve the goal with the right angle, speed, and phi
- avoid obstacles with highest priority
- when one goal is reached, try to achive the next one

Additional considerations
- Train recovery strategies in case things go wrong.


TODO

[ ] read the rddf in python to: 
	- reset the initial position to a random pose in the rddf
		- at least 5m before the last pose
	- swith goals when one is achieved

[ ] implement reward mechanisms:
	- diff distance to the goal
	- reward when dist to goal is smaller than a threshold
	- punishment for collisions
	- some way of rewarding achiving the goal with the right speed, phi, etc.


[ ] how to limit episode size?

[ ] is it possible to speed up the simulation?

[ ] add a ddpg implementation

[ ] investigate if it's possible to use hindsight experience replay

[ ] grab data from MPP to train using SL

[ ] test with the obstacle avoider ligado

[ ] move processes to the module dir

Coisas para conversar com o Alberto

- Como avaliar a qualidade do plano?
- Navigator_gui2 esta' morrendo e com leak memoria (ou alguma outra coisa q faz o computador travar)
- Qual mensagem de behavior selector esta valendo?








