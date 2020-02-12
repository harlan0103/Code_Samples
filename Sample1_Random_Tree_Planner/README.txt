Name: Haoran Liang

Directory strucutre:
---- src
	|---- CollisionChecking.cpp
	|---- CollisionChecking.h
	|---- RTP.cpp
	|---- RTP.h
	|---- SquareBoxPlanner.cpp
---- result_path.png
---- README.txt

Description:
This code sample is from a course project I was written at Rice University. The goal for this project is to implement the Random Tree Planner algorithm and apply this algorithm to a square robot to find a path between start and end points. The square robot is able to translate and rotate in the 2D space. There are some obstacles inside of the given environment and the Random Tree Planner need to avoid obstacles.

--- CollisionChecking.cpp:
	This file contains some collision checking algorithms. By given a set of obstacles and a robot, the checking algorithm can determine if the robot is collision with obstacles. The shape of the robot can be point, circle and square. The algorithm will determine if the given robot is intersect, overlap or not collision with obstacles.

--- RTP.cpp:
	This file specify the detail of Random Tree Planner algorithm. The procuedure of this algorithm is to first select a random configuration from the existing Random Tree. Then sample a random configuration from the configuration space, with small chance, the sampled one is the goal configuration. The last step is to connect two configurations and check if the straight line between these two configurations is collision-free with obstacles. If it is collision free, then add new sampled configuration into the Random Tree. Repeat the 1-3 steps until the algorithm find a path between start and goal configuration.

--- SquareBoxPlanner.cpp:
	This file creates obstacles and start and goal points for the environment. And will load RTP algorithm to find a path for the start and goal configurations, then print out the path on the terminal.

result_path.png includes an example of path planning for a square robot

This code sample was written on: 2019.10.10