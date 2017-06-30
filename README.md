# rob_auto_cal
Automatically calculate the camera/hand and camera/base transformation using a set of movements and an opencv chessboard

This ROS package was developed as a part of a Master's thesis on Technical University of Denmark.
It uses Global polynomial optimizations through semi-definite programming to solve the hand-eye and robot-world transformations simultaneously.
The solution is developed using GpoSolver http://cmp.felk.cvut.cz/gposolver/
It can be used to calibrate both with camera in hand and in world.
The package is developed for use with the Universal Robots UR series of robot and requires the ur_modern_driver.
The package also requires ROS indigo, SDPA, ncurses.
