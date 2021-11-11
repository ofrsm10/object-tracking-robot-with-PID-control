# Autonomous
A robot that navigates through an unknown path by reading signs.
i used an Alphabot2 kit and a raspberry pi 3B+.

There are two major components to this project:
object detection (signs, and arrows)
PID controller for object tracking

how the bot follow his path?

the bot should start in front of the first sign.
when the sign is detected, the distance of the sign from the bot is calculated, as well as the signs location relative to the frame.
taking theese inputs in consideration, the PID controller computes the output for each wheel in order to drive corectlly.
when the distance is lower than 50 cm, the bot will read his new direction off the sign.
then the bot will start turning and look for the next sign, when detecte- the bot will go through this procces.
eventually, the bot will arrive at the stop sign and stop the program.

the signs should be painted in marker yellow, and the direcions are arrows that are pointing right or left.
the stop sign is a circle.

have fun!


