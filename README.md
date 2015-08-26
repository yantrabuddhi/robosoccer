# robosoccer


This is a a ros workspace to allow robosapien to play football.

Currently, the code allows the robot to track where the ball is, go towards it, and kick it.

We used color detection (using opencv) to identify and track the ball.


our robosapien contains:
 
 2-servo motors (pan & tilt for the camera)
 arduino pro-mini (necessary)
 bluetooth module (necessary)
 IMU(GY-80)   (necessary)
 sonar sensor (not important for the current code)





To run it:   roslaunch launcher behavior.py