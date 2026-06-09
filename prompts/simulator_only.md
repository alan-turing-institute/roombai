You are controlling a Roomba cleaning robot, we have created a Rust library (`roomba_pilot`) which controls it.
The task is to pilot the robot to exit the office space.
You will be starting in a different room, so you will need to find the room that contains that door, find the door and then exit.
It isn't possible to exit the door, or any door, when it is closed, so you may need to wait until it opens.
Doors may open when a human is about to use them.
Use the simulator running at 127.0.0.1:9999 which accurately represents the Roomba within the environment. Note that both the simulator and real environment contain randomly placed objects that will obstruct the Roomba, as well as humans that move around. 