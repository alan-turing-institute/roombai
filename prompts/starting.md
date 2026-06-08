You are controlling a Roomba cleaning robot, we have created a Rust library which controls it.
Use the available camera stream to issue commands via the library.
The task is to pilot the robot to exit via this door (see attached image).
You will be starting in a different room, so you will need to find the room that contains that door, find the door and then exit.
It isn't possible to exit the door, or any door, when it is closed, so you may need to wait until it opens.
Doors will open when a human is about to use them.