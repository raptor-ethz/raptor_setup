# Gripper Setup

The gripper is a relatively simple composition of parts: we have the microcontroller which controls the servos holding the fingers. The microcontroller receives commands over serial from the RPi. From [this repository](https://github.com/raptor-ethz/gripper_interface), you will need to have the gripper interface running on the RPi and the Arduino file needs to be flashed to the MCU. Connect the RPI to the MCU using a USB cable. 

The servos draw a significant amount of current, so we need a sufficient source of power for them. We can utilize the BEC to supply their operating voltage. Connect the BEC in parallel to the battery (we've soldered together an XT60/90 splitter cable for this) and set it up to supply the appropriate operating voltage. Now, we have a stable voltage and can supply vast amounts of currents to the servos with the BEC. You can solder everything together with a solderable breadboard, but make sure to use sockets for the MCU. 

So, a little overview:
- The MCU is connected to the RPi using a USB cable
- The servo power pins are connected to the BEC on a solderable breadboard
- The servo PWM pins are connected to the pins specified in the Arduino program

You can verify your setup by running the gripper test program in the reference generator repository. 

With the gripper correctly installed, you will now be able to write applications in the reference generator that utilise the gripper. This allows for fast, dynamic pickup of objects which you can creatively use at your will.