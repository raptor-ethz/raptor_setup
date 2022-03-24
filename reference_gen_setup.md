# Reference Generator Setup

You will need three repositories built on your laptop:

- https://github.com/raptor-ethz/position_ctrl_interface
- https://github.com/raptor-ethz/reference_generator
- https://github.com/raptor-ethz/mocap_publisher

The reference generator is a high-level interface that generates position commands for the drone. These commands will be published using FastDDS, received by the position control interface and then sent to the drone over the Xbee link (which also means you can't access QGroundControl and have the position control interface running at the same time). Feel inspired by the trajectories that are already in the reference generator to create your own. 

For flying, you will have to do the following steps:

- Make sure the ViCon is running
- Have the Raspberry Pi running the publisher for the pose to the PX4
- Have the RC connected and ready to kill

Then:
1. Start the mocap_publisher application
2. Start the position control interface. The drone should take off.
3. Once it has reached a stable position, run the reference generator. 

Then, the drone should execute the commands specified in the reference generator. 

Make sure to take appropriate safety precautions before doing this. There can be a lot of things going wrong and a quadcopter out of control is extremely dangerous. Ideally, have multiple persons present, one holding the drone on a tether system, one on the remote ready to use the kill switch and one operating the laptop/PC. Ideally, you also want to have a physical barrier to protect you such as a safety net.

Next, we will consider the [gripper setup](gripper_setup.md).