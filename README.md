# RAPTOR Setup

Welcome to the RAPTOR setup repository! Here, you will be able to find a complete overview of our system and detailed instructions on how to replicate our results. We will provide a detailed overview of the necessary components to get our system up and running and replicate our results.

# Components

First, you will need a number of components. We use off-the-shelf components wherever possible and use custom designs if we can't find an off-the-shelf solution. These custom designs will be made available separately. For our off-the-shelf components, we have the following items:

## Drone and RC components

- [Holybro X500 Kit with a PX4 Flight Controller](https://shop.holybro.com/x500-kit_p1180.html)
- [FrSky Taranis X9D series RC Controller](https://www.frsky-rc.com/product/taranis-x9d-plus-2019/)
- [FrSky RX8R Pro RC Transceiver](https://www.frsky-rc.com/product/rx8r/)
- 2x [Xbee ZigBee Series](https://www.digi.com/products/models/xb3-24z8pt) together with carrier boards with a mini-USB port
- [Sparkfun STDI 3.3V Breakout Board](https://www.sparkfun.com/products/9873)
- [Hobbywing BEC](https://www.conrad.ch/de/p/hobbywing-bec-spannungsregler-6-25-2-v-2108346.html)
- [Swaytronic 5200mAh 4S 14.8V LiPo](https://www.swaytronic.ch/SWAYTRONIC-LiPo-4S-14.8V-5200mAh-85C-170C-XT90)
- A generic 3000mAh USB powerbank
- Raspberry Pi 4B

## Gripper components

- 2x [HiTec Servo](https://www.conrad.ch/de/p/hitec-standard-servo-hsb-9360th-digital-servo-getriebe-material-stahl-vernickelt-stecksystem-jr-1081927.html)
- [STM32-based MCU](https://www.st.com/en/evaluation-tools/nucleo-l432kc.html) (any MCU in Arduino Nano form factor that can run Arduino will do)
- Custom Finray Soft Fingers and servo brackets

  
### Assembly and setup

At the heart of everything, we have the PixHawk 4 flight controller. The X500 kit comes with the carbon frame for the drone, the PX4 and its power board, 4 motors, ESCs for the motors and propellers. There is a detailed assembly guide [here](https://www.youtube.com/watch?v=mNsxkGMAofc). 

After you have assembled the X500 kit, bind the RX8R transceiver to the Taranis X9D. The RX8R will need to be connected to the RC/SBUS input on the PX4. An example for binding can be found [here](https://www.youtube.com/watch?v=1IYg5mQdLVI). Then, you will need to bind the throttle sticks and any other levers you want to use to outputs. For this, you will need the QGroundControl software. The core steps involved in binding physical inputs to outputs on the PX4 are the following:

1. Use the Mixer setting on the Taranis X9D to map physical inputs to channels that will be transmitted
2. With QGroundControl opened, connect the PX4 to your PC using a microUSB cable. You should be able to go into the settings and configure your remote control - there, you can select channels and map channels to commands for the drone. Select the according channels that you have used in the mixer on the Taranis X9D. 

Safety is very important when running applications on your drone that might contain errors that could threaten your safety. We have had our fair share of unpredictable behaviour, especially in early development, so this is important - **set up a physical kill switch on your remote so you can stop the motors at any time, in any situation.**

You will also need to consider a switch for arming the drone.

We have also taken the step of labelling all switches on the remote, particularly the kill switch so that it is always easy to see which one it is, particularly for inexperienced users.

At this point, you should have a very simple drone kit that is fully assembled and a remote controller that connects to your flight controller. Go and undertake a flight test in a safe place. Check that your throttle is at zero, arm the drone, check that your kill switch works and finally, you can take off. Since the drone is only flying with the IMU data from the PixHawk in manual mode, it will be rather unstable with a lot of drift. However, this will be fixed once we [connect the Raspberry Pi](mocap_setup.md) with motion capture.








