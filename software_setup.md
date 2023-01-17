# Software Setup
Here, you will be able to find a guide on how to setup your installation of whatever's required to run our drone in simulation.

# Installation Guide - Ubuntu

The following instructions are written for Ubtuntu 22.04.1 LTS (jammy). Installation may differ for other versions, Linux distributions or operating systems.

__Make sure you have installed the [dependencies](#dependencies) first!__

## Overview
- [MAVSDK](#mavsdk)
- [ROS2](#ROS2)

## MAVSDK

The communication with the PX4 flight controller uses the [MAVLink](https://en.wikipedia.org/wiki/MAVLink) communication protocol. The [MAVSDK](https://mavsdk.mavlink.io/main/en/index.html) includes a collection of libraries which provide a c++ interface for using the MAVLink protocol.

### Installation as debian Package

- Download the latest [release](https://github.com/mavlink/MAVSDK/releases). (Version for 20.04 should work on 22.01)
- Install the downloaded debian package, e.g.
  ```bash
  sudo dpkg -i file.deb
  ```

## ROS2
We use ROS2 as our middleware. To install ROS2 Humble follow the tutorial: 
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html



# Clone our repositories

To run the applications in a simulator, you need to clone the PX4-Autopilot and the raptor repository. For this navigate your terminal to the folder where you wish to install the repositories. Our reccomendation is to install all the repositories in a "src" folder. The folder structure after installation of the two repositoreis should look like this.
```
RAPTOR
|-  src/
    |-  PX4-Autopilot/
    |-  raptor/
```

## Clone PX4-Autopilot
Navigate your terminal into the src/ folder and clone the PX4-Autopilot repository.
```bash
  cd /path/to/RAPTOR/src
  git clone https://github.com/raptor-ethz/PX4-Autopilot
  ```

Run the provided script, to setup the PX4_Autopilot:
  ```bash
  bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
  ```

Some dependencies might not be properly installed by the ubunt.sh script above. In this case, manually install them through the package manager:
- Gazebo
  ```bash
  sudo apt install gazebo libgazebo-dev
  ```

- OpenCV
  ```bash
  sudo apt install libopencv-dev
  ```
  
- GStreamer
  ```bash
  sudo apt install libgstreamer1.0-0 gstreamer1.0-dev gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
  ```
  Note: gstreamer1.0-dev can sometimes not be located but looks like it's not needed anyway 


## Clone Raptor
Navigate your terminal into the "src" folder and clone the raptor repository.
```bash
    cd /path/to/RAPTOR/src
    git clone https://github.com/raptor-ethz/raptor
  ```


# Run a simulation
To run a simulation of our drone you'll need to run three terminals. But first you need to build the programs.
```bash
cd /path/to/RAPTOR
colcon build
. install/setup.bash
```

Your folder structure should now look like this
```
RAPTOR
|-  build/
|-  install/
|-  log/
|-  src/
    |-  PX4-Autopilot/
    |-  raptor/
```

## Start Gazebo Environment
Open a terminal and invoke the respective makefile within the PX4 source code repository:
```bash
cd /path/to/PX4-Autopilot
make px4_sitl gazebo-classic
```

To start a simulator with our RAPTOR drone use:
```bash
cd /path/to/PX4-Autopilot
make px4_sitl gazebo-classic_raptor
```

To start a simulator within the LEO C6 world use:
```bash
cd /path/to/PX4-Autopilot
make px4_sitl gazebo-classic__leoc6
```

To start a simulator with our RAPTOR drone within the LEO C6 world use:
```bash
cd /path/to/PX4-Autopilot
make px4_sitl gazebo-classic_raptor__leoc6
make 
```

## Start mavsdk interface
Open a terminal, source ROS2 and start the mavsdk interface. 
udp://:14540 is used to connect the mavsdk interface with the gazebo simulator.
```bash
cd /path/to/RAPTOR
source /opt/ros/humble/setup.bash
. install/local_setup.bash
ros2 run quad_interface mav_interface udp://:14540
```

## Start reference generator
Open a terminal, source ROS2 and run the reference generator. 
```bash
cd /path/to/RAPTOR
source /opt/ros/humble/setup.bash
. install/local_setup.bash
ros2 run quad_interface reference_generator
```

In the gazebo environment you should now see the drone doing whatever you specified in the main function of the reference generator.
You can find the file in
```bash
cd /path/to/RAPTOR/src/raptor/src/quad_interface/src
```
