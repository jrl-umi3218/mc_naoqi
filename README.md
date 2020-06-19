# mc_naoqi

### Interface to handle communication between `mc_rtc` and NAO/Pepper robots

![mc_naoqi](doc/mc_naoqi.png "mc_naoqi interface")


# Installation

## On the control computer

Instead of using NAOqi SDK, the communication with naoqi OS is based on [naoqi_libqi](http://wiki.ros.org/naoqi_libqi) and [naoqi_libqicore](http://wiki.ros.org/naoqi_libqicore) ros packages:

```sh
sudo apt-get install ros-kinetic-naoqi-libqi
sudo apt-get install ros-kinetic-naoqi-libqicore
```

Then compile `mc_naoqi`.

``` bash
git clone git@gite.lirmm.fr:multi-contact/mc_naoqi.git
cd mc_naoqi
git submodule update --init
mkdir build
cd build
cmake ..
make
```

## On the robot

Communication with the robot sensors and actuators is managed using the embedded [DCM](http://doc.aldebaran.com/2-1/naoqi/sensors/dcm.html) module on the robot.
To access its features, [mc_naoqi_dcm](https://gite.lirmm.fr/softbankrobotics/mc_naoqi_dcm) needs to be installed and run on the robot.

# Usage

To use the interface and connect to a real robot run

```bash
# while in 'build' folder
./src/mc_naoqi -h <robot_hostname> -p <robot_port> -f <mc_rtc_configuration_file.conf>
```

If you wish to run the simulation only use `simulation` as a `<robot_hostname>`

```bash
# while in 'build' folder
./src/mc_naoqi -h simulation -f <mc_rtc_configuration_file.conf>
```

## CLI Commands

- `on` : servo on actuators
- `off` : servo off actuators
- `s` : starts `mc_rtc` controller if stopped, stops otherwise
- `hs` : go to half-sitting posture
- `cc <controller name>` : change controller

## ROS Services

If `mc_rtc` was compiled with ROS support, then services will be available to interact with controllers. You can see the list of all services available with

```sh
rosservice list /mc_rtc
```
