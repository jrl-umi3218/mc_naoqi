ROS package for mc_rtc <-> naoqi communication.
Instead of using NAOqi SDK, the communication with naoqi OS is based on [naoqi_libqi](http://wiki.ros.org/naoqi_libqi) and [naoqi_libqicore](http://wiki.ros.org/naoqi_libqicore) ros packages:

(apt-get install ros-kinetic-naoqi-libqi)

(apt-get install ros-kinetic-naoqi-libqicore)

Package requires updated version of nao_fastgetsetdcm to be run on the robot, which returns std::vector structures instead of AL::ALValue (see topic/for_mc_rtc_naoqi_ros in nao_festgetsetdcm repository)
