mc_rtc_nao
==

This modules handles communication between Aldebaran's NAO robot and mc_rtc.

Requirements
==

- mc_rtc
- [naoqi c++ SDK](http://doc.aldebaran.com/2-1/dev/cpp/install_guide.html)
- [nao_fastgetsetdcm](https://gite.lirmm.fr/atanguy/nao_fastgetsetdcm)

How to build
==

First, you need to install the `nao_fastgetsetdcm` module on the NAO.
This module is responsible for fast access to encoders and actuators.
Follow the instructions on [nao_fastgetsetdcm](https://gite.lirmm.fr/atanguy/nao_fastgetsetdcm)


Then you need to build the control module, as follow.
Fist create a toolchain (compilation environment for NAO).

```qitoolchain create naoqi-sdk /path/to/cpp/sdk/toolchain.xml```

Create a workspace, clone the project and build it using NAO sdk

```
cd /path/to/workspace
git clone <mc_rtc_nao.git>
qibuild init
qibuild configure -c naoqi-sdk mc_rtc_nao
qibuild make -c naoqi-sdk mc_rtc_nao
```


