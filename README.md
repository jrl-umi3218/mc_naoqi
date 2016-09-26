mc_rtc_nao
==

This modules handles communication between Aldebaran's NAO robot and mc_rtc.

Requirements
==

- mc_rtc
- naoqi c++ SDK

How to build
==

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


