It's free linux API for the ARISC CNC firmware
---
* You can find the firmware here - https://github.com/orangecnc/h3_arisc_firmware
* This linux API uses to communicate with ARISC CNC firmware

How to build?
---
* You'll need any ``Orange Pi`` board with ``Alwinner H3 SoC`` and any ``Linux OS`` built by ``armbian``.
  SD images can be found here - https://github.com/orangecnc/armbian_build/releases, 
  and here - https://www.armbian.com/download/.
* ``git clone https://github.com/orangecnc/h3_arisc_api.git``
* ``cd h3_arisc_api``
* ``make all``

How to use?
---
* ``sudo ./arisc``
* ``sudo ./arisc help``
* ``sudo ./arisc examples``

Where I can download pre-built binaries?
---
* https://github.com/orangecnc/h3_arisc_firmware/releases
* https://github.com/orangecnc/h3_arisc_api/releases

