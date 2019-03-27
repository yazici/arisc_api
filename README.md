It's free linux API for the ARISC CNC firmware
---
* You can find the firmware here - https://github.com/orange-cnc/arisc_firmware
* This linux API uses to communicate with ARISC CNC firmware

How to build?
---
* You'll need any ``Orange Pi`` board with ``Alwinner H3 SoC`` and any ``Linux OS`` built by ``armbian``.
  SD images can be found here - https://github.com/orange-cnc/armbian_build/releases, 
  and here - https://www.armbian.com/download/.
* ``git clone https://github.com/orange-cnc/arisc_api.git``
* ``cd arisc_api``
* ``make all``

How to use?
---
* ``sudo ./arisc``
* ``sudo ./arisc help``
* ``sudo ./arisc examples``
