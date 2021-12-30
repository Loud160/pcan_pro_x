# PEAK PCAN PRO/PRO FD firmware for STM32F4 based boards

Target hardware:
* Any STM32F407/405/417/415 based boards with 8/25MHz oscillator

Pinout:
|PIN/PINS|DESCRIPTION|
| ------ | ------ |
|PC10|STATUS LED|
|PA2/PA3|TX/RX CAN1 LED|
|PC6/PC7|TX/RX CAN2 LED|
|PB8/PB9|CAN1 RX/TX|
|PB5/PB6|CAN2 RX/TX|
|PB14/PB15 |USB HS DM/DP|
|PA11/PA12 |USB FS DM/DP*|

Features:
- Works out of the box in Linux
- Works with Linux PCAN-View ( needs to install [PEAK Linux drivers][pld] )
- Works with [BUSMASTER][bsmw] and [PEAK PCAN-View][pvw] in Windows


Limits:
- PRO FD firmware does not support FD frames cause bxCAN not supports it, but it will works with classic CAN
- Some protocol specific messages not implemented yet
- Be sure to use **PB14/PB15** pins for USB if you wants **PRO/PRO FD**

Toolchain:
- GNU Arm Embedded Toolchain

Tips:
- PRO FD firmware has better performance on windows ( due internal PEAK driver implementation )

Build:
- PCAN PRO firmware `make pro` only for `USB HS`
- PCAN PRO FD firmware `make pro_fd` only for `USB HS`
- PCAN FD firmware `make fd` supports by `USB FS/HS`
- PCAN FD firmware for HEX-V2 HW `make fd TARGET_CRYSTAL=25 TARGET_USB_ID=FS TARGET_VARIANT=HEXV2_CLONE`


#### Support me
- <a href="https://www.patreon.com/bePatron?u=58145249" data-patreon-widget-type="become-patron-button">Become a Patron!</a>
- **Bitcoin (P2WPKH):** bc1qstnsjqu2kw9v2axens54ycegn3stwvluq7ze5j


License
----

WTFPL

[pld]: <https://www.peak-system.com/fileadmin/media/linux/index.htm>
[pvw]: <https://www.peak-system.com/PCAN-View.242.0.html>
[bsmw]: <https://rbei-etas.github.io/busmaster/>
