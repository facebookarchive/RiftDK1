Oculus Rift Development Kit 1
=======

Firmware, Schematics, and Mechanicals for the Oculus Rift Development Kit 1.

Firmware
--------

The Tracker firmware uses the CooCox CoIDE for development along with 
the gcc-arm-embedded toolchain.

The ST-Link/V2 is the cheapest way to load firmware onto a Tracker
board, though any SWD compatible programmer should work.  You can follow these steps
to set up a build environment and load firmware onto the board:

1. Install ST-Link drivers if you haven't: http://www.st.com/web/en/catalog/tools/PF258167
2. Install CoCenter: http://www.coocox.org/CooCox_CoIDE.htm
3. Through CoCenter, install CooCox and CoFlash
4. Install gcc-arm-embedded: https://launchpad.net/gcc-arm-embedded/+download
5. Set up gcc-arm-embedded in CooCox: http://www.coocox.org/CoIDE/Compiler_Settings.html
6. Clone our RiftDK1 repository to C:\CooCox\CoIDE\workspace
7. Open the Tracker.coproj
8. Build
9. Plug the ST-Link into the board's 10 pin SWD header.
10. Plug in USB to power the board.
12. Download code to Flash 
13. Enjoy!

Note that the USB Product ID is set to 0x0001, which matches the production
version of the Tracker.  If you modify the firmware in a way that
changes the USB interface, please change the Product ID to 0x0000.

Mechanical
----------

The Mechanical folder contains STEP files of the Rift DK1 housing, control box, and case.

Schematics
----------

The Schematics folder contains a pdf of the schematic, as well as Altium Designer files
of the schematic and board layout.
