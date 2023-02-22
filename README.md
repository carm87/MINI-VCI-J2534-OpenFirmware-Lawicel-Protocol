# MINI-VCI-J2534-OpenFirmware-Lawicel-Protocol
This is the firmware I ported from Peak System CAN-RS232 hardware to Chinese MINI-VCI J2534 to implement on it the Lawicel Protocol that seems to be the standard de facto for serial to can adapter.

## So we can use MVCI with following tools:
- Bustmaster (best freeware CAN monitor tool) via VSCOM driver (with modified api to use 115200, the original one will try 3M baud on USB-CAN FTDI converter). NEW: I added a specific HEX file and VSCOM driver to work at 1250000bps on the virtual serial. 
- Any diagnostic tool that support J2534 via Sardine CAN api (with modified dlls and reg key to support different bauds, the original one uses only 125KBPS). NEW: I added a specific HEX file reg file to work at 1250000bps on the virtual serial.
- Canhacker
- Titan Can

## For whom wants just to use this firmware:

1- connect your MINI-VCI and verify in windows control panel driver is correctly loaded shoul be COMx present
2- open Flash Magic or LPC210x_ISP, select port COMx, baud 38400, select file MVCI_Lawicel_prot_v0001.bin/.hex
3- write it into device flash


### RTS/DTR/CTS/DSR...

MVCI devices support LPC2119 reset and enable bootloadr via DTR and RTS.

This makes very easy programming it via windows tool (no need to open it to enter in boot mode).

The side effect is that is diffulct to interface with existing software due to the fact that RTS/DTR management depens of the flow control that developers chosed to use.

I found two working configurations: 

###Configuration for busmaster and sardine can:
in FT_prog, read device, go into Hardware specific, invert RTS, CTS, program devicesm unplug and replug devide.
if you need to use again flashmagic open FT_prog and revert those changes.
make sure that sardine registry antry has DISABLE_DTR=1

###Configuration for busmaster and sardine can:
in FT_prog, read device, go into Hardware specific, invert RTS, CTS, program devicesm unplug and replug devide.
if you need to use again flashmagic open FT_prog and revert those changes.
make sure that sardine registry antry has DISABLE_DTR=1

###Configuration for canhacker , titan :
in FT_prog, read device, go into Hardware specific, invert DTR, program devicesm unplug and replug devide.
if you need to use again flashmagic open FT_prog and revert those changes.


A possible way to use all the tool without reprogramming the FTDI eeprom is:
create a virtual COM port with Virtual Serial Ports Emulator e.g COMy, open realterm select COMx and 115200, properly set CTS and DTR in the pins tab to have blinking led. In echo port select COMy, 115200 and open it. Your application or dll shall be configured to work on COMy.
This could be also used to debug (realterm windows will show, if enabled, received and sent commands).

Really appreciaty any other suggestion




## For whom want to develop:

Download from peak-system development pack

https://www.peak-system.com/PEAK-DevPack.565.0.html?&L=1

Download Visual Studio ( I used v 1.28)

Replace GIT source in the PCAN-Router_DR folder (PCAN-Router_DR shares LPC2119 with MINI VCI).

Run VBS script.

you can run make all task.
