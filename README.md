# MINI-VCI-J2534-OpenFirmware-Lawicel-Protocol
This is the firmware I ported from Peak RS232 to Chinese MINI-VCI J2534 to implement on in the Lawicel Protocol (e.g. CANHACKER tool or Sardine CAN J2534 dll)

For whom wants just to use this firmware:

1- connect your MINI-VCI and verify in windows control panel driver is correctly loaded shoul be COMx present
2- open Flash Magic or LPC210x_ISP, select port COMx, baud 38400, select file MVCI_Lawicel_prot_v0001.bin/.hex
3- write it into device flash

Now we have to manage the MCU reset when the application or dll open COMx port due to DTR level change.
We can procede in 3 different ways:

1 - cut on the PCB the track that from the FTD chip goes into the LPC2119 reset pin (deprecated)
2 - open FT_prog, read device, go into Hardware specific, invert RTS, CTS, DTR, RTS, program devices.
3 - create a virtual COM port with Virtual Serial Ports Emulator e.g COMy, open realterm select COMx and 115200, in pins clear CTS, clear DTR (with this order). In echo port select COMy, 115200 and open it. Your application or dll shall be configured to work on COMy.

The 3rd way can be also used to debug (realterm windows will show, if enabled, received and sent commands).

For whom want to develop:

Download from peak-system development pack
https://www.peak-system.com/PEAK-DevPack.565.0.html?&L=1
Download Visual Studio ( I used v 1.28)
Replace GIT source in the PCAN-Router_DR folder (PCAN-Router_DR shares LPC2119 with MINI VCI).
Run VBS script.

you can run make all task.
