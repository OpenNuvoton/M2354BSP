The M2354 NuMaker-IoT board integrates the computing power of NuMicro M2354 IoT Series and 
complete microcontroller security functions, and equips Wi-Fi and LoRa communication modules on the board. 
The NuMaker-IoT board with prebuilt APP is a demo code based on Mbed OS. 
It could sense pressure, temperature and humidity then displaying on the LCD panel.  
In the meantime, the data also be sent to the AWS cloud by on-board Wi-Fi module. 
For monitoring IoT device easily, Nuvoton provide NuCloudConnector which is designed to connect IoT cloud server 
and monitors IoT devices' status or data.

For more information about NuMaker-IoT-M2354 clould demo, please refer to link:

https://www.nuvoton.com/board/numaker-iot-m2354/?index=2

The pin footprint is different between NuMaker and NuMaker-IoT boards.
For example, the VCOM pins UART0 RXD, TXD of NuMaker are PA6, PA7.
However, the VCOM pin of UART0 RXD, TXD are changed to PB8, PB9.
Most sample codes in BSP is based on NuMaker board except the sample codes under NuMaker-IoT folder.
Therefore, to run these samples on NuMaker-IoT, re-assign the I/O pin may be necessary.

The detail pin comparison list between NuMaker and NuMaker IoT board, please refer to 

NuMaker and NuMaker-IoT board pin table.xlsx


