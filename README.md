example of using a teco L510 VFD with an esp32.

This uses UART1 of the esp32 via pins 16 and 17.  The RS485 module is the HW-0519 with automagic rx/tx enable.

To control the VFD:

CR/LF control the commands, ensure you have  it set in your serial program
Start "+" 
Stop "-" 
Frequency "F XXXXX" where XXXXX is the frequency * 100 aka 10000 == 100hz
