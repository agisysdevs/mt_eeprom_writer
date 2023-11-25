# mt_eeprom_writer
ESP32 program to write serial code received from UART on eeprom

Memory Writing Protocol

Writing Procedure

1. Use a terminal to type the text to be saved according to the SCM_STD_003_Marking_Standard_for_MT_Designed_Electronic_Assemblies.docx Version <1.3>, November 2022, item 6.

Example:

PN 1 "00205675" CR 2 "A" PY 1 "08" PW 1 "25" SN 2 "0000000000620089" MI 1 "S" RI 1 "R" MAC 1 "00:10:52:D2:43:FB"

2. Send a carriage return (CR) and line feed (LF) to confirm and send the text to the microcontroller.

Possible Responses

Error Code | Description | Action
------- | -------- | --------
E0 | Buffer overflow (> 256 characters) | Prints the sent buffer.
E1 | Buffer too short (< 100 characters) | Prints the sent buffer.
E2 | Memory read error | Prints the error given by the processor.
E3 | Memory not empty | Prints the information saved on the target address.
E4 | Memory write error | Prints the error given by the processor.
E5 | Memory read error | Prints the error given by the processor.
E6 | Comparison error (read string differs from written string) | Prints the read buffer.
OK | Success | Prints the read buffer.

Commands

Command | Description | Response
------- | -------- | --------
ff | Erases the memory (writes 0xFF to every slot) | Sends "ME OK" when finished.
vv | Prints the software version and date (example of current version) | V: 0.1 D: 20231124
mf | Memory fill | Sets the program to fill the memory with the next sent character. (*)
mp | Prints the full extent of the memory | Prints the contents of the memory. (*)
dp | Detects and prints all devices on the I2C bus | Prints a list of detected I2C devices. (*)

(*) These commands are primarily for debugging purposes.