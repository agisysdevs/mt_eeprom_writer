# mt_eeprom_writer
ESP32 program to write serial code received from UART on eeprom

Memory writing protocol
1) using a terminal, type the text to be saved according to SCM_STD_003_Marking_Standard_for_MT_Designed_Electronic_Assemblies.docx Version <1.3>, November 2022, item 6
example: 
PN 1 "00205675" CR 2 "A" PY 1 "08" PW 1 "25" SN 2 "0000000000620089" MI 1 "S" RI 1 "R" MAC 1 "00:10:52:D2:43:FB"
send <CR><LF> to confirm and send the text to the microcontroller
2) possible answers:
2.1) E0: buffer overflow, it has more than 256 characters. Prints the sent buffer.
2.2) E1: buffer too short, it has less than 100 chars. Prints the sent buffer.
2.3) E2: memory read error. Prints the error given by the processor. 
2.4) E3: memory not empty. Prints the information saved on the target address.
2.5) E4: memory write error. Prints the error given by the processor. 
2.6) E5: memory read error. Prints the error given by the processor.
2.7) E6: comparison error, the read string is different than the one written. Prints the read buffer. 
2.8): OK: everyting went OK, prints the read buffer. 
3) commands
3.1) ff: erases the memory (writes 0xFF on every slot), sends "ME OK" when finished
3.2) vv: prints software version and date, example of current version:
V: 0.1 D: 20231124
3.3) other commands, for debugging purposes:
3.3.1) mf: memory fill, sets the program to fill the memory with the next sent character
3.3.2) mp: prints the full extent of the memory
3.3.3) dp: detects and print all the devices on the i2c bus