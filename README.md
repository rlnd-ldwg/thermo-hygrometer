# thermo-hygrometer
Thermo-Hygrometer based on DHT22, ATtiny13 and HD44780 compatible display  

 ![LCD](https://github.com/rlnd-ldwg/thermo-hygrometer/raw/master/LCD.png)
 ![Circuit](https://github.com/rlnd-ldwg/thermo-hygrometer/raw/master/circuit.png)

v 1.0.3 wrong indication of negative temperature corrected,
pinout change for better use of IPS header for enhancements,
adaptations for the 16x1 display which uses addresses from a 2-line display
v 1.0.5 corrected error in pcb

To assemble the code, adjust the Makefile to your needs. You have then the following options:

* __make__  
assemble, link and convert to hex format
* __make flash__  
same as above but additionally flash the program
* __make list__  
assemble, link and create list file
* __make clean__  
delete all files except the source file
* __make showfuses__  
show the actual fuses from the mcu
