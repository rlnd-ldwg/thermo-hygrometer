# thermo-hygrometer
Thermo-Hygrometer based on DHT22, ATtiny13 and HD44780 kompatible display  

 ![LCD](https://github.com/rlnd-ldwg/thermo-hygrometer/raw/master/LCD.png)
 ![Circuit](https://github.com/rlnd-ldwg/thermo-hygrometer/raw/master/circuit.png)

 
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
