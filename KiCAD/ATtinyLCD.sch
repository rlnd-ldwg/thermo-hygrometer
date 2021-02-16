EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr User 8268 5827
encoding utf-8
Sheet 1 1
Title "Thermo-Hygrometer"
Date "2020-02-11"
Rev "1.0.3"
Comp ""
Comment1 "Changed pin assigment"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ATtinyLCD-rescue:74164-74xx_IEEE IC1
U 1 1 5DE15F63
P 4650 2100
F 0 "IC1" H 4600 2400 50  0000 C CNN
F 1 "74164" H 4650 2300 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm" H 4650 2100 50  0001 C CNN
F 3 "" H 4650 2100 50  0001 C CNN
	1    4650 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 2200 4100 2100
Wire Wire Line
	5200 2900 5550 2900
Wire Wire Line
	5550 2800 5200 2800
Wire Wire Line
	5200 2700 5550 2700
Wire Wire Line
	5550 2600 5200 2600
Wire Wire Line
	5200 2500 5550 2500
Wire Wire Line
	5550 2400 5200 2400
Wire Wire Line
	5200 2300 5550 2300
Wire Wire Line
	5550 2200 5350 2200
Wire Wire Line
	5350 2200 5350 2150
Wire Wire Line
	5350 2150 5200 2150
$Comp
L ATtinyLCD-rescue:GND-power #PWR0101
U 1 1 5DE1E26B
P 5950 3400
F 0 "#PWR0101" H 5950 3150 50  0001 C CNN
F 1 "GND" H 5955 3227 50  0000 C CNN
F 2 "" H 5950 3400 50  0001 C CNN
F 3 "" H 5950 3400 50  0001 C CNN
	1    5950 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 3400 5950 3100
Connection ~ 5950 3100
$Comp
L ATtinyLCD-rescue:+5V-power #PWR0102
U 1 1 5DE1FA10
P 5950 1350
F 0 "#PWR0102" H 5950 1200 50  0001 C CNN
F 1 "+5V" H 5965 1523 50  0000 C CNN
F 2 "" H 5950 1350 50  0001 C CNN
F 3 "" H 5950 1350 50  0001 C CNN
	1    5950 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 1600 5550 1600
Wire Wire Line
	5550 1600 5550 1700
Wire Wire Line
	4850 1750 4850 1350
Wire Wire Line
	4850 1350 5950 1350
Connection ~ 5950 1350
Wire Wire Line
	4300 1500 4650 1500
Wire Wire Line
	4650 1500 4650 1750
$Comp
L ATtinyLCD-rescue:R_POT-Device R1
U 1 1 5DE242DA
P 6800 2200
F 0 "R1" H 6730 2154 50  0000 R CNN
F 1 "10k" H 6730 2245 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Piher_PT-10-V10_Vertical_Hole" H 6800 2200 50  0001 C CNN
F 3 "~" H 6800 2200 50  0001 C CNN
	1    6800 2200
	-1   0    0    1   
$EndComp
Wire Wire Line
	6800 1350 6800 2050
Wire Wire Line
	6800 2350 6800 2500
Wire Wire Line
	6800 3100 5950 3100
Wire Wire Line
	6350 2200 6650 2200
Wire Wire Line
	6800 1350 6650 1350
Wire Wire Line
	4100 1850 4000 1850
Wire Wire Line
	4000 1850 4000 1350
Wire Wire Line
	4000 1350 4850 1350
Connection ~ 4850 1350
$Comp
L ATtinyLCD-rescue:DHT11-Sensor IC3
U 1 1 5E08CAFD
P 1700 1100
F 0 "IC3" H 1456 1146 50  0000 R CNN
F 1 "DHT22" H 1456 1055 50  0000 R CNN
F 2 "Sensor:Aosong_DHT11_5.5x12.0_P2.54mm" H 1700 700 50  0001 C CNN
F 3 "http://akizukidenshi.com/download/ds/aosong/DHT11.pdf" H 1850 1350 50  0001 C CNN
	1    1700 1100
	1    0    0    -1  
$EndComp
$Comp
L ATtinyLCD-rescue:+5V-power #PWR0103
U 1 1 5E096469
P 1650 3550
F 0 "#PWR0103" H 1650 3400 50  0001 C CNN
F 1 "+5V" H 1665 3723 50  0000 C CNN
F 2 "" H 1650 3550 50  0001 C CNN
F 3 "" H 1650 3550 50  0001 C CNN
	1    1650 3550
	1    0    0    -1  
$EndComp
$Comp
L ATtinyLCD-rescue:+5V-power #PWR0104
U 1 1 5E09735F
P 2000 1950
F 0 "#PWR0104" H 2000 1800 50  0001 C CNN
F 1 "+5V" H 2015 2123 50  0000 C CNN
F 2 "" H 2000 1950 50  0001 C CNN
F 3 "" H 2000 1950 50  0001 C CNN
	1    2000 1950
	1    0    0    -1  
$EndComp
$Comp
L ATtinyLCD-rescue:+5V-power #PWR0105
U 1 1 5E097AF8
P 1700 800
F 0 "#PWR0105" H 1700 650 50  0001 C CNN
F 1 "+5V" H 1715 973 50  0000 C CNN
F 2 "" H 1700 800 50  0001 C CNN
F 3 "" H 1700 800 50  0001 C CNN
	1    1700 800 
	1    0    0    -1  
$EndComp
$Comp
L ATtinyLCD-rescue:GND-power #PWR0106
U 1 1 5E099422
P 1650 4450
F 0 "#PWR0106" H 1650 4200 50  0001 C CNN
F 1 "GND" H 1655 4277 50  0000 C CNN
F 2 "" H 1650 4450 50  0001 C CNN
F 3 "" H 1650 4450 50  0001 C CNN
	1    1650 4450
	1    0    0    -1  
$EndComp
$Comp
L ATtinyLCD-rescue:GND-power #PWR0107
U 1 1 5E099DB3
P 2000 3150
F 0 "#PWR0107" H 2000 2900 50  0001 C CNN
F 1 "GND" H 2005 2977 50  0000 C CNN
F 2 "" H 2000 3150 50  0001 C CNN
F 3 "" H 2000 3150 50  0001 C CNN
	1    2000 3150
	1    0    0    -1  
$EndComp
$Comp
L ATtinyLCD-rescue:GND-power #PWR0108
U 1 1 5E099F98
P 1700 1400
F 0 "#PWR0108" H 1700 1150 50  0001 C CNN
F 1 "GND" H 1705 1227 50  0000 C CNN
F 2 "" H 1700 1400 50  0001 C CNN
F 3 "" H 1700 1400 50  0001 C CNN
	1    1700 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 2650 3450 1950
Wire Wire Line
	4300 1500 4300 3100
$Comp
L ATtinyLCD-rescue:ATtiny13-20PU-MCU_Microchip_ATtiny IC2
U 1 1 5E08BFAC
P 2000 2550
F 0 "IC2" H 1471 2596 50  0000 R CNN
F 1 "ATtiny13-20PU" H 1471 2505 50  0000 R CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 2000 2550 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/doc2535.pdf" H 2000 2550 50  0001 C CNN
	1    2000 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 1700 3300 2550
Wire Wire Line
	5400 1700 3300 1700
Wire Wire Line
	2600 2250 2800 2250
Wire Wire Line
	2600 2550 3300 2550
Wire Wire Line
	2600 2650 3450 2650
$Comp
L ATtinyLCD-rescue:AVR-ISP-6-Connector JP1
U 1 1 5E08DF28
P 1750 4050
F 0 "JP1" H 1471 4146 50  0000 R CNN
F 1 "AVR-ISP-6" H 1471 4055 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Horizontal" V 1500 4100 50  0001 C CNN
F 3 " ~" H 475 3500 50  0001 C CNN
	1    1750 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 3950 2800 3950
Wire Wire Line
	2800 3950 2800 2250
Wire Wire Line
	2800 2250 2800 1100
Wire Wire Line
	2150 4050 2900 4050
Wire Wire Line
	2900 4050 2900 2450
Wire Wire Line
	4100 2450 4100 2200
Connection ~ 4100 2200
Wire Wire Line
	2150 4150 3000 4150
Wire Wire Line
	3000 4150 3000 2750
Connection ~ 2800 2250
Connection ~ 2900 2450
Wire Wire Line
	2900 2450 2600 2450
Wire Wire Line
	2900 2450 4100 2450
Wire Wire Line
	3000 2750 2600 2750
Wire Wire Line
	2000 1100 2800 1100
$Comp
L ATtinyLCD-rescue:Screw_Terminal_01x02-Connector J1
U 1 1 5E5CB843
P 2200 4650
F 0 "J1" H 2118 4867 50  0000 C CNN
F 1 "Screw_Terminal_01x02" H 2118 4776 50  0000 C CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 2200 4650 50  0001 C CNN
F 3 "~" H 2200 4650 50  0001 C CNN
	1    2200 4650
	-1   0    0    -1  
$EndComp
$Comp
L ATtinyLCD-rescue:+5V-power #PWR0109
U 1 1 5E5D7BED
P 2400 4650
F 0 "#PWR0109" H 2400 4500 50  0001 C CNN
F 1 "+5V" H 2415 4823 50  0000 C CNN
F 2 "" H 2400 4650 50  0001 C CNN
F 3 "" H 2400 4650 50  0001 C CNN
	1    2400 4650
	1    0    0    -1  
$EndComp
$Comp
L ATtinyLCD-rescue:GND-power #PWR0110
U 1 1 5E5D8415
P 2400 4750
F 0 "#PWR0110" H 2400 4500 50  0001 C CNN
F 1 "GND" H 2405 4577 50  0000 C CNN
F 2 "" H 2400 4750 50  0001 C CNN
F 3 "" H 2400 4750 50  0001 C CNN
	1    2400 4750
	1    0    0    -1  
$EndComp
$Comp
L ATtinyLCD-rescue:NHD-0420H1Z-Display_Character U1
U 1 1 5E5DD57F
P 5950 2300
F 0 "U1" H 5950 1411 50  0000 C CNN
F 1 "NHD-0420H1Z" H 5950 1320 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x16_P2.54mm_Vertical" H 5950 1400 50  0001 C CNN
F 3 "http://www.newhavendisplay.com/specs/NHD-0420H1Z-FSW-GBW-33V3.pdf" H 6050 2200 50  0001 C CNN
	1    5950 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 1500 5950 1350
Wire Wire Line
	6350 2200 6350 1500
Wire Wire Line
	6350 1500 6050 1500
Wire Wire Line
	4300 3100 5500 3100
Wire Wire Line
	5550 1900 5400 1900
Wire Wire Line
	5400 1900 5400 1700
Wire Wire Line
	5550 1800 5500 1800
Wire Wire Line
	5500 1800 5500 3100
Connection ~ 5500 3100
Wire Wire Line
	5500 3100 5950 3100
Wire Wire Line
	6350 2500 6800 2500
Connection ~ 6800 2500
Wire Wire Line
	6800 2500 6800 3100
Connection ~ 6650 1350
Wire Wire Line
	6650 1350 5950 1350
Wire Wire Line
	6650 1350 6650 2600
Wire Wire Line
	3450 1950 4100 1950
Connection ~ 4100 2100
Wire Wire Line
	4100 1600 4100 2100
$Comp
L ATtinyLCD-rescue:R-Device R2
U 1 1 5E5FF9C7
P 6500 2600
F 0 "R2" V 6293 2600 50  0000 C CNN
F 1 "R" V 6384 2600 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 6430 2600 50  0001 C CNN
F 3 "~" H 6500 2600 50  0001 C CNN
	1    6500 2600
	0    1    1    0   
$EndComp
Wire Wire Line
	2600 2350 2700 2350
Wire Wire Line
	2700 3850 2700 2350
Wire Wire Line
	2150 3850 2700 3850
$EndSCHEMATC
