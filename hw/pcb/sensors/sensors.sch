EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L mdbt:MDBT42Q U2
U 1 1 611973F1
P 5700 3350
F 0 "U2" H 5700 5637 60  0000 C CNN
F 1 "MDBT42Q" H 5700 5531 60  0000 C CNN
F 2 "mdbt:MDBT42Q" H 5200 3350 60  0001 C CNN
F 3 "" H 5200 3350 60  0001 C CNN
	1    5700 3350
	1    0    0    -1  
$EndComp
$Comp
L Sensor:BME680 U1
U 1 1 61199BC7
P 2900 5300
F 0 "U1" H 2471 5346 50  0000 R CNN
F 1 "BME680" H 2471 5255 50  0000 R CNN
F 2 "Package_LGA:Bosch_LGA-8_3x3mm_P0.8mm_ClockwisePinNumbering" H 4350 4850 50  0001 C CNN
F 3 "https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME680-DS001.pdf" H 2900 5100 50  0001 C CNN
	1    2900 5300
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_ABRG D1
U 1 1 611D5334
P 7300 5200
F 0 "D1" H 7300 5697 50  0000 C CNN
F 1 "LED_ABRG" H 7300 5606 50  0000 C CNN
F 2 "led:WL-SFTW" H 7300 5150 50  0001 C CNN
F 3 "~" H 7300 5150 50  0001 C CNN
	1    7300 5200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 611DA48C
P 6700 5000
F 0 "R3" V 6600 4900 50  0000 C CNN
F 1 "100" V 6600 5150 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric_Pad0.72x0.64mm_HandSolder" V 6630 5000 50  0001 C CNN
F 3 "~" H 6700 5000 50  0001 C CNN
	1    6700 5000
	0    1    1    0   
$EndComp
Wire Wire Line
	6850 5000 7100 5000
Wire Wire Line
	7100 5200 6850 5200
Wire Wire Line
	6850 5400 7100 5400
$Comp
L power:VDD #PWR0101
U 1 1 611DC398
P 7700 4550
F 0 "#PWR0101" H 7700 4400 50  0001 C CNN
F 1 "VDD" H 7715 4723 50  0000 C CNN
F 2 "" H 7700 4550 50  0001 C CNN
F 3 "" H 7700 4550 50  0001 C CNN
	1    7700 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 4550 7700 5200
Wire Wire Line
	7700 5200 7500 5200
Text Label 6250 5000 0    50   ~ 0
LED_R
Wire Wire Line
	6550 5000 6250 5000
Wire Wire Line
	6550 5200 6250 5200
Wire Wire Line
	6550 5400 6250 5400
Text Label 6250 5200 0    50   ~ 0
LED_G
Text Label 6250 5400 0    50   ~ 0
LED_B
$Comp
L power:GND #PWR0102
U 1 1 611E918E
P 3650 5000
F 0 "#PWR0102" H 3650 4750 50  0001 C CNN
F 1 "GND" V 3655 4872 50  0000 R CNN
F 2 "" H 3650 5000 50  0001 C CNN
F 3 "" H 3650 5000 50  0001 C CNN
	1    3650 5000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2800 5900 2800 5950
Wire Wire Line
	2800 5950 2900 5950
Wire Wire Line
	3000 5950 3000 5900
Wire Wire Line
	2900 6050 2900 5950
Connection ~ 2900 5950
Wire Wire Line
	2900 5950 3000 5950
$Comp
L Device:C C2
U 1 1 611E9F93
P 2250 4800
F 0 "C2" H 2135 4754 50  0000 R CNN
F 1 "100n" H 2135 4845 50  0000 R CNN
F 2 "Capacitor_SMD:C_0402_1005Metric_Pad0.74x0.62mm_HandSolder" H 2288 4650 50  0001 C CNN
F 3 "~" H 2250 4800 50  0001 C CNN
	1    2250 4800
	-1   0    0    1   
$EndComp
$Comp
L Device:C C1
U 1 1 611ED1BF
P 1850 4800
F 0 "C1" H 1735 4754 50  0000 R CNN
F 1 "100n" H 1735 4845 50  0000 R CNN
F 2 "Capacitor_SMD:C_0402_1005Metric_Pad0.74x0.62mm_HandSolder" H 1888 4650 50  0001 C CNN
F 3 "~" H 1850 4800 50  0001 C CNN
	1    1850 4800
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 611ED907
P 2050 5050
F 0 "#PWR0103" H 2050 4800 50  0001 C CNN
F 1 "GND" H 2055 4877 50  0000 C CNN
F 2 "" H 2050 5050 50  0001 C CNN
F 3 "" H 2050 5050 50  0001 C CNN
	1    2050 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 4950 1850 5000
Wire Wire Line
	1850 5000 2050 5000
Wire Wire Line
	2250 5000 2250 4950
Wire Wire Line
	2050 5050 2050 5000
Connection ~ 2050 5000
Wire Wire Line
	2050 5000 2250 5000
Wire Wire Line
	1850 4650 1850 4550
Wire Wire Line
	1850 4550 2250 4550
Wire Wire Line
	3000 4550 3000 4700
Wire Wire Line
	2800 4700 2800 4550
Connection ~ 2800 4550
Wire Wire Line
	2800 4550 3000 4550
Wire Wire Line
	2250 4650 2250 4550
Connection ~ 2250 4550
Wire Wire Line
	2250 4550 2800 4550
$Comp
L power:VDD #PWR0104
U 1 1 611F0926
P 2800 4350
F 0 "#PWR0104" H 2800 4200 50  0001 C CNN
F 1 "VDD" H 2815 4523 50  0000 C CNN
F 2 "" H 2800 4350 50  0001 C CNN
F 3 "" H 2800 4350 50  0001 C CNN
	1    2800 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 4350 2800 4550
$Comp
L power:VDD #PWR0105
U 1 1 611F15C6
P 3650 5600
F 0 "#PWR0105" H 3650 5450 50  0001 C CNN
F 1 "VDD" V 3665 5728 50  0000 L CNN
F 2 "" H 3650 5600 50  0001 C CNN
F 3 "" H 3650 5600 50  0001 C CNN
	1    3650 5600
	0    1    1    0   
$EndComp
Wire Wire Line
	3650 5600 3500 5600
Text Label 5050 5400 2    50   ~ 0
I2C_SDA
Wire Wire Line
	5050 5400 4400 5400
Text Label 5050 5200 2    50   ~ 0
I2C_SCL
Wire Wire Line
	5050 5200 4100 5200
Wire Wire Line
	3650 5000 3500 5000
$Comp
L power:GND #PWR0106
U 1 1 611F5072
P 2900 6050
F 0 "#PWR0106" H 2900 5800 50  0001 C CNN
F 1 "GND" H 2905 5877 50  0000 C CNN
F 2 "" H 2900 6050 50  0001 C CNN
F 3 "" H 2900 6050 50  0001 C CNN
	1    2900 6050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 611F5BDA
P 4100 4900
F 0 "R1" H 3950 4850 50  0000 C CNN
F 1 "4k7" H 3950 4950 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric_Pad0.72x0.64mm_HandSolder" V 4030 4900 50  0001 C CNN
F 3 "~" H 4100 4900 50  0001 C CNN
	1    4100 4900
	-1   0    0    1   
$EndComp
Wire Wire Line
	4100 5050 4100 5200
Connection ~ 4100 5200
Wire Wire Line
	4100 5200 3500 5200
Wire Wire Line
	4400 5050 4400 5400
Connection ~ 4400 5400
Wire Wire Line
	4400 5400 3500 5400
Wire Wire Line
	3000 4550 4100 4550
Wire Wire Line
	4400 4550 4400 4750
Connection ~ 3000 4550
Wire Wire Line
	4100 4750 4100 4550
Connection ~ 4100 4550
Wire Wire Line
	4100 4550 4400 4550
$Comp
L power:GND #PWR0107
U 1 1 611FA348
P 4750 1600
F 0 "#PWR0107" H 4750 1350 50  0001 C CNN
F 1 "GND" V 4755 1472 50  0000 R CNN
F 2 "" H 4750 1600 50  0001 C CNN
F 3 "" H 4750 1600 50  0001 C CNN
	1    4750 1600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 611FB21D
P 6650 1600
F 0 "#PWR0108" H 6650 1350 50  0001 C CNN
F 1 "GND" V 6655 1472 50  0000 R CNN
F 2 "" H 6650 1600 50  0001 C CNN
F 3 "" H 6650 1600 50  0001 C CNN
	1    6650 1600
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 611FBDB3
P 6300 3600
F 0 "#PWR0109" H 6300 3350 50  0001 C CNN
F 1 "GND" H 6305 3427 50  0000 C CNN
F 2 "" H 6300 3600 50  0001 C CNN
F 3 "" H 6300 3600 50  0001 C CNN
	1    6300 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 611FC94C
P 5100 3600
F 0 "#PWR0110" H 5100 3350 50  0001 C CNN
F 1 "GND" H 5105 3427 50  0000 C CNN
F 2 "" H 5100 3600 50  0001 C CNN
F 3 "" H 5100 3600 50  0001 C CNN
	1    5100 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 1600 4800 1600
Wire Wire Line
	6650 1600 6600 1600
Wire Wire Line
	6300 3600 6300 3550
$Comp
L Device:C C4
U 1 1 612021FD
P 4650 3200
F 0 "C4" H 4765 3246 50  0000 L CNN
F 1 "10 u" H 4765 3155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 4688 3050 50  0001 C CNN
F 3 "~" H 4650 3200 50  0001 C CNN
	1    4650 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal Y1
U 1 1 61202DE7
P 5350 4400
F 0 "Y1" H 5600 4450 50  0000 C CNN
F 1 "32.768 kHz 9 pF FC-135" H 6050 4350 50  0000 C CNN
F 2 "Crystal:Crystal_SMD_3215-2Pin_3.2x1.5mm" H 5350 4400 50  0001 C CNN
F 3 "~" H 5350 4400 50  0001 C CNN
	1    5350 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3600 5100 3550
Wire Wire Line
	5500 4400 5500 4150
$Comp
L Device:C C5
U 1 1 612072FF
P 5200 4650
F 0 "C5" H 5450 4700 50  0000 R CNN
F 1 "12 p" H 5500 4600 50  0000 R CNN
F 2 "Capacitor_SMD:C_0402_1005Metric_Pad0.74x0.62mm_HandSolder" H 5238 4500 50  0001 C CNN
F 3 "~" H 5200 4650 50  0001 C CNN
	1    5200 4650
	-1   0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 61207C3D
P 5500 4650
F 0 "C6" H 5615 4696 50  0000 L CNN
F 1 "12 p" H 5615 4605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric_Pad0.74x0.62mm_HandSolder" H 5538 4500 50  0001 C CNN
F 3 "~" H 5500 4650 50  0001 C CNN
	1    5500 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 4500 5200 4400
Connection ~ 5200 4400
Wire Wire Line
	5500 4500 5500 4400
Connection ~ 5500 4400
$Comp
L power:GND #PWR0111
U 1 1 6120ACED
P 5350 4900
F 0 "#PWR0111" H 5350 4650 50  0001 C CNN
F 1 "GND" H 5355 4727 50  0000 C CNN
F 2 "" H 5350 4900 50  0001 C CNN
F 3 "" H 5350 4900 50  0001 C CNN
	1    5350 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 4800 5200 4850
Wire Wire Line
	5200 4850 5350 4850
Wire Wire Line
	5500 4850 5500 4800
Wire Wire Line
	5350 4900 5350 4850
Connection ~ 5350 4850
Wire Wire Line
	5350 4850 5500 4850
$Comp
L Device:L L2
U 1 1 61210148
P 4350 2900
F 0 "L2" V 4300 2950 50  0000 C CNN
F 1 "10 u" V 4200 2900 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 4350 2900 50  0001 C CNN
F 3 "~" H 4350 2900 50  0001 C CNN
	1    4350 2900
	0    -1   -1   0   
$EndComp
$Comp
L Device:L L1
U 1 1 61211238
P 3950 2900
F 0 "L1" V 3900 2850 50  0000 C CNN
F 1 "15 n" V 3800 2800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric_Pad0.74x0.62mm_HandSolder" H 3950 2900 50  0001 C CNN
F 3 "~" H 3950 2900 50  0001 C CNN
	1    3950 2900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4100 2900 4200 2900
Wire Wire Line
	4500 2900 4800 2900
Wire Wire Line
	4800 2800 3700 2800
Wire Wire Line
	3700 2800 3700 2900
Wire Wire Line
	3700 2900 3800 2900
$Comp
L Device:C C3
U 1 1 61215E62
P 3700 3200
F 0 "C3" H 3815 3246 50  0000 L CNN
F 1 "1 u" H 3815 3155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 3738 3050 50  0001 C CNN
F 3 "~" H 3700 3200 50  0001 C CNN
	1    3700 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 3050 3700 2900
Connection ~ 3700 2900
$Comp
L power:GND #PWR0112
U 1 1 61217C8D
P 3700 3450
F 0 "#PWR0112" H 3700 3200 50  0001 C CNN
F 1 "GND" H 3705 3277 50  0000 C CNN
F 2 "" H 3700 3450 50  0001 C CNN
F 3 "" H 3700 3450 50  0001 C CNN
	1    3700 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 3450 3700 3350
Wire Wire Line
	4800 3000 4650 3000
Wire Wire Line
	4650 3000 4650 3050
$Comp
L power:GND #PWR0113
U 1 1 6121DBBC
P 4650 3450
F 0 "#PWR0113" H 4650 3200 50  0001 C CNN
F 1 "GND" H 4655 3277 50  0000 C CNN
F 2 "" H 4650 3450 50  0001 C CNN
F 3 "" H 4650 3450 50  0001 C CNN
	1    4650 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 3450 4650 3350
$Comp
L Connector:Conn_ARM_JTAG_SWD_10 J1
U 1 1 61221D86
P 2150 1700
F 0 "J1" H 1707 1746 50  0000 R CNN
F 1 "Conn_ARM_JTAG_SWD_10" H 1707 1655 50  0000 R CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_2x05_P1.27mm_Vertical_SMD" H 2150 1700 50  0001 C CNN
F 3 "http://infocenter.arm.com/help/topic/com.arm.doc.ddi0314h/DDI0314H_coresight_components_trm.pdf" V 1800 450 50  0001 C CNN
	1    2150 1700
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0114
U 1 1 61225A00
P 4650 2700
F 0 "#PWR0114" H 4650 2550 50  0001 C CNN
F 1 "VDD" H 4665 2873 50  0000 C CNN
F 2 "" H 4650 2700 50  0001 C CNN
F 3 "" H 4650 2700 50  0001 C CNN
	1    4650 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 2700 4650 3000
Connection ~ 4650 3000
$Comp
L power:VDD #PWR0115
U 1 1 6122F96E
P 2150 1000
F 0 "#PWR0115" H 2150 850 50  0001 C CNN
F 1 "VDD" H 2165 1173 50  0000 C CNN
F 2 "" H 2150 1000 50  0001 C CNN
F 3 "" H 2150 1000 50  0001 C CNN
	1    2150 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 1000 2150 1100
$Comp
L power:GND #PWR0116
U 1 1 61231210
P 2050 2400
F 0 "#PWR0116" H 2050 2150 50  0001 C CNN
F 1 "GND" V 2055 2272 50  0000 R CNN
F 2 "" H 2050 2400 50  0001 C CNN
F 3 "" H 2050 2400 50  0001 C CNN
	1    2050 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 2300 2050 2350
Wire Wire Line
	2050 2350 2150 2350
Wire Wire Line
	2150 2350 2150 2300
Connection ~ 2050 2350
Wire Wire Line
	2050 2350 2050 2400
Wire Wire Line
	2650 1400 3300 1400
Wire Wire Line
	2650 1600 3300 1600
Wire Wire Line
	2650 1700 3300 1700
Text Label 3300 1400 2    50   ~ 0
RESET
Text Label 7200 2000 2    50   ~ 0
RESET
Wire Wire Line
	7200 2000 6600 2000
Text Label 3300 1700 2    50   ~ 0
SWDIO
Text Label 7200 1800 2    50   ~ 0
SWDIO
Wire Wire Line
	7200 1800 6600 1800
Text Label 3300 1600 2    50   ~ 0
SWDCLK
Text Label 7200 1900 2    50   ~ 0
SWDCLK
Wire Wire Line
	7200 1900 6600 1900
Wire Wire Line
	2650 1800 3300 1800
Wire Wire Line
	6600 2300 7200 2300
Text Label 3300 1800 2    50   ~ 0
SWO
Text Label 7200 2300 2    50   ~ 0
SWO
Text Label 7200 2900 2    50   ~ 0
I2C_SCL
Text Label 7200 3000 2    50   ~ 0
I2C_SDA
Wire Wire Line
	7200 3000 6600 3000
Wire Wire Line
	6600 2900 7200 2900
Text Label 7200 2500 2    50   ~ 0
LED_G
Text Label 7200 2600 2    50   ~ 0
LED_B
$Comp
L Device:R R2
U 1 1 6125A8D7
P 4400 4900
F 0 "R2" H 4250 4850 50  0000 C CNN
F 1 "4k7" H 4250 4950 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric_Pad0.72x0.64mm_HandSolder" V 4330 4900 50  0001 C CNN
F 3 "~" H 4400 4900 50  0001 C CNN
	1    4400 4900
	-1   0    0    1   
$EndComp
$Comp
L Device:R R4
U 1 1 6125AD66
P 6700 5200
F 0 "R4" V 6600 5100 50  0000 C CNN
F 1 "100" V 6600 5350 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric_Pad0.72x0.64mm_HandSolder" V 6630 5200 50  0001 C CNN
F 3 "~" H 6700 5200 50  0001 C CNN
	1    6700 5200
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 6125AFEB
P 6700 5400
F 0 "R5" V 6600 5300 50  0000 C CNN
F 1 "100" V 6600 5550 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric_Pad0.72x0.64mm_HandSolder" V 6630 5400 50  0001 C CNN
F 3 "~" H 6700 5400 50  0001 C CNN
	1    6700 5400
	0    1    1    0   
$EndComp
Text Label 7200 2400 2    50   ~ 0
LED_R
Wire Wire Line
	7200 2400 6600 2400
Wire Wire Line
	6600 2500 7200 2500
Wire Wire Line
	7200 2600 6600 2600
$Comp
L Connector_Generic:Conn_01x01 J2
U 1 1 612245A6
P 1850 3150
F 0 "J2" H 1768 2925 50  0000 C CNN
F 1 "Conn_01x01" H 1768 3016 50  0000 C CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 1850 3150 50  0001 C CNN
F 3 "~" H 1850 3150 50  0001 C CNN
	1    1850 3150
	-1   0    0    1   
$EndComp
$Comp
L power:VDD #PWR0117
U 1 1 61226254
P 2250 3050
F 0 "#PWR0117" H 2250 2900 50  0001 C CNN
F 1 "VDD" H 2265 3223 50  0000 C CNN
F 2 "" H 2250 3050 50  0001 C CNN
F 3 "" H 2250 3050 50  0001 C CNN
	1    2250 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 3150 2250 3150
Wire Wire Line
	2250 3150 2250 3050
$Comp
L power:GND #PWR0118
U 1 1 61228B7E
P 2250 3600
F 0 "#PWR0118" H 2250 3350 50  0001 C CNN
F 1 "GND" H 2255 3427 50  0000 C CNN
F 2 "" H 2250 3600 50  0001 C CNN
F 3 "" H 2250 3600 50  0001 C CNN
	1    2250 3600
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J3
U 1 1 61228FB3
P 1850 3500
F 0 "J3" H 1768 3275 50  0000 C CNN
F 1 "Conn_01x01" H 1768 3366 50  0000 C CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 1850 3500 50  0001 C CNN
F 3 "~" H 1850 3500 50  0001 C CNN
	1    1850 3500
	-1   0    0    1   
$EndComp
Wire Wire Line
	2050 3500 2250 3500
Wire Wire Line
	2250 3500 2250 3600
$Comp
L Connector_Generic:Conn_01x01 J8
U 1 1 6122F771
P 10150 3400
F 0 "J8" H 10068 3175 50  0000 C CNN
F 1 "Conn_01x01" H 10068 3266 50  0000 C CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 10150 3400 50  0001 C CNN
F 3 "~" H 10150 3400 50  0001 C CNN
	1    10150 3400
	1    0    0    -1  
$EndComp
Text Label 8900 3400 0    50   ~ 0
ADC
$Comp
L Connector_Generic:Conn_01x01 J4
U 1 1 61232A93
P 10150 2000
F 0 "J4" H 10068 1775 50  0000 C CNN
F 1 "Conn_01x01" H 10068 1866 50  0000 C CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 10150 2000 50  0001 C CNN
F 3 "~" H 10150 2000 50  0001 C CNN
	1    10150 2000
	1    0    0    -1  
$EndComp
Text Label 8900 2000 0    50   ~ 0
D0
Wire Wire Line
	8900 2000 9950 2000
Text Label 8900 2350 0    50   ~ 0
D1
Text Label 8900 2700 0    50   ~ 0
D2
Text Label 8900 3050 0    50   ~ 0
D3
$Comp
L Connector_Generic:Conn_01x01 J5
U 1 1 61235FF7
P 10150 2350
F 0 "J5" H 10068 2125 50  0000 C CNN
F 1 "Conn_01x01" H 10068 2216 50  0000 C CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 10150 2350 50  0001 C CNN
F 3 "~" H 10150 2350 50  0001 C CNN
	1    10150 2350
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J6
U 1 1 61236618
P 10150 2700
F 0 "J6" H 10068 2475 50  0000 C CNN
F 1 "Conn_01x01" H 10068 2566 50  0000 C CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 10150 2700 50  0001 C CNN
F 3 "~" H 10150 2700 50  0001 C CNN
	1    10150 2700
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J7
U 1 1 61236B80
P 10150 3050
F 0 "J7" H 10068 2825 50  0000 C CNN
F 1 "Conn_01x01" H 10068 2916 50  0000 C CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 10150 3050 50  0001 C CNN
F 3 "~" H 10150 3050 50  0001 C CNN
	1    10150 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 3050 9950 3050
Wire Wire Line
	9950 2700 8900 2700
Wire Wire Line
	8900 2350 9950 2350
Wire Wire Line
	5200 3550 5200 4400
Wire Wire Line
	5500 4150 5300 4150
Wire Wire Line
	5300 3550 5300 4150
$Comp
L Device:R R7
U 1 1 612494E4
P 9700 3400
F 0 "R7" V 9493 3400 50  0000 C CNN
F 1 "4M" V 9584 3400 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric_Pad0.72x0.64mm_HandSolder" V 9630 3400 50  0001 C CNN
F 3 "~" H 9700 3400 50  0001 C CNN
	1    9700 3400
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 6124A3F2
P 9450 3650
F 0 "R6" H 9380 3604 50  0000 R CNN
F 1 "10M" H 9380 3695 50  0000 R CNN
F 2 "Resistor_SMD:R_0402_1005Metric_Pad0.72x0.64mm_HandSolder" V 9380 3650 50  0001 C CNN
F 3 "~" H 9450 3650 50  0001 C CNN
	1    9450 3650
	-1   0    0    1   
$EndComp
$Comp
L Device:C C7
U 1 1 6124ACEB
P 9250 3650
F 0 "C7" H 9365 3696 50  0000 L CNN
F 1 "10 n" H 9365 3605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric_Pad0.74x0.62mm_HandSolder" H 9288 3500 50  0001 C CNN
F 3 "~" H 9250 3650 50  0001 C CNN
	1    9250 3650
	-1   0    0    1   
$EndComp
Wire Wire Line
	9550 3400 9450 3400
Wire Wire Line
	9250 3400 9250 3500
Wire Wire Line
	9450 3500 9450 3400
Connection ~ 9450 3400
Wire Wire Line
	9450 3400 9250 3400
Wire Wire Line
	9850 3400 9950 3400
$Comp
L power:GND #PWR0119
U 1 1 6125331C
P 9350 3900
F 0 "#PWR0119" H 9350 3650 50  0001 C CNN
F 1 "GND" H 9355 3727 50  0000 C CNN
F 2 "" H 9350 3900 50  0001 C CNN
F 3 "" H 9350 3900 50  0001 C CNN
	1    9350 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 3800 9250 3900
Wire Wire Line
	9250 3900 9350 3900
Wire Wire Line
	9450 3800 9450 3900
Wire Wire Line
	9450 3900 9350 3900
Connection ~ 9350 3900
Wire Wire Line
	8900 3400 9250 3400
Connection ~ 9250 3400
Text Label 5600 4050 2    50   ~ 0
ADC
Wire Wire Line
	5600 4050 5400 4050
Wire Wire Line
	5400 4050 5400 3550
Text Label 6150 4000 2    50   ~ 0
D0
Text Label 6150 3900 2    50   ~ 0
D1
Text Label 6150 3800 2    50   ~ 0
D2
Text Label 6150 3700 2    50   ~ 0
D3
Wire Wire Line
	6150 3700 6000 3700
Wire Wire Line
	6000 3700 6000 3550
Wire Wire Line
	6150 3800 5900 3800
Wire Wire Line
	5900 3800 5900 3550
Wire Wire Line
	6150 3900 5800 3900
Wire Wire Line
	5800 3900 5800 3550
Wire Wire Line
	6150 4000 5700 4000
Wire Wire Line
	5700 4000 5700 3550
Text Notes 9750 3950 0    50   ~ 0
Rin = 1 / (fsample * C7)\n
$EndSCHEMATC
