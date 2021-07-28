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
L spv:SPV1050 U1
U 1 1 60F3D450
P 7750 2900
F 0 "U1" H 7750 4037 60  0000 C CNN
F 1 "SPV1050" H 7750 3931 60  0000 C CNN
F 2 "Package_DFN_QFN:QFN-20-1EP_3x3mm_P0.4mm_EP1.65x1.65mm" H 7750 3931 60  0001 C CNN
F 3 "" H 7750 2900 60  0000 C CNN
	1    7750 2900
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J2
U 1 1 60F42181
P 6050 2150
F 0 "J2" H 6100 2557 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 6100 2466 50  0001 C CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_2x03_P1.27mm_Vertical_SMD" H 6050 2150 50  0001 C CNN
F 3 "~" H 6050 2150 50  0001 C CNN
F 4 "1-3 & 2-4: Buck-Boost, 1-2 & 3-4 & 5-6: Boost" H 6350 2450 50  0000 C CNN "config"
	1    6050 2150
	1    0    0    -1  
$EndComp
$Comp
L Device:L L1
U 1 1 60F45295
P 6600 2050
F 0 "L1" V 6700 1900 50  0000 C CNN
F 1 "22u" V 6699 2050 50  0000 C CNN
F 2 "we-lqs:WE-LQS" H 6600 2050 50  0001 C CNN
F 3 "~" H 6600 2050 50  0001 C CNN
	1    6600 2050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6750 2050 6850 2050
Wire Wire Line
	6450 2050 6350 2050
Wire Wire Line
	6350 2150 6850 2150
Wire Wire Line
	6850 2250 6400 2250
$Comp
L power:GND #PWR0101
U 1 1 60F4B8E6
P 5800 2300
F 0 "#PWR0101" H 5800 2050 50  0001 C CNN
F 1 "GND" H 5805 2127 50  0000 C CNN
F 2 "" H 5800 2300 50  0001 C CNN
F 3 "" H 5800 2300 50  0001 C CNN
	1    5800 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 2300 5800 2250
Wire Wire Line
	5800 2250 5850 2250
Wire Wire Line
	6400 2250 6400 2450
Wire Wire Line
	6400 2450 5650 2450
Wire Wire Line
	5650 2450 5650 2150
Wire Wire Line
	5650 2150 5850 2150
Connection ~ 6400 2250
Wire Wire Line
	6400 2250 6350 2250
$Comp
L Device:R R1
U 1 1 60F4E221
P 5300 2350
F 0 "R1" H 5370 2396 50  0000 L CNN
F 1 "10M" H 5370 2305 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric_Pad0.72x0.64mm_HandSolder" V 5230 2350 50  0001 C CNN
F 3 "~" H 5300 2350 50  0001 C CNN
	1    5300 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 2600 5300 2550
Wire Wire Line
	5300 3000 5300 2950
$Comp
L power:VBUS #PWR0102
U 1 1 60F59E50
P 4950 1750
F 0 "#PWR0102" H 4950 1600 50  0001 C CNN
F 1 "VBUS" H 4965 1923 50  0000 C CNN
F 2 "" H 4950 1750 50  0001 C CNN
F 3 "" H 4950 1750 50  0001 C CNN
	1    4950 1750
	1    0    0    -1  
$EndComp
Connection ~ 5300 2550
Wire Wire Line
	5300 2550 5300 2500
Connection ~ 5300 2950
Wire Wire Line
	5300 2950 5300 2900
Wire Wire Line
	5300 2550 6850 2550
Wire Wire Line
	5300 2950 6850 2950
$Comp
L power:GND #PWR0103
U 1 1 60F5FDF8
P 5300 3400
F 0 "#PWR0103" H 5300 3150 50  0001 C CNN
F 1 "GND" H 5305 3227 50  0000 C CNN
F 2 "" H 5300 3400 50  0001 C CNN
F 3 "" H 5300 3400 50  0001 C CNN
	1    5300 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 3400 5300 3350
$Comp
L Device:C C2
U 1 1 60F608BC
P 6100 3550
F 0 "C2" H 6215 3596 50  0000 L CNN
F 1 "10n" H 6215 3505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric_Pad0.74x0.62mm_HandSolder" H 6138 3400 50  0001 C CNN
F 3 "~" H 6100 3550 50  0001 C CNN
	1    6100 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 3350 6100 3350
Wire Wire Line
	6100 3350 6100 3400
$Comp
L power:GND #PWR0104
U 1 1 60F6229F
P 6100 3750
F 0 "#PWR0104" H 6100 3500 50  0001 C CNN
F 1 "GND" H 6105 3577 50  0000 C CNN
F 2 "" H 6100 3750 50  0001 C CNN
F 3 "" H 6100 3750 50  0001 C CNN
	1    6100 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 3750 6100 3700
$Comp
L power:GND #PWR0105
U 1 1 60F62B4C
P 6650 3500
F 0 "#PWR0105" H 6650 3250 50  0001 C CNN
F 1 "GND" V 6655 3372 50  0000 R CNN
F 2 "" H 6650 3500 50  0001 C CNN
F 3 "" H 6650 3500 50  0001 C CNN
	1    6650 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	6850 3550 6750 3550
Wire Wire Line
	6850 3450 6750 3450
Wire Wire Line
	6750 3450 6750 3500
$Comp
L Device:Jumper_NC_Dual JP1
U 1 1 60F64B5A
P 6500 4050
F 0 "JP1" V 6591 4152 50  0000 L CNN
F 1 "Jumper_NC_Dual" V 6500 4152 50  0000 L CNN
F 2 "Jumper:SolderJumper-3_P1.3mm_Open_RoundedPad1.0x1.5mm_NumberLabels" H 6500 4050 50  0001 C CNN
F 3 "~" H 6500 4050 50  0001 C CNN
F 4 "1-2: Buck-Boost, 2-3: Boost" V 6409 4152 50  0000 L CNN "config"
	1    6500 4050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6750 3500 6650 3500
Connection ~ 6750 3500
Wire Wire Line
	6750 3500 6750 3550
$Comp
L power:VBUS #PWR0106
U 1 1 60F6E31F
P 6500 3800
F 0 "#PWR0106" H 6500 3650 50  0001 C CNN
F 1 "VBUS" H 6515 3973 50  0000 C CNN
F 2 "" H 6500 3800 50  0001 C CNN
F 3 "" H 6500 3800 50  0001 C CNN
	1    6500 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 60F6EAAB
P 6500 4300
F 0 "#PWR0107" H 6500 4050 50  0001 C CNN
F 1 "GND" H 6505 4127 50  0000 C CNN
F 2 "" H 6500 4300 50  0001 C CNN
F 3 "" H 6500 4300 50  0001 C CNN
	1    6500 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 3650 6750 3650
Wire Wire Line
	6750 3650 6750 4050
Wire Wire Line
	6750 4050 6600 4050
$Comp
L Device:C C1
U 1 1 60F702B4
P 4950 2750
F 0 "C1" H 5065 2796 50  0000 L CNN
F 1 "4u7" H 5065 2705 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 4988 2600 50  0001 C CNN
F 3 "~" H 4950 2750 50  0001 C CNN
	1    4950 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 3350 4950 3350
Wire Wire Line
	4950 3350 4950 2900
Connection ~ 5300 3350
Wire Wire Line
	5300 3350 5300 3300
Wire Wire Line
	4950 1750 4950 2050
Wire Wire Line
	5850 2050 5300 2050
Connection ~ 4950 2050
Wire Wire Line
	4950 2050 4950 2600
Wire Wire Line
	5300 2200 5300 2050
Connection ~ 5300 2050
Wire Wire Line
	5300 2050 4950 2050
Wire Wire Line
	8650 2150 9100 2150
Wire Wire Line
	9100 2150 9100 2200
Wire Wire Line
	8650 2550 9100 2550
Wire Wire Line
	9100 2550 9100 2500
Wire Wire Line
	9100 2600 9100 2550
Connection ~ 9100 2550
Wire Wire Line
	8650 2950 9100 2950
Wire Wire Line
	9100 2950 9100 2900
Wire Wire Line
	9100 3000 9100 2950
Connection ~ 9100 2950
$Comp
L power:GND #PWR0108
U 1 1 60F7C78A
P 9100 3400
F 0 "#PWR0108" H 9100 3150 50  0001 C CNN
F 1 "GND" H 9105 3227 50  0000 C CNN
F 2 "" H 9100 3400 50  0001 C CNN
F 3 "" H 9100 3400 50  0001 C CNN
	1    9100 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 3400 9100 3350
$Comp
L Device:C C3
U 1 1 60F7DC49
P 9500 2750
F 0 "C3" H 9615 2796 50  0000 L CNN
F 1 "C" H 9615 2705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9538 2600 50  0001 C CNN
F 3 "~" H 9500 2750 50  0001 C CNN
	1    9500 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 2150 9500 2150
Wire Wire Line
	9500 2150 9500 2600
Connection ~ 9100 2150
Wire Wire Line
	9100 3350 9500 3350
Wire Wire Line
	9500 3350 9500 2900
Connection ~ 9100 3350
Wire Wire Line
	9100 3350 9100 3300
$Comp
L Connector_Generic:Conn_01x01 J12
U 1 1 60F82671
P 10100 2050
F 0 "J12" H 10180 2046 50  0000 L CNN
F 1 "Conn_01x01" H 10180 2001 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 10100 2050 50  0001 C CNN
F 3 "~" H 10100 2050 50  0001 C CNN
	1    10100 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 2050 9100 2050
$Comp
L power:+BATT #PWR0109
U 1 1 60F86719
P 9100 1700
F 0 "#PWR0109" H 9100 1550 50  0001 C CNN
F 1 "+BATT" H 9115 1873 50  0000 C CNN
F 2 "" H 9100 1700 50  0001 C CNN
F 3 "" H 9100 1700 50  0001 C CNN
	1    9100 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 1700 9100 2050
Connection ~ 9100 2050
Wire Wire Line
	9100 2050 8650 2050
$Comp
L Connector_Generic:Conn_01x01 J1
U 1 1 60F8908A
P 4600 2050
F 0 "J1" H 4518 1825 50  0000 C CNN
F 1 "Conn_01x01" H 4518 1916 50  0000 C CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 4600 2050 50  0001 C CNN
F 3 "~" H 4600 2050 50  0001 C CNN
	1    4600 2050
	-1   0    0    1   
$EndComp
Wire Wire Line
	4950 2050 4800 2050
$Comp
L Connector_Generic:Conn_01x01 J3
U 1 1 60F8CDA2
P 10300 3550
F 0 "J3" H 10380 3546 50  0000 L CNN
F 1 "Conn_01x01" H 10380 3501 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 10300 3550 50  0001 C CNN
F 3 "~" H 10300 3550 50  0001 C CNN
	1    10300 3550
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J4
U 1 1 60F8D358
P 8850 3150
F 0 "J4" H 8930 3146 50  0000 L CNN
F 1 "Conn_01x01" H 8930 3101 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 8850 3150 50  0001 C CNN
F 3 "~" H 8850 3150 50  0001 C CNN
	1    8850 3150
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J5
U 1 1 60F8D5EE
P 10300 4400
F 0 "J5" H 10380 4396 50  0000 L CNN
F 1 "Conn_01x01" H 10380 4351 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 10300 4400 50  0001 C CNN
F 3 "~" H 10300 4400 50  0001 C CNN
	1    10300 4400
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J6
U 1 1 60F8D82C
P 8850 3350
F 0 "J6" H 8930 3346 50  0000 L CNN
F 1 "Conn_01x01" H 8930 3301 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 8850 3350 50  0001 C CNN
F 3 "~" H 8850 3350 50  0001 C CNN
	1    8850 3350
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J7
U 1 1 60F8DA22
P 8850 3650
F 0 "J7" H 8930 3646 50  0000 L CNN
F 1 "Conn_01x01" H 8930 3601 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 8850 3650 50  0001 C CNN
F 3 "~" H 8850 3650 50  0001 C CNN
	1    8850 3650
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J8
U 1 1 60F8DF1C
P 8850 3750
F 0 "J8" H 8930 3746 50  0000 L CNN
F 1 "Conn_01x01" H 8930 3701 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 8850 3750 50  0001 C CNN
F 3 "~" H 8850 3750 50  0001 C CNN
	1    8850 3750
	1    0    0    -1  
$EndComp
NoConn ~ 6850 3750
$Comp
L Device:R R2
U 1 1 60F8F433
P 5300 2750
F 0 "R2" H 5370 2796 50  0000 L CNN
F 1 "0M59" H 5370 2705 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric_Pad0.72x0.64mm_HandSolder" V 5230 2750 50  0001 C CNN
F 3 "~" H 5300 2750 50  0001 C CNN
	1    5300 2750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 60F8F87A
P 5300 3150
F 0 "R3" H 5370 3196 50  0000 L CNN
F 1 "2M49" H 5370 3105 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric_Pad0.72x0.64mm_HandSolder" V 5230 3150 50  0001 C CNN
F 3 "~" H 5300 3150 50  0001 C CNN
	1    5300 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 60F8FBD4
P 9100 2350
F 0 "R4" H 9170 2396 50  0000 L CNN
F 1 "10M" H 9170 2305 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric_Pad0.72x0.64mm_HandSolder" V 9030 2350 50  0001 C CNN
F 3 "~" H 9100 2350 50  0001 C CNN
	1    9100 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 60F902E5
P 9100 2750
F 0 "R5" H 9170 2796 50  0000 L CNN
F 1 "1M2" H 9170 2705 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric_Pad0.72x0.64mm_HandSolder" V 9030 2750 50  0001 C CNN
F 3 "~" H 9100 2750 50  0001 C CNN
	1    9100 2750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 60F90655
P 9100 3150
F 0 "R6" H 9170 3196 50  0000 L CNN
F 1 "4M7" H 9170 3105 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric_Pad0.72x0.64mm_HandSolder" V 9030 3150 50  0001 C CNN
F 3 "~" H 9100 3150 50  0001 C CNN
	1    9100 3150
	1    0    0    -1  
$EndComp
Text Notes 3750 2600 0    50   ~ 0
SM141K10L\nVmpp = 5.6 V
Text Notes 3750 2350 0    50   ~ 0
SM141K10L\nVmpp = 5.6 V
$Comp
L Connector_Generic:Conn_01x01 J9
U 1 1 60F9D140
P 8850 4050
F 0 "J9" H 8930 4046 50  0000 L CNN
F 1 "Conn_01x01" H 8930 4001 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 8850 4050 50  0001 C CNN
F 3 "~" H 8850 4050 50  0001 C CNN
	1    8850 4050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J10
U 1 1 60F9D412
P 8850 4200
F 0 "J10" H 8930 4196 50  0000 L CNN
F 1 "Conn_01x01" H 8930 4151 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 8850 4200 50  0001 C CNN
F 3 "~" H 8850 4200 50  0001 C CNN
	1    8850 4200
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J11
U 1 1 60F9D659
P 8850 4350
F 0 "J11" H 8930 4346 50  0000 L CNN
F 1 "Conn_01x01" H 8930 4301 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 8850 4350 50  0001 C CNN
F 3 "~" H 8850 4350 50  0001 C CNN
	1    8850 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 60F9D8E0
P 8550 4450
F 0 "#PWR0110" H 8550 4200 50  0001 C CNN
F 1 "GND" H 8555 4277 50  0000 C CNN
F 2 "" H 8550 4450 50  0001 C CNN
F 3 "" H 8550 4450 50  0001 C CNN
	1    8550 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 4050 8550 4050
Wire Wire Line
	8550 4050 8550 4200
Wire Wire Line
	8650 4350 8550 4350
Connection ~ 8550 4350
Wire Wire Line
	8550 4350 8550 4450
Wire Wire Line
	8650 4200 8550 4200
Connection ~ 8550 4200
Wire Wire Line
	8550 4200 8550 4350
$Comp
L Device:C C4
U 1 1 60FA43C7
P 9850 2750
F 0 "C4" H 9965 2796 50  0000 L CNN
F 1 "C" H 9965 2705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 9888 2600 50  0001 C CNN
F 3 "~" H 9850 2750 50  0001 C CNN
	1    9850 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 2600 9850 2150
Wire Wire Line
	9850 2150 9500 2150
Connection ~ 9500 2150
Wire Wire Line
	9500 3350 9850 3350
Wire Wire Line
	9850 3350 9850 2900
Connection ~ 9500 3350
$Comp
L Connector_Generic:Conn_01x01 J13
U 1 1 60FCC66D
P 4600 3350
F 0 "J13" H 4518 3125 50  0000 C CNN
F 1 "Conn_01x01" H 4518 3216 50  0000 C CNN
F 2 "TestPoint:TestPoint_Pad_D2.0mm" H 4600 3350 50  0001 C CNN
F 3 "~" H 4600 3350 50  0001 C CNN
	1    4600 3350
	-1   0    0    1   
$EndComp
Wire Wire Line
	4800 3350 4950 3350
Connection ~ 4950 3350
Text Label 8900 3050 2    50   ~ 0
LDO2
Wire Wire Line
	8900 3050 8650 3050
Text Label 8900 3250 2    50   ~ 0
LDO1
Wire Wire Line
	8900 3250 8650 3250
$Comp
L Device:C C5
U 1 1 60FE19C6
P 9950 3750
F 0 "C5" H 10065 3796 50  0000 L CNN
F 1 "100n" H 10065 3705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric_Pad0.74x0.62mm_HandSolder" H 9988 3600 50  0001 C CNN
F 3 "~" H 9950 3750 50  0001 C CNN
	1    9950 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 3550 9950 3550
Wire Wire Line
	9950 3550 9950 3600
$Comp
L power:GND #PWR0111
U 1 1 60FE41AC
P 9950 4000
F 0 "#PWR0111" H 9950 3750 50  0001 C CNN
F 1 "GND" H 9955 3827 50  0000 C CNN
F 2 "" H 9950 4000 50  0001 C CNN
F 3 "" H 9950 4000 50  0001 C CNN
	1    9950 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9950 4000 9950 3900
Text Label 9750 3550 0    50   ~ 0
LDO1
Wire Wire Line
	9750 3550 9950 3550
Connection ~ 9950 3550
$Comp
L Device:C C6
U 1 1 60FE744C
P 9950 4600
F 0 "C6" H 10065 4646 50  0000 L CNN
F 1 "100n" H 10065 4555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric_Pad0.74x0.62mm_HandSolder" H 9988 4450 50  0001 C CNN
F 3 "~" H 9950 4600 50  0001 C CNN
	1    9950 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 4400 9950 4400
Wire Wire Line
	9950 4400 9950 4450
Wire Wire Line
	9950 4400 9750 4400
Connection ~ 9950 4400
Text Label 9750 4400 0    50   ~ 0
LDO2
$Comp
L power:GND #PWR0112
U 1 1 60FEB499
P 9950 4850
F 0 "#PWR0112" H 9950 4600 50  0001 C CNN
F 1 "GND" H 9955 4677 50  0000 C CNN
F 2 "" H 9950 4850 50  0001 C CNN
F 3 "" H 9950 4850 50  0001 C CNN
	1    9950 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9950 4850 9950 4750
Text Notes 9500 1750 0    50   ~ 0
Vuvp = 3.32 V\nVeoc = 4.16 V
$EndSCHEMATC
