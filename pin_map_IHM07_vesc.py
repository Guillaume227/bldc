#IHM07/vesc

pin_map = \
#IHM08 / IHM07 / VESC
'''
PC3      PC3 IN13 VOLTAGE1    IN0 PA0
PC4 IN14 PB0 IN8  VOLTAGE2    IN1 PA1   
PC5 IN15 PA7 IN7  VOLTAGE3    IN2 PA2   
PC2      PC2 IN12 ADC_TEMP    IN3 PA3
?        ?   IN14 TEMP_MOTOR  IN14	PC4 
?        ?        ADC_EXT3	  IN15	PC5 
PA1      PA1 IN1  VBUS_SENSING/AN_IN IN13 PC3

PA0 PA0 IN0  CURRENT1 IN10 PC0   
PC1 PC1 IN11 CURRENT2 IN11 PC1   
PC0 PC0 IN10 CURRENT3 IN12 PC2   

PC9 PC9 GPIO_BEMF[Connect to GND] PC9

PA4 IN4 PB1 IN9 POTENTIOMETER IN9 PB1

? Vrefint IN2 ?
PB4 CURRENT_REF ?

PC13 PC13 Blue Button ?

(PA4) PA4 DAC_ch ?
PA5 PA5 GPIO/DAC/PWM ?
PA12 CPOUT ? 

PA8  PA8  H1 PA8
PA9  PA9  H2 PA9
PA10 PA10 H3 PA10

PA7 PC10 UL/L1 PB13
PB0 PC11 VL/L2 PB14
PB1 PC12 WL/L3 PB15

? USB_DM PA11
? USB_DP PA12

PC6 (USART6?)(UART_TX_PIN = TX_SCL_MOSI)(USART3?) PB10 
PC7 (USART6?)(UART_RX_PIN = RX_SDA_NSS) (USART3?) PB11 

?   ?   LED_GREEN PB0
PB2 PB2 LED_RED   PB2

? ENABLE_GATE PB5

? ICU (for servo decoding) PB6

? I2C PB10
? I2C PB11

PA15 PA15 HALL_ENC1 PC7
PB3  PB3  HALL_ENC2 PC8
PB10 PB10 HALL_ENC3 PC6


PA6|PA11|PB14 x  BKIN ?
x PA6|PB14  BKIN1 ?
X PA11 BKIN2 ?

? NRF PB12
? NRF PB4
? NRF PB3
? NRF PD2

? SPI PA4
? SPI PA5
? SPI PA7
? SPI PA6
'''


if __name__ == '__main__':
	
	pins = {}
	errors = set()
	for line in pin_map.splitlines():
		if line:
			pin, vesc_affectation = line.split(None, 1)
			if pin in pins:
				errors.add(pin)
				print(f"ERROR: pin taken:{pin}")
				print(f"\t-{vesc_affectation}")
				print(f"\t-{pins[pin]}")
			elif pin != '?':
				pins[pin] = vesc_affectation

	for pin in sorted(pins):
		print(pin)
	
	if errors:
		print(f"{len(errors)} duplicate pin allocation found")
	else:
		print("Pin allocation looks good")