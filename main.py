import network
from machine import UART, Pin, SoftI2C
from lcd_api import LcdApi
from i2c_lcd import I2cLcd
import uasyncio as asyncio
import usocket as socket
import time

# Este codigo envia solo una vez y no constantemente el contenido del puerto
def connect_wifi(ssid, password, lcd):
	wlan = network.WLAN(network.STA_IF)
	wlan.active(True)
	wlan.connect(ssid, password)
	while not wlan.isconnected():
			pass
	msg_conect = 'WiFi Connect IP:'
	msg_ipaddr =  str(wlan.ifconfig()[0])
	print(msg_conect)
	print(msg_ipaddr)
	lcdprint(lcd,0,0,msg_conect,True)
	lcdprint(lcd,1,0,msg_ipaddr,False)
	time.sleep(2)

def configure_LED():
	GPIO32_LED = 32
	led_pin32 = Pin(GPIO32_LED, Pin.OUT)
	return led_pin32

def configure_LCD_16x2():
	# Parametros del LCD 16x2
	I2C_ADDR = 0X27
	TOTAL_ROWS = 2
	TOTAL_COLS = 16

	SCL_GPIO_22 = Pin(22)
	SDA_GPIO_21 = Pin(21)
	FREQ = 10000

	i2c = SoftI2C(scl=SCL_GPIO_22, sda=SDA_GPIO_21, freq=FREQ)
	lcd = I2cLcd(i2c, I2C_ADDR, TOTAL_ROWS, TOTAL_COLS)
	# lcd.backlight_off()
	lcd.backlight_on()
	lcd.clear()
	return lcd

def ledReady(led_pin, activo):
	led_pin.value(activo)


def lcdprint(lcd, line, col, message, isclear=False):
	if(isclear):
		lcd.clear()
	lcd.move_to( col,line)
	lcd.putstr(message)

def configure_uart(tx_pin, rx_pin, baudrate=9600):
	uart = UART(1, baudrate=baudrate, tx=tx_pin, rx=rx_pin)
	return uart

def configure_websocket(uri, port):
	addr_info = socket.getaddrinfo(uri, port)
	addr = addr_info[0][-1]
	s = socket.socket()
	s.connect(addr)
	s.send(b'GET / HTTP/1.1\r\nHost: %s\r\n\r\n' % uri)
	return s

def verific_end(data,lcd):
	retvalue = False
	if(data.decode()=='END'):
		lcdprint(lcd, 0, 2, '==> END <==', True)
		lcdprint(lcd, 1, 0, '[RESET] iniciar', False)
		retvalue = True
	return retvalue

# async def websocket_send(uri, port, message):
async def websocket_send(s, message):
	s.send(message)
	print("Send Data: ", message.decode())

# async def read_and_send(uart, websocket_uri, port_uri):
async def read_and_send(uart, websocket, lcd):
	while True:
		if uart.any():
			data = uart.read()
			if(verific_end(data,lcd)):
				break
			datalcd ='TARA: '+ data.decode() + '     '
			lcdprint(lcd, 0, 0, datalcd, False)
			print("Data UART:", data)
			await websocket_send(websocket, data)
		await asyncio.sleep(1)

# async def main():
async def main():
	SSID = 'JJLOSROQUES'
	PASSWORD = 'pedro10292569carlos27174055'
	TX_GPIO17 = 17
	RX_GPIO16 = 16

	WEBSOCKET_URI = '192.168.1.103'
	PORT_URI = 1234
	
	pinled = configure_LED()
	lcd = configure_LCD_16x2()
	connect_wifi(SSID, PASSWORD,lcd)
	uart = configure_uart(tx_pin=TX_GPIO17, rx_pin=RX_GPIO16)  # Ajusta los pines TX y RX según tu configuración
	sockt = configure_websocket(WEBSOCKET_URI,PORT_URI)
	

	lcdprint(lcd,0,0,"Ready...",True)
	time.sleep(1)
	ledReady(pinled,1)
	await read_and_send(uart, sockt, lcd)
	ledReady(pinled,0)


# Ejecuta la función principal
asyncio.run(main())


print('\n-----------------------')
print('|  Fin de aplicacion  |')
print('-----------------------')