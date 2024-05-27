# Cliente Socket para ESP32 - RS232 y LCD 16x2

Una sencilla aplicacion que se conecta a una red WiFi, abriendo el puerto RS232 a TTL para leer datos y ser enviados mediante socket a un servidor de socket que esta escuchando, tambien muestra los datos leidos en una pantalla LCD, con la finaldad que el usuario verifique los datos.

Microcontrolador: _ESP32 DevkitC_v4_

Programa escrito en micropython, con ayuda de chatGPT

## Conexion Pines de LCD 16x2 HD44780 a ESP32

```
  VCC => 5V
  SDA => GPIO21
  SLC => GPIO22
  GND => GND
```

## Conexion Pines de RS232 a ESP32

```
  VCC => 3V
  RXD => GPIO16
  TXD => GPIO17
  GND => GND
```

## Conexion Pines del LED a ESP32

```
  ANODO  => GPIO32
  CATODO => GND
```

## Conexion Pines TOUCH a ESP32

```
  RESET  => GPIO4
```
