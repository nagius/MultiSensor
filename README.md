# MultiSensor

## Concept

MultiSensor is an Arduino sketch to collect data from multiple sensors and send the values over serial port.

It has been desiged with low memory footprint to fit on Aruino Uno and Nano.

## Features

MultiSensor can collect data from multiple sensors : 
 - 2x ultrasonic distance sensors type AJ-SR04M in Low Power Serial mode (R19 = 47k)
 - 1x temperature sensors DS18B20
 - 1x pulse sensor like flow sensor YF-B5

Each of them is optional and can be selected in the source code.

It can also control up to 4 standard relays as output.

Data and control commands are send as JSON over standard RS232 Serial or RS485 with the help of a MAX485 adapter.

## API

Data is sent periodically over serial as followming (may vary depending the option activated):

```json
{
  "data": {
    "flow": 1875,       # Counter of flow sensor pulses
    "temp": 23.5,       # Temperature (Celcius)
    "A": 0,             # Distance from sensor A (mm)
    "B": 0,             # Distance from sensor B (mm)
    "relay0": false,    # Relay activated
    "relay1": false,
    "relay2": false,
    "relay3": false
  }
}
```

Configuration can be done by sending JSON payload over serial. To get the current configuration, send on one line:

```json
{"config": {}}
```

To update it, send the same payload with modified values:

```json
{"config": {"frequency": 3000, "debug": false }}
```

Frequency (ms) represent how often the data will be collected and sent. Changes are not persistent on restart.

To activate relays, send payload with the following form. Multiple relays can be specified at the same time:

```json
{"relay0": "on", "relay2": "off"}
```

Possible values are `"on"`, `"off"` and `"toggle"`.

## Compilation and upload

Compile this sketch with Arduino IDE and select board `Arduino Uno` or `Atmel atmega328p` for compatible boards.

## License

Copyleft 2024 - Nicolas AGIUS - GNU GPLv3
