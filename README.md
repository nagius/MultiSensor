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

The API support both Pull and Push model. Multiple devices can be connected to the same RS485 bus and are identified by a unique ID. This ID must be present in each request.

An ID with the broadcast value `255` will be accepted by all devices.

### Data

- To request the data on demand, send on one line :

```json
{"id": 1, "data": {}}
```

The device will respond over serial with the following message (may vary depending the option activated):

```json
{
  "id": 1,
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

Push mode: if the frequency parameter (see configuration below) is >0, the data will be sent periodically. Beware of the risk of collision if multiple devices are on the same bus.

- To activate relays, send payload with the following form. Multiple relays can be specified at the same time:

```json
{"relay0": "on", "relay2": "off"}
```

Possible values are `"on"`, `"off"` and `"toggle"`.

### Config

Configuration can be done by sending JSON payload over serial. The configuration is persisted upon power cycles.

- To get the current configuration, send on one line:

```json
{"id": 1, "config": {}}
```

This configuration is also broadcasted at boot time.

- To update it, send the same payload with modified values:

```json
{"id": 1, "config": {"freq_ms": 3000, "debug": false }}
```

`freq_ms` represent the frequency the data will be collected and sent in push mode. Set to 0 to disable.

- To change the ID of the device:

```json
{"id": 1, "config": {"id": 3 }}
```

Subsequent requests will need to reference the new ID. Messages with the wrong ID will be ignored.

- To changes the same parameters on all devices, use the broadcast ID:


```json
{"id": 255, "config": {"debug": false}}
```

## Compilation and upload

Compile this sketch with Arduino IDE and select board `Arduino Uno` or `Atmel atmega328p` for compatible boards.

## License

Copyleft 2024-2026 - Nicolas AGIUS - GNU GPLv3
