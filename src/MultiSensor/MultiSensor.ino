/*************************************************************************
 *
 * This file is part of the MultiSensor Arduino sketch.
 * Copyleft 2024 Nicolas Agius <nicolas.agius@lps-it.fr>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * ***********************************************************************/

// Supported sensors:
// - 2x Distance AJ-SR04M See https://tutorials.probots.co.in/communicating-with-a-waterproof-ultrasonic-sensor-aj-sr04m-jsn-sr04t/
// - 1x Temperature DS18B20
// - 1x Flow YF-B5
//
// Supported outputs:
// - JSON Serial API
// - 4x standard relays

// This code has beed desiged to run on Arduino Uno/Nano with limited RAM,
// hence the RAM usage optimisation avoiding Strings as both DallasTemperature
// and AduinoJson uses a lot of RAM.
//
// Use board "Atmel atmega328p (old bootloader)" to compile for compatible boards

// MAX485 Setup:
//   - TX -> DI
//   - RX -> RO
//   - GPIO_PTT -> RE and DE
// See https://www.circuitstate.com/tutorials/what-is-rs-485-how-to-use-max485-with-arduino-for-reliable-long-distance-serial-communication/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <util/atomic.h>

//#include "MemoryFree.h"

// Feature configuration (comment out unneeded features)
#define HAS_FLOW_SENSOR   // Water flow sensor YF-B5 (5v)
#define HAS_DS18B20       // Temperature Dallas sensor
#define HAS_NB_RELAYS 4   // Number of relays (1-4), comment out to disable relays
#define HAS_SENSOR_A      // AJ-SR04M
#define HAS_SENSOR_B      // AJ-SR04M
#define HAS_MAX485        // Use MAX485 on hardware serial

// GPIO configuration
#define GPIO_ONEWIRE 12     // OneWire pin bus
#define GPIO_FLOW_SENSOR 3  // Hall effect flow sensor YF-B5
#define GPIO_SENSOR_A_RX 2  // AJ-SR04M in Low Power Serial mode (R19 = 47k)
#define GPIO_SENSOR_A_TX 4
#define GPIO_SENSOR_B_RX 7
#define GPIO_SENSOR_B_TX 8
#define GPIO_RELAY0 6      // Relays GPIO
#define GPIO_RELAY1 11
#define GPIO_RELAY2 10
#define GPIO_RELAY3 9
#define GPIO_PTT 13        // Connect to RE and DE on MAX485
#define DEFAULT_FREQUENCY_MS 60000 // Frequency of the execution in ms
#define BROADCAST_ID 0xFF // Generic ID to address all devices
#define DEFAULT_ID 0x01 // Unique ID for multi-device communication

#define ONEWIRE_ADDR_LEN 16        // 6 bytes + 3 chars header + EOS = 16 chars
#define BUF_SIZE 256               // Used for string buffers

// Debug macro
#define DBG(...) if(debug) { snprintf_P(buffer, BUF_SIZE, __VA_ARGS__); println(buffer); }

unsigned long lastrun_ms = 0;
unsigned long frequency_ms = DEFAULT_FREQUENCY_MS;
uint8_t id = DEFAULT_ID;
bool debug = false;
char buffer[BUF_SIZE];            // Global char* to avoir multiple String concatenation which causes RAM fragmentation

StaticJsonDocument<200> json_input;


/**
 * Serial JSON API handlers
 ********************************************************************************/

// Require F("message") as first parameter
void json_error(const __FlashStringHelper *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);

  vsnprintf_P(buffer, BUF_SIZE, (const char*)fmt, ap);

  ptt_push();
  Serial.print(F("{\"error\": \""));
  Serial.print(buffer);
  Serial.println(F("\"}"));
  ptt_release();

  va_end(ap);
}

void print_help()
{
  ptt_push();
  Serial.println(F(R"(
MultiSensor v2.0 JSON API:
 {"id": X, "config": {}} : Request config
 {"id": X, "data": {}} : Request data
 {"id": X, "config": {"freq_ms": 2000}} : Set frequency in ms (0 to disable broadcast)
 {"id": X, "config": {"debug": true}} : Enable debug mode
 {"id": X, "config": {"id": 2}} : Change ID number
 {"id": X, "relayN": "on"} : Switch X on
 {"id": X, "relayN": "off"} : Switch X off
 {"id": X, "relayN": "toggle"} : Toggle switch X
  )"));
  ptt_release();
}

void send_json_config()
{
  snprintf_P(buffer, BUF_SIZE, PSTR("{\"id\": %u, \"config\": {\"freq_ms\": %lu, \"debug\": %s }}"), id, frequency_ms, debug ? "true": "false");
  println(buffer);
}

bool save_json_config(JsonObject config)
{
  // Save debug
  if(config.containsKey(F("debug")))
  {
    debug = config[F("debug")];
    DBG(PSTR("Saved debug=%s"), debug ? "true": "false");
  }

  if(config.containsKey(F("freq_ms")))
  {
    // Save frequency
    const int freq = config[F("freq_ms")];

    if(freq < 0)
    {
      json_error(F("Invalid freq_ms parameter: need to be positive integer or zero"));
      return false;
    }
    else
    {
      frequency_ms = freq;
      DBG(PSTR("Saved freq_ms=%i"), frequency_ms);
    }
  }

  if(config.containsKey(F("id")))
  {
    const int new_id = config[F("id")];

    if(new_id < 0 || new_id >= 255) // 0xFF is broadcast value
    {
      json_error(F("Invalid ID parameter: need to be positive short integer"));
      return false;
    }
    else
    {
      id = new_id;
      DBG(PSTR("Saved ID=%i"), id);
    }
  }

  return true;
}


void handle_serial_api()
{
  if(Serial.available())
  {
    uint8_t size = Serial.readBytesUntil('\n', buffer, BUF_SIZE-1);
    buffer[size] = '\0'; // Terminate input string

    if(strcmp_P(buffer, PSTR("help")) == 0)
    {
      print_help();
      return;
    }
    
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(json_input, buffer);

     // Test if parsing succeeds.
    if(error)
    {
      json_error(F("deserializeJson() failed: %s"), error.c_str());
      print_help();
      return;
    }

    // Manage identification
    if(json_input.containsKey(F("id")))
    {
      const int received_id = json_input[F("id")];
      if(received_id != id && received_id != BROADCAST_ID)
      {
        // Message not for us
        return;
      }
    }
    else
    {
      // Invalid message
      return;
    }

    // Manage configuration
    if(json_input.containsKey(F("config")))
    {
      if(json_input[F("config")].isNull() || json_input[F("config")].size() == 0)
      {
        send_json_config();
      }
      else
      {
        if(save_json_config(json_input[F("config")]))
        {
          send_json_config();
        }
      }
      return;
    }

    // Manage data
    if(json_input.containsKey(F("data")))
    {
      if(json_input[F("data")].isNull() || json_input[F("data")].size() == 0)
      {
        send_sensors_json_data();
      }
      return;
    }

    // Manage relays
#ifdef HAS_NB_RELAYS
    for(uint8_t i=0; i < HAS_NB_RELAYS; i++)
    {
      if(json_input.containsKey(get_relay_label(i)))
      {
        const char* action = json_input[get_relay_label(i)];

        if(strcmp_P(action, PSTR("on")) == 0)
        {
          switch_relay(i, true);
        }
        else if(strcmp_P(action, PSTR("off")) == 0)
        {
          switch_relay(i, false);
        }
        else if(strcmp_P(action, PSTR("toggle")) == 0)
        {
          switch_relay(i, !is_relay_active(i));
        }
        else
        {
          json_error(F("Unknown action: %s"), action);
        }
      }
    }
#endif
  }
}

void setup()
{
  Serial.begin(9600);  // Default 8N1
  
#ifdef HAS_SENSOR_A
  setup_distance_A();
#endif

#ifdef HAS_SENSOR_B
  setup_distance_B();
#endif

#ifdef HAS_DS18B20
  setup_DS18B20();
#endif

#ifdef HAS_MAX485
  setup_MAX485();
#endif

#ifdef HAS_NB_RELAYS
  setup_relays();
#endif

#ifdef HAS_FLOW_SENSOR
  setup_flow_counter();
#endif

  ptt_push();
  Serial.println(F("MultiSensor v2.0 started."));
  ptt_release();

  // Broadcast config at boot
  send_json_config();
  
  // Initial broadcast
  if(frequency_ms>0)
    send_sensors_json_data();
}

void loop()
{
  handle_serial_api();
  
  if(frequency_ms>0)
  {
    unsigned long now_ms = millis();
    if(now_ms - lastrun_ms > frequency_ms)
    {
      send_sensors_json_data();

      //Serial.print("freeMemory()=");
      //Serial.println(freeMemory());
      
      // update the timing variable
      lastrun_ms = now_ms;
    }
  }  
}

void send_sensors_json_data()
{
  unsigned int len = 0;
  char output[BUF_SIZE];

  len += snprintf_P(output, BUF_SIZE, PSTR("{\"id\": %u, \"data\":{"), id);
  
#ifdef HAS_FLOW_SENSOR
  len += snprintf_P(output+len, BUF_SIZE-len, PSTR(" \"flow\": %lu,"), get_flow_counter());
#endif
#ifdef HAS_DS18B20
  double temp = get_temp();
  len += snprintf_P(output+len, BUF_SIZE-len, PSTR(" \"temp\": %i.%i,"), (int)temp, abs((int)(temp * 100) % 100)); // AVR do not support float in printf
#endif
#ifdef HAS_SENSOR_A
  len += snprintf_P(output+len, BUF_SIZE-len, PSTR(" \"A\": %u,"), get_distance_A());
#endif
#ifdef HAS_SENSOR_A
  len += snprintf_P(output+len, BUF_SIZE-len, PSTR(" \"B\": %u,"), get_distance_B());
#endif
#ifdef HAS_NB_RELAYS
  for(uint8_t i=0; i < HAS_NB_RELAYS; i++)
  {
    len += snprintf_P(output+len, BUF_SIZE-len, PSTR(" \"%s\": %s,"), get_relay_label(i), is_relay_active(i) ? "true": "false");
  }
#endif

  len--; // Remove last comma
  len += snprintf_P(output+len, BUF_SIZE-len, PSTR("}}"));
  println(output);
}

// EOF
