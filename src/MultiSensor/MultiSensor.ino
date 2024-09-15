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
// - Distance AJ-SR04M See https://tutorials.probots.co.in/communicating-with-a-waterproof-ultrasonic-sensor-aj-sr04m-jsn-sr04t/
// - Temperature DS18B20
// - Flow YF-B5
//
// Supported outputs:
// - JSON Serial API
// - Standard relays

// Use board "Atmel atmega328p" to compile for compatible boards

#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <util/atomic.h>

// Feature configuration (coment out unneeded features)
#define HAS_FLOW_SENSOR   // Water flow sensro YF-B5 (5v)
#define HAS_DS18B20       // Temperature
#define HAS_NB_RELAYS 4   // Number of relays (1-4), comment out to disable relays
//#define HAS_SENSOR_A      // AJ-SR04M
//#define HAS_SENSOR_B      // AJ-SR04M

// GPIO configuration
#define GPIO_ONEWIRE 12     // OneWire pin bus
#define GPIO_FLOW_SENSOR 3  // Hall effect flow sensor YF-B5
#define GPIO_SENSOR_A_RX 2  // AJ-SR04M in Low Power Serial mode (R19 = 47k)
#define GPIO_SENSOR_A_TX 4
#define GPIO_SENSOR_B_RX 7
#define GPIO_SENSOR_B_TX 8
#define GPIO_RELAY0 13      // Relays GPIO
#define GPIO_RELAY1 11
#define GPIO_RELAY2 10
#define GPIO_RELAY3 9

#define DEFAULT_FREQUENCY_MS 1000 // Frequency of the execution in ms
#define ONEWIRE_ADDR_LEN 16       // 6 bytes + 3 chars header + EOS = 16 chars
#define BUF_SIZE 256              // Used for string buffers

// Debug macro
#define DBG(...) if(debug) { snprintf(buffer, BUF_SIZE, __VA_ARGS__); Serial.println(buffer); }

unsigned long lastrun_ms = 0;
unsigned long frequency_ms = DEFAULT_FREQUENCY_MS;
bool relays_active[] = {false, false, false, false};
int relays_gpio[] = { GPIO_RELAY0, GPIO_RELAY2, GPIO_RELAY2, GPIO_RELAY3 };
bool debug = false;
volatile unsigned long pulse_count = 0;  // Counter for flow sensor interruptions
SoftwareSerial serial_A(GPIO_SENSOR_A_RX, GPIO_SENSOR_A_TX);  // Serial interface to ultrasonic sensor AJ-SR04M
SoftwareSerial serial_B(GPIO_SENSOR_B_RX, GPIO_SENSOR_B_TX);
OneWire oneWire(GPIO_ONEWIRE);
DallasTemperature sensors(&oneWire);
char buffer[BUF_SIZE];            // Global char* to avoir multiple String concatenation which causes RAM fragmentation

StaticJsonDocument<200> json_output;
StaticJsonDocument<200> json_input;

struct ST_CALIBRATION {
  char addr[ONEWIRE_ADDR_LEN];
  float offset;
};

// Builtin calibration data for known devices 
ST_CALIBRATION calibration[] = {
  // 2022-04-30
  { "26-00000238aec7", -1.15},
  { "28-0416549140ff",  0.01},
  { "28-0316442b74ff", -1.43},
  { "28-04168438ddff", -0.05},
  { "28-00044c9e09ff", -0.30},
  { "28-000003dd2964", -0.35},
  { "28-00000ab5377d", -1.05},
  { "28-00000ab525b3", -2.88},
  { "28-0620153d35c4", -0.79},  // Unstable
  { "28-062015a347a4", -0.28},
  { "28-0620153cd4ea", -0.79},  // Unstable
  { "28-00044d4226ff", -1.23},
  { "28-0316859752ff",  0.03},
  
  // Updated 2024-04-21
  {"28-062015683130", -0.19 },
  {"28-4d23d44382e6", -0.56 },
  {"28-607e2a346461", -0.44 },
  {"28-6c7e2a346461", -0.50 },
  {"28-d1632a346461", -0.44 },
  {"28-747ad5346461", -0.38 },
  {"28-e576d5346461", -0.50 },
  {"28-2ed9d4432c09", -0.81 },
  {"28-062015408cb5", -0.37 },
};

/**
 * One wire hepers
 ********************************************************************************/

char *convertAddress(char *str, DeviceAddress addr)
{
  // Linux kernel format for 1-wire adresses
  snprintf(str, ONEWIRE_ADDR_LEN, "%02x-%02x%02x%02x%02x%02x%02x", addr[0], addr[6], addr[5],addr[4], addr[3], addr[2], addr[1]);
  return str;
}

char *getDeviceAddress(char *str,  uint8_t index)
{
  DeviceAddress addr;
  if(sensors.getAddress(addr, index))
  {
    snprintf(str, ONEWIRE_ADDR_LEN, "%02x-%02x%02x%02x%02x%02x%02x", addr[0], addr[6], addr[5],addr[4], addr[3], addr[2], addr[1]);
  }
  else
  {
    json_error(F("Onewire address not found"));
    str[0]='\0';
  }
  return str;
}

float calibrate(char *addr, float input)
{
  // Forward no-data
  if (input == DEVICE_DISCONNECTED_C)
    return DEVICE_DISCONNECTED_C;

  for(uint8_t i=0; i <= (sizeof(calibration) / sizeof(ST_CALIBRATION)); i++)
  {
    if(strcmp(addr, calibration[i].addr) == 0)
    {
      return input + calibration[i].offset;
    }
  }

  // No calibration data found
  return input;
}


double get_temp()
{
  double temp;
  char addr[ONEWIRE_ADDR_LEN];
  int max_try=10;
  int index=0;
  
  do
  {
    sensors.requestTemperaturesByIndex(index); 
    delay(10); 
    temp = sensors.getTempCByIndex(index);
    
    max_try--;
  }
  while ((temp == 85.0 || temp == (-127.0)) && max_try > 0);

  if (max_try <= 0)
  {
    temp = -127;
  }
  else
  {
     // Builtin calibration for known sensors
     getDeviceAddress(addr, index);
     
     if(strlen(addr)>0)
     {
       temp = calibrate(addr, temp);    
     }
     else
     {
       json_error("Can't get sensor address for calibration.");
     }
  }

  return temp;
}


/**
 * Serial JSON API handlers
 ********************************************************************************/

void json_error(String msg)
{
  json_output.clear();
  json_output["error"] = msg;
  serializeJson(json_output, Serial);
  Serial.println();
}

void print_help()
{
  Serial.println(F(R"(
    MultiSensor v1.0 JSON API
    {"config": {}} : Get config
    {"config": {"frequency": 2000}} : Set frequency in s
    {"config": {"debug": true}} : Enable debug mode
    {"relayX": "on"} : Switch X on
    {"relayX": "off"} : Switch X off
    {"relayX": "toggle"} : Toggle switch X
  )"));
}

void print_json_config()
{
  json_output.clear();
  JsonObject config = json_output.createNestedObject("config");
  config["frequency"] = frequency_ms;
  config["debug"] = debug;
  serializeJson(json_output, Serial);
  Serial.println();
}

bool save_json_config(JsonObject json)
{
  // Save debug
  if(json.containsKey("debug"))
  {
    debug = json["debug"];
    DBG("Saved debug=%s", debug ? "true": "false");
  }

  if(json.containsKey("frequency"))
  {
    // Save frequency
    const int freq = json["frequency"];

    if(freq <= 0)
    {
      json_error(F("Invalid frequency parameter: need to be positive integer"));
      return false;
    }
    else
    {
      frequency_ms = freq;
      DBG("Saved frequency=%i", frequency_ms);
    }
  }

  return true;
}


void handle_serial_api()
{
  if(Serial.available())
  {
    String input = Serial.readString();
    input.trim();
    
    if(input=="help")
    {
      print_help();
      return;
    }
    
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(json_input, input);

     // Test if parsing succeeds.
    if(error)
    {
      json_error("deserializeJson() failed: " + String(error.f_str()));
      return;
    }

    // Manage configuration
    if(json_input.containsKey("config"))
    {
      if(json_input["config"].isNull() || json_input["config"].size() == 0)
      {
        print_json_config();
      }
      else
      {
        if(save_json_config(json_input["config"]))
        {
          print_json_config();
        }
      }
      return;
    }

    // Manage relays
#ifdef HAS_NB_RELAYS
    for(int i=0; i < HAS_NB_RELAYS; i++)
    {
      snprintf(buffer, BUF_SIZE, "relay%d", i);
      if(json_input.containsKey(buffer))
      {
        String action = json_input[buffer];
        if(action == "on")
        {
          switch_on_relay(i, true);
        }
        else if(action == "off")
        {
          switch_on_relay(i, false);
        }
        else if(action == "toggle")
        {
          switch_on_relay(i, !relays_active[i]);
        }
        else
        {
          json_error("Unknown action:" + action);
        }
      }
    }
#endif
  }
}

void switch_on_relay(int id, bool on)
{
  digitalWrite(relays_gpio[id], on ? HIGH : LOW);
  relays_active[id]=on;
}

/**
 * Flow sensor helpers
 ********************************************************************************/

unsigned long get_flow_counter()
{
  unsigned long count = 0;  

  // Need atomic as a lock to access volatile variable
  // https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    count = pulse_count;
  }
  
  return count;
}


void pulse()
{
  pulse_count++;
}


/**
 * Ultrasonic sensor helpers
 ********************************************************************************/
 
unsigned int get_distance(SoftwareSerial serial)
{
  unsigned int distance;
  byte start_byte, h_data, l_data, sum = 0;
  byte buf[3];

  // Trigger measurement
  serial.write(0x01);
  serial.listen();
  delay(100);
  if(serial.available())
  {
    start_byte = (byte)serial.read();
    if(start_byte == 255)
    {
      serial.readBytes(buf, 3);
      h_data = buf[0];
      l_data = buf[1];
      sum = buf[2];

      DBG("h_data=%x l_data=%x sum=%x", h_data, l_data, sum);

      if(((h_data + l_data)-1) != sum)
      {
        DBG("Wrong checksum");
         return 0;
      }

      distance = (h_data<<8) + l_data;
      return distance;
    }
  }
  else
  {
    DBG("Serial port not available");
    return 0;
  }
}

void setup()
{
  Serial.begin(9600);
  
#ifdef HAS_SENSOR_A
  serial_A.begin(9600);
#endif
#ifdef HAS_SENSOR_B
  serial_B.begin(9600);
#endif
#ifdef HAS_DS18B20
  sensors.begin();
#endif

  // Relays output
#ifdef HAS_NB_RELAYS
  for(int i=0; i < HAS_NB_RELAYS; i++)
  {
    pinMode(relays_gpio[i], OUTPUT);
    digitalWrite(relays_gpio[i], LOW);
  }
#endif

  // Flow Sensor
#ifdef HAS_FLOW_SENSOR
  attachInterrupt(digitalPinToInterrupt(GPIO_FLOW_SENSOR), pulse, RISING);
#endif

  Serial.println(F("MultiSensor v1.0 started."));
}

void loop()
{
  handle_serial_api();
  
  unsigned long now_ms = millis();
  if(now_ms - lastrun_ms > frequency_ms)
  {
    send_sensors_json_data();
    
    // update the timing variable
    lastrun_ms = now_ms;
  }  
}


void send_sensors_json_data()
{
  json_output.clear();
  JsonObject data = json_output.createNestedObject("data");
  
#ifdef HAS_FLOW_SENSOR
  data["flow"] = get_flow_counter();
#endif
#ifdef HAS_DS18B20
  data["temp"] = get_temp();
#endif
#ifdef HAS_SENSOR_A
  data["A"] = get_distance(serial_A);
#endif
#ifdef HAS_SENSOR_A
  data["B"] = get_distance(serial_B);
#endif
#ifdef HAS_NB_RELAYS
  for(int i=0; i < HAS_NB_RELAYS; i++)
  {
    snprintf(buffer, BUF_SIZE, "relay%d", i);
    data[buffer] = relays_active[i];
  }
#endif
  
  serializeJson(json_output, Serial);
  Serial.println();
}

// EOF
