/*
 * This file is part of MultiSensor Arduino sketch under GPLv3
 * All helpers related to DS18B20 sensors management are here for ease of navigation.
 *
 */

OneWire oneWire(GPIO_ONEWIRE);
DallasTemperature sensors(&oneWire);

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
  { "28-062015683130", -0.19 },
  { "28-4d23d44382e6", -0.56 },
  { "28-607e2a346461", -0.44 },
  { "28-6c7e2a346461", -0.50 },
  { "28-d1632a346461", -0.44 },
  { "28-747ad5346461", -0.38 },
  { "28-e576d5346461", -0.50 },
  { "28-2ed9d4432c09", -0.81 },
  { "28-062015408cb5", -0.37 },
};

/**
 * One wire hepers
 ********************************************************************************/

void setup_DS18B20()
{
  sensors.begin();
}


char *convertAddress(char *str, DeviceAddress addr)
{
  // Linux kernel format for 1-wire adresses
  snprintf_P(str, ONEWIRE_ADDR_LEN, PSTR("%02x-%02x%02x%02x%02x%02x%02x"), addr[0], addr[6], addr[5],addr[4], addr[3], addr[2], addr[1]);
  return str;
}

char *getDeviceAddress(char *str,  uint8_t index)
{
  DeviceAddress addr;
  if(sensors.getAddress(addr, index))
  {
    snprintf_P(str, ONEWIRE_ADDR_LEN, PSTR("%02x-%02x%02x%02x%02x%02x%02x"), addr[0], addr[6], addr[5],addr[4], addr[3], addr[2], addr[1]);
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
      DBG(PSTR("Applied calibration %i.%i"), (int)calibration[i].offset, abs((int)(calibration[i].offset * 100) % 100));
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
  uint8_t max_try=10;
  uint8_t index=0;  // Only read the first sensor
  
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
       json_error(F("Can't get sensor address for calibration."));
     }
  }

  return temp;
}

// EOF