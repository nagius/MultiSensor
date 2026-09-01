/*
 * This file is part of MultiSensor Arduino sketch under GPLv3
 * All helpers related to relays management are here for ease of navigation.
 *
 */

struct ST_SETTINGS {
  uint8_t  id;
  uint32_t freq_ms;
};

ST_SETTINGS settings;

 /**
 * Flash memory helpers 
 ********************************************************************************/

// CRC8 simple calculation
// Based on https://github.com/PaulStoffregen/OneWire/blob/master/OneWire.cpp
uint8_t crc8(const uint8_t *addr, uint8_t len)
{
  uint8_t crc = 0;

  while (len--) {
    uint8_t inbyte = *addr++;
    for (uint8_t i = 8; i; i--) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      inbyte >>= 1;
    }
  }
  return crc;
}

void setup_settings()
{
  load_settings();
}

void set_id(uint8_t id)
{
  settings.id = id;
  save_settings();
}

void set_freq_ms(uint32_t freq_ms)
{
  settings.freq_ms = freq_ms;
  save_settings();
}

uint8_t get_id()
{
  return settings.id;
}

uint32_t get_freq_ms()
{
  return settings.freq_ms;
}

void save_settings()
{
  uint8_t buffer[sizeof(settings) + 1];  // Use the last byte for CRC

  memcpy(buffer, &settings, sizeof(settings));
  buffer[sizeof(settings)] = crc8(buffer, sizeof(settings));

  for(int i=0; i < sizeof(buffer); i++)
  {
    EEPROM.write(i, buffer[i]);
  }
}

void load_settings()
{
  uint8_t buffer[sizeof(settings) + 1];  // Use the last byte for CRC

  for(int i=0; i < sizeof(buffer); i++)
  {
    buffer[i] = uint8_t(EEPROM.read(i));
  }

  // Check CRC
  if(crc8(buffer, sizeof(settings)) == buffer[sizeof(settings)])
  {
    memcpy(&settings, buffer, sizeof(settings));
    DBG(PSTR("Loaded settings from flash"));
  }
  else
  {
    DBG(PSTR("Bad CRC, loading default settings"));
    set_default_settings();
    save_settings();
  }
}

void set_default_settings()
{
  set_id(DEFAULT_ID);
  set_freq_ms(DEFAULT_FREQUENCY_MS);
}
 
// EOF