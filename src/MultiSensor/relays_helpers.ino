/*
 * This file is part of MultiSensor Arduino sketch under GPLv3
 * All helpers related to relays management are here for ease of navigation.
 *
 */

char* relays_label[] = { "relay0", "relay1", "relay2", "relay3" };
bool relays_active[] = {false, false, false, false};
uint8_t relays_gpio[] = { GPIO_RELAY0, GPIO_RELAY1, GPIO_RELAY2, GPIO_RELAY3 };

void setup_relays()
{
  for(uint8_t i=0; i < HAS_NB_RELAYS; i++)
  {
    pinMode(relays_gpio[i], OUTPUT);
    digitalWrite(relays_gpio[i], LOW);
  }
}

char* get_relay_label(uint8_t id)
{
  return relays_label[id];
}

bool is_relay_active(uint8_t id)
{
  return relays_active[id];
}

void switch_relay(uint8_t id, bool on)
{
  digitalWrite(relays_gpio[id], on ? HIGH : LOW);
  relays_active[id]=on;
  DBG(PSTR("Event on %s : %s"), relays_label[id], on ? "true": "false");
  send_sensors_json_data();
}

// EOF