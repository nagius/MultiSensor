/*
 * This file is part of MultiSensor Arduino sketch under GPLv3
 * All helpers related to serial communications are here for ease of navigation.
 *
 */


 /**
 * MAX485 helpers
 ********************************************************************************/

void setup_MAX485()
{
  pinMode(GPIO_PTT, OUTPUT);
  digitalWrite(GPIO_PTT, LOW); // Start as receiver
}

void println(const char * msg)
{
  ptt_push();
  Serial.println(msg);
  ptt_release();
}

void ptt_push()
{
#ifdef HAS_MAX485
  digitalWrite(GPIO_PTT, HIGH);
  delay(10);
#endif
}

void ptt_release()
{
#ifdef HAS_MAX485
  Serial.flush();
  delay(10);
  digitalWrite(GPIO_PTT, LOW);
#endif
}
