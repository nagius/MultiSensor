/*
 * This file is part of MultiSensor Arduino sketch under GPLv3
 * All helpers related to AJ-SR04M ultrasonic sensors management are here for ease of navigation.
 *
 */

// Serial interface to ultrasonic sensor AJ-SR04M
SoftwareSerial serial_A(GPIO_SENSOR_A_RX, GPIO_SENSOR_A_TX);  
SoftwareSerial serial_B(GPIO_SENSOR_B_RX, GPIO_SENSOR_B_TX);

 /**
 * Ultrasonic sensor helpers
 ********************************************************************************/
 
void setup_distance_A()
{
    serial_A.begin(9600);
}

void setup_distance_B()
{
    serial_B.begin(9600);
}

unsigned int get_distance_A()
{
  return get_distance(serial_A);
}

unsigned int get_distance_B()
{
  return get_distance(serial_B);
}

unsigned int get_distance(SoftwareSerial& serial)
{
  unsigned int distance;
  byte start_byte, h_data, l_data, sum = 0;
  byte buf[3];

  // Cleanup buffer
  serial.listen();
  while(serial.available())
  {
    serial.read();
  }

  // Trigger measurement
  serial.write(0x01);
  delay(150);
  if(!serial.available())
  {
    DBG(PSTR("Serial port not available"));
    return 0;
  }

  while(serial.available())
  {
    start_byte = (byte)serial.read();
    if(start_byte == 255)
    {
      if(serial.readBytes(buf, 3) != 3)
      {
        DBG(PSTR("Wrong message length"));
        return 0;
      }

      h_data = buf[0];
      l_data = buf[1];
      sum = buf[2];

      DBG(PSTR("h_data=%x l_data=%x sum=%x"), h_data, l_data, sum);

      if(((0xFF + h_data + l_data) & 0xFF) != sum)
      {
        DBG(PSTR("Wrong checksum"));
        return 0;
      }

      distance = (h_data<<8) + l_data;
      return distance;
    }
  }

  DBG(PSTR("Synchonization lost"));
  return 0;
}
