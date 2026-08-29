/*
 * This file is part of MultiSensor Arduino sketch under GPLv3
 * All helpers related to YF-B5 flow sensors management are here for ease of navigation.
 *
 */


// Counter for flow sensor interruptions
volatile unsigned long pulse_count = 0;

/**
 * Flow sensor helpers
 ********************************************************************************/

 void setup_flow_counter()
 {
   attachInterrupt(digitalPinToInterrupt(GPIO_FLOW_SENSOR), pulse, RISING);
 }

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

// EOF