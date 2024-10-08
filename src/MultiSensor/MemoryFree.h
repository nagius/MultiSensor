// MemoryFree library based on code posted here:
// http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1213583720/15
//
// Extended by Matthew Murdoch to include walking of the free list.

// To use, add to your sketch
// #include "MemoryFree.h"
// Serial.print("freeMemory()=");
// Serial.println(freeMemory());

#ifndef MEMORY_FREE_H
#define MEMORY_FREE_H

#ifdef __cplusplus
extern "C" {
#endif

  int freeMemory();

#ifdef  __cplusplus
}
#endif

#endif
