#ifndef _ACSR_HARDWARE_
#define _ACSR_HARDWARE_

#include <ArduinoHardware.h>

class AcsrHardware : public ArduinoHardware
{
  public:
  AcsrHardware():ArduinoHardware(&Serial1, 115200){};
};

#endif