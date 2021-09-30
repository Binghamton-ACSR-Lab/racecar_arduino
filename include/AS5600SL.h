#ifndef _AS5600SL_H_
#define _AS5600SL_H_

#include <Arduino.h>
#include <SlowSoftI2CMaster.h>

class AS5600SL{

public:
	AS5600SL(uint8_t sda,uint8_t scl);	
	bool init();
	bool detectMagnet();
	uint16_t getAngle();


private:
	SlowSoftI2CMaster si;
    const byte _i2c_address = (0x36<<1);
    const uint8_t  _stat = 0x0b;
    const uint8_t  _mag_hi = 0x1b;
    const uint8_t  _mag_lo = 0x1c;
    const uint8_t _zmco = 0x00;
    const uint8_t _raw_ang_hi=0x0C;
    const uint8_t _raw_ang_lo=0x0D;
    const uint8_t _ang_hi=0x0E;
    const uint8_t _ang_lo=0x0F;

  	byte readData(int in_adr);
    
};

#endif