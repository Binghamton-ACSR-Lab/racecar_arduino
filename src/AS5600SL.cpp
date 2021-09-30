#include "AS5600SL.h"

AS5600SL::AS5600SL(uint8_t sda,uint8_t scl):si(sda,scl,true){

}

bool AS5600SL::init(){
    return si.i2c_init();
}

bool AS5600SL::detectMagnet(){

  /*0 0 MD ML MH 0 0 0*/
  /* MD high = magnet detected*/
  /* ML high = AGC Maximum overflow, magnet to weak*/
  /* MH high = AGC minimum overflow, Magnet to strong*/
  uint8_t magStatus = readData(_stat);
  if (magStatus & 0x20)
    return true;
  return false;
}

uint16_t AS5600SL::getAngle()
{
  uint16_t v = readData(_ang_hi) & 0xFF;
  uint8_t v_l = readData(_ang_lo);
  return v<<8 | v_l;
}

byte AS5600SL::readData(int in_adr)
{
  si.i2c_start(_i2c_address | I2C_READ);
  si.i2c_read(true);
  si.i2c_stop();
  //int retVal = -1;
  si.i2c_start(_i2c_address | I2C_WRITE);
  si.i2c_write(in_adr);
  si.i2c_stop();
  
  si.i2c_rep_start(_i2c_address | I2C_READ);
  uint8_t retVal = si.i2c_read(true);
  return retVal;  
}




