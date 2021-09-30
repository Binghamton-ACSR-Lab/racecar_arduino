
/****************************************************
  AMS 5600 class for Arduino platform
  Author: Tom Denton
  Date: 15 Dec 2014
  File: AMS_5600.h 
  Version 1.00
  www.ams.com
   
  Description:  This class has been designed to
  access the AMS 5600 “potuino” shield.
***************************************************/

#ifndef AMS_5600_h
#define AMS_5600_h

#include <Arduino.h>

class AMS_5600
{
public:
  AMS_5600(byte);
  int getAddress();

  word setMaxAngle(word newMaxAngle = -1);
  word getMaxAngle();

  word setStartPosition(word startAngle = -1);
  word getStartPosition();

  word setEndPosition(word endAngle = -1);
  word getEndPosition();

  word getRawAngle();
  word getScaledAngle();

  byte detectMagnet();
  byte getMagnetStrength();
  int getAgc();
  word getMagnitude();

  int getBurnCount();
  int burnAngle();
  int burnMaxAngleAndConfig();
  void setOutPut(uint8_t mode);

private:
  byte _ams5600_Address;

  word _rawStartAngle;
  word _zPosition;
  word _rawEndAngle;
  word _mPosition;
  word _maxAngle;

  /* Registers */
  const byte _zmco = 0x00;
  const byte _zpos_hi = 0x01;
  const byte _zpos_lo = 0x02;
  const byte _mpos_hi = 0x03;
  const byte _mpos_lo = 0x04;
  const byte _mang_hi = 0x05;
  const byte _mang_lo = 0x06;
  const byte _conf_hi = 0x07;
  const byte _conf_lo = 0x08;
  const byte _raw_ang_hi = 0x0c;
  const byte _raw_ang_lo = 0x0d;
  const byte _ang_hi = 0x0E;
  const byte _ang_lo = 0x0F;
  const byte _stat = 0x0b;
  const byte _agc = 0x1a;
  const byte _mag_hi = 0x1b;
  const byte _mag_lo = 0x1c;
  const byte _burn = 0xff;

/*
  byte _zmco;
  byte _zpos_hi; 
  byte _zpos_lo;
  byte _mpos_hi;
  byte _mpos_lo;
  byte _mang_hi; 
  byte _mang_lo; 
  byte _conf_hi;
  byte _conf_lo;
  byte _raw_ang_hi;
  byte _raw_ang_lo;
  byte _ang_hi;
  byte _ang_lo;
  byte _stat;
  byte _agc;
  byte _mag_hi;
  byte _mag_lo;
  byte _burn;
*/
  byte readOneByte(byte in_adr);
  word readTwoBytes(byte in_adr_hi, byte in_adr_lo);
  void writeOneByte(int adr_in, int dat_in);
};
#endif