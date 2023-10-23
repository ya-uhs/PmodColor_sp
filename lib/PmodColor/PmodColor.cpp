/************************************************************************/
/*																		*/
/*	PmodColor.cpp		--		Definition for PmodColor library 	    */
/*																		*/
/************************************************************************/
/*	Author:		James Colvin											*/
/*	Copyright 2017, Digilent Inc.										*/
/************************************************************************/
/*  File Description:													*/
/*		This file defines functions for Pmod Color						*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	10/06/2017(JColvin): created										*/
/*																		*/
/************************************************************************/

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */
#include "PmodColor.h"
#include <Wire.h>
#include <Arduino.h>

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

/* -------------------------------------------------------- */
/*        PmodColor::PmodColor
**
**        Synopsis:
**				PmodColor()
**
**        Parameters:
**				none
**
**        Return Values:
**                none 
**
**        Errors:
**			none
**
**        Description:
**			This function is a constructor.
************************************************************/
PmodColor::PmodColor()
{
  //yay class functions!
}

/* -------------------------------------------------------- */
/*        PmodColor::begin
**
**        Synopsis:
**				begin()
**
**        Parameters:
**				none
**
**        Return Values:
**                boolean flag 
**
**        Errors:
**			none
**
**        Description:
**			This function sets up some initial registers and confirms we are successfully connected to the Pmod Color.
************************************************************/
bool PmodColor::begin(void)
{
  Wire.begin();										//start up I2C

  if (detect()){		//if the ID doesn't match either possible ID tag
	//setIntegrationTime(_PmodColorIntegrationTime);	//set integration time
	//setGain(_PmodColorGain);						//set gain
	enable();
    return true;
  }
  else{
	  return false;
  }  

}

/* -------------------------------------------------------- */
/*        PmodColor::detect
**
**        Synopsis:
**				detect()
**
**        Parameters:
**				none
**
**        Return Values:
**                boolean flag 
**
**        Errors:
**			none
**
**        Description:
**			This function sets up some initial registers and confirms we are successfully connected to the Pmod Color.
************************************************************/
bool PmodColor::detect(void)
{
  uint8_t idValue = readByteI2C(PmodColor_ID);		//read the ID of the chip
  if ((idValue != 0x44) && (idValue != 0x10))		//if the ID doesn't match either possible ID tag
  {
    return false;
  }

  return true;
}


/* ------------------------------------------------------------ */
/*        PmodColor::writeByteI2C
**
**        Synopsis:
**				writeByteI2C(uint8_t reg, int value);
**
**        Parameters:
**				uint8_t reg - the register address to be written to
**				int value - the byte to be written
**
**        Return Values:
**                void 
**
**        Errors:
**			none
**
**        Description:
**			This function writes to a specified register over I2C. 
************************************************************************/
void PmodColor::writeByteI2C (uint8_t reg, int value)
{
  Wire.beginTransmission(PmodColor_address);

  Wire.write(cmd_repeated_byte | reg);
  Wire.write(value & 0xFF);

  Wire.endTransmission();
}

/* ------------------------------------------------------------ */
/*        PmodColor::writeSingleByteI2C
**
**        Synopsis:
**				writeSingleByteI2C(uint8_t reg, int value);
**
**        Parameters:
**				uint8_t reg - the register address to be written to
**				int value - the byte to be written
**
**        Return Values:
**                void 
**
**        Errors:
**			none
**
**        Description:
**			This function writes to a specified register over I2C. 
************************************************************************/
void PmodColor::writeSingleByteI2C (uint8_t reg, int value)
{
  Wire.beginTransmission(PmodColor_address);

  Wire.write(cmd_repeated_byte | reg);
  Wire.write(value & 0xFF);

  Wire.endTransmission();
}

/* ------------------------------------------------------------ */
/*        PmodColor::readByteI2C
**
**        Synopsis:
**				readByteI2C(uint8_t reg)
**        Parameters:
**				uint8_t reg - the register address to be read from
**
**        Return Values:
**                the read byte 
**
**        Errors:
**			none
**
**        Description:
**			This function reads from a specified register over I2C. 
************************************************************************/
uint8_t PmodColor::readByteI2C(uint8_t reg)
{
  Wire.beginTransmission(PmodColor_address);	//start up I2C

  Wire.write(cmd_auto_increment | reg);			//Send the register of interest

  Wire.endTransmission();						//briefly stop I2C

  Wire.requestFrom(PmodColor_address, 1);		//Request a single byte from the Pmod from the previously sent register address

  return Wire.read();						//return the received data

}

/* ------------------------------------------------------------ */
/*        PmodColor::readWordI2C
**
**        Synopsis:
**				readWordI2C(uint8_t reg)
**        Parameters:
**				uint8_t reg - the register address to be read from
**
**        Return Values:
**                the read word 
**
**        Errors:
**			none
**
**        Description:
**			This function reads from a specified register over I2C. 
************************************************************************/
uint16_t PmodColor::readWordI2C(uint8_t reg)
{
  uint16_t x; uint16_t t;

  Wire.beginTransmission(PmodColor_address);	//start up communications

  Wire.write(cmd_auto_increment | reg);			//send the register of interest to communicate with

  Wire.endTransmission();						//briefly pause

  Wire.requestFrom(PmodColor_address, 2);		//request 2 bytes of data from the previously sent address

  t = Wire.read();							//receive the 2 bytes
  x = Wire.read();

  x <<= 8;
  x |= t;										//merge the two bytes
  return x;										//return the data
}

/* ------------------------------------------------------------ */
/*        PmodColor::getData
**
**        Synopsis:
**				getData(uint16_t *c, uint16_t *r, uint16_t *g, uint16_t *b)
**        Parameters:
**				uint16_t *c - the two bytes of clear data
**				uint16_t *r - the two bytes of red data
**				uint16_t *g - the two bytes of green data
**				uint16_t *b - the two bytes of blue data
**
**        Return Values:
**                the data stored in the registers 
**
**        Errors:
**			none
**
**        Description:
**			This function reads all the data registers over I2C. 
************************************************************************/
void PmodColor::getData(uint16_t &c, uint16_t &r, uint16_t &g, uint16_t &b)
{
  uint16_t x; uint16_t t;

  Wire.beginTransmission(PmodColor_address);	//start up communications

  Wire.write(cmd_auto_increment | PmodColor_CdataL);			//send the register of interest to communicate with

  Wire.endTransmission();						//briefly pause

  Wire.requestFrom(PmodColor_address, 8);		//request all 8 bytes of color data starting from the previously sent address

  //Collect the low and high bytes of the clear data
  t = Wire.read();							//receive the 2 bytes
  x = Wire.read();
  x <<= 8;										//shift the high byte
  x |= t;										//merge the two bytes
  c = x;										//store the clear data
  
  //Collect the low and high bytes of the red data
  t = Wire.read();							//receive the 2 bytes
  Serial.print("initial low: ");Serial.println(t);
  x = Wire.read();
  Serial.print("initial high: ");Serial.println(x);
  x <<= 8;										//shift the high byte
  Serial.print("shifted high: ");Serial.println(x);
  x |= t;										//merge the two bytes
  Serial.print("merged bytes: ");Serial.println(x);
  r = x;										//store the red data
  
  //Collect the low and high bytes of the green data
  t = Wire.read();							//receive the 2 bytes
  x = Wire.read();
  x <<= 8;										//shift the high byte
  x |= t;										//merge the two bytes
  g = x;										//store the green data
  
  //Collect the low and high bytes of the blue data
  t = Wire.read();							//receive the 2 bytes
  x = Wire.read();
  x <<= 8;										//shift the high byte
  x |= t;										//merge the two bytes
  b = x;										//store the blue data
  
  
}

/* ------------------------------------------------------------ */
/*        PmodColor::enable
**
**        Synopsis:
**				enable()
**
**        Parameters:
**				none
**
**        Return Values:
**                none 
**
**        Errors:
**			none
**
**        Description:
**			This function sets the appropriate bits in the enable register to allow the Pmod Color to function. 
************************************************************************/
void PmodColor::enable(void)
{
  writeSingleByteI2C(PmodColor_Enable, PmodColor_Enable_PON);
  delay(3);		//required delay of 2.4 ms
  writeSingleByteI2C(PmodColor_Enable, PmodColor_Enable_PON | PmodColor_Enable_AEN);
}





/* ------------------------------------------------------------ */
/*        PmodColor::setIntegrationTime
**
**        Synopsis:
**				setIntegrationTime
**
**        Parameters:
**				?
**
**        Return Values:
**                none 
**
**        Errors:
**			none
**
**        Description:
**			This function  
************************************************************************/
void PmodColor::setIntegrationCycles(uint8_t it)
{

  /* Update the timing register */
  //writeByteI2C(PmodColor_ATIME, it);
  if(it<1){
	  it = 1;
  }
  if(it>255){
	  it=255;
  }
  
  uint8_t integration_cycles = 256 - it;
  writeSingleByteI2C(PmodColor_integrationTimeReg, integration_cycles);

}

/* ------------------------------------------------------------ */
/*        PmodColor::setGain
**
**        Synopsis:
**				setGain
**
**        Parameters:
**				?
**
**        Return Values:
**                none 
**
**        Errors:
**			none
**
**        Description:
**			This function  
************************************************************************/
void PmodColor::setGain(uint8_t gain)
{

  /* Update the timing register */
  writeSingleByteI2C(PmodColor_Control, gain);

}


/* ------------------------------------------------------------ */
/*        PmodColor::setInterrupt
**
**        Synopsis:
**				setInterrupt
**
**        Parameters:
**				?
**
**        Return Values:
**                none 
**
**        Errors:
**			none
**
**        Description:
**			This function  
************************************************************************/

void PmodColor::setInterrupt(bool i) {
/*  uint8_t r = readByteI2C(PmodColor_Enable);
  if (i) {
    r |= PmodColor_ENABLE_AIEN;
  } else {
    r &= ~PmodColor_ENABLE_AIEN;
  }
  writeByteI2C(PmodColor_ENABLE, r);*/
}


/* ------------------------------------------------------------ */
/*        PmodColor::clearInterrupt
**
**        Synopsis:
**				clearInterrupt
**
**        Parameters:
**				?
**
**        Return Values:
**                none 
**
**        Errors:
**			none
**
**        Description:
**			This function  
************************************************************************/
void PmodColor::clearInterrupt(void) {
  Wire.beginTransmission(PmodColor_address);

  Wire.write(cmd_auto_increment | 0x66);

  Wire.endTransmission();
}

/* ------------------------------------------------------------ */
/*        PmodColor::setIntLimits
**
**        Synopsis:
**				setIntLimits
**
**        Parameters:
**				?
**
**        Return Values:
**                none 
**
**        Errors:
**			none
**
**        Description:
**			This function  
************************************************************************/
void PmodColor::setIntLimits(uint16_t low, uint16_t high) {
   writeByteI2C(0x04, low & 0xFF);
   writeByteI2C(0x05, low >> 8);
   writeByteI2C(0x06, high & 0xFF);
   writeByteI2C(0x07, high >> 8);
}