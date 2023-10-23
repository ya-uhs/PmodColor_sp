#ifndef PmodColor_H
#define PmodColor_H

#include <Wire.h>

#define PmodColor_address		(0x29)
#define cmd_auto_increment		(0xA0)
#define cmd_repeated_byte		(0x80)

#define PmodColor_Enable		(0x00)
#define PmodColor_Enable_PON	(0x01)
#define PmodColor_Enable_AEN	(0x02)
#define PmodColor_integrationTimeReg	(0x01)
#define PmodColor_Control          (0x0F)    /* Set the gain level for the sensor */
#define PmodColor_ID               (0x12)    /* 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
#define PmodColor_Status           (0x13)
#define PmodColor_Status_Aint      (0x10)    /* RGBC Clean channel interrupt */
#define PmodColor_Status_Avalid    (0x01)    /* Indicates that the RGBC channels have completed an integration cycle */
#define PmodColor_CdataL           (0x14)    /* Clear channel data */
#define PmodColor_CdataH           (0x15)
#define PmodColor_RdataL           (0x16)    /* Red channel data */
#define PmodColor_RdataH           (0x17)
#define PmodColor_GdataL           (0x18)    /* Green channel data */
#define PmodColor_GdataH           (0x19)
#define PmodColor_BdataL           (0x1A)    /* Blue channel data */
#define PmodColor_BdataH           (0x1B)

class PmodColor{
 public:
  PmodColor();
  bool	 	begin(void);
  void		setIntegrationCycles(uint8_t integrationCycles);
  void		setGain(uint8_t gain);
  void		getData(uint16_t &c, uint16_t &r, uint16_t &g, uint16_t &b);
  void		writeByteI2C (uint8_t reg, int value);
  void		writeSingleByteI2C (uint8_t reg, int value);
  uint8_t	readByteI2C (uint8_t reg);
  uint16_t	readWordI2C (uint8_t reg);
  void		setInterrupt(bool flag);
  void		clearInterrupt(void);
  void		setIntLimits(uint16_t l, uint16_t h);
  void		enable(void);
  bool		detect(void);
  //send and requestFrom
};

#endif