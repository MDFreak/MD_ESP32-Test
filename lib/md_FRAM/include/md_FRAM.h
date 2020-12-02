/**************************************************************************/
/*!
    @file     Adafruit_FRAM_I2C.h
    @author   KTOWN (Adafruit Industries)

    Software License Agreement (BSD License)

    Copyright (c) 2013, Adafruit Industries
    All rights reserved.
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
*/
/**************************************************************************/
#ifndef _MD_FRAM_H_
  #define _MD_FRAM_H_

  #if ARDUINO >= 100
      #include <Arduino.h>
    #else
      #include <WProgram.h>
    #endif

  #include <Wire.h>

  #define MB85RC_DEFAULT_ADDRESS (0x50)  ///<* 1010 + A2 + A1 + A0 = 0x50 default */
  #define MB85RC_SLAVE_ID (0xF8)         ///< SLAVE ID

  /*!
   *    @brief  Class that stores state and functions for interacting with
   *            I2C FRAM chips
   */
  class md_FRAM {
    public:
      md_FRAM(void);

      boolean begin(uint8_t addr = MB85RC_DEFAULT_ADDRESS);
      boolean begin(int sda, int scl, uint8_t addr = MB85RC_DEFAULT_ADDRESS);
      void write8(uint16_t framAddr, uint8_t value);
      uint8_t read8(uint16_t framAddr);
      void getDeviceID(uint16_t *manufacturerID, uint16_t *productID);
      void writeBlock(uint16_t memaddr, uint8_t * obj, uint8_t size);
      void readBlock(uint16_t memaddr, uint8_t * obj, uint8_t size);

    private:
      uint8_t _i2c_addr;
      boolean _framInitialised;
      uint8_t _sda;
      uint8_t _scl;
      bool    _doI2cAutoInit = false;
  };

#endif
