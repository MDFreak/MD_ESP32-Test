#ifndef _MAIN_H_
  #define _MAIN_H_

  #include <Arduino.h>
  #include <unity.h>
  #include <Wire.h>
  #include <md_time.hpp>
  #include <string.h>
  //#include <Print.h>
  //#include <SPI.h>
  #include <prj_config.h>
  #include <md_util.h>
  #include <md_defines.h>
  #include <ip_list.hpp>
  #include <md_filter.hpp>

  // --- user interface
    #if (USE_TOUCHSCREEN > OFF)
      #include "md_touch.h"
    #endif // USE_TOUCHSCREEN

    #if (USE_BUZZER > OFF)
      #include "md_buzzer.h"
    #endif // USE_BUZZER

    #if (USE_OLED_I2C > OFF)
      #ifdef OLED1
          #if !(OLED1_DRV ^ OLED_DRV_1106)
              #include "md_oled_SH1106.h"
            #else
              #include "md_oled.h"
            #endif
        #endif
      #ifdef OLED2
          #if !(OLED2_DRV ^ OLED_DRV_1106)
              #include "md_oled_SH1106.h"
            #else
              #include "md_oled.h"
            #endif
        #endif
    #endif // USE_OLED_I2C

    #if (USE_KEYPADSHIELD > OFF)
      #include "md_keypadshield.h"
    #endif // USE_KEYPADSHIELD

    #if (USE_TFT > OFF)
      #include "md_lcd.h"
    #endif

    #if (USE_FRAM_I2C > OFF)
      #include <md_FRAM.h>
    #endif
  //
  // --- network
    #if (USE_WIFI > OFF)
      #include <md_webserver.h>
    #endif
  //
  // --- sensors
    #if (USE_DS18B20_1W > OFF)
        #include <OneWire.h>
        #include <DallasTemperature.h>
      #endif

    #if ( USE_BME280_I2C > OFF )
        #include <Adafruit_Sensor.h>
        #include <Adafruit_BME280.h>
      #endif
    #if ( USE_TYPE_K > OFF)
        #include <SPI.h>
        #include <md_31855_ktype.h>
      #endif

  //
  // -------------------------

#endif // MAIN_H