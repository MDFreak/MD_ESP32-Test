#ifndef _MAIN_H_
  #define _MAIN_H_

  #include <Arduino.h>
  #include <unity.h>
  #include <Wire.h>
  #include <string.h>
  #include <md_time.hpp>
  #include <md_defines.h>
  #include <md_util.h>
  #include <ip_list.hpp>
  #include <md_filter.hpp>
  #include <project.h>
  #include <prj_config.h>

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

    #if (USE_WS2812_LINE >OFF)
        #include <FastLED.h>
      #endif

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

  // ---------------------------------------
  // --- prototypes
    // ------ user interface -----------------
      // --- user output
        // --- display
          void clearDisp();
          void dispStatus(String msg);
          void dispStatus(const char* msg);
          void dispText(char* msg, uint8_t col, uint8_t row, uint8_t len);
          void dispText(String msg, uint8_t col, uint8_t row, uint8_t len);
          void startDisp();

        // --- passive buzzer
          #ifdef PLAY_MUSIC
              void playSong(int8_t songIdx);
              void playSong();
            #endif

        // --- traffic Light of gas sensor
          #if (USE_MQ135_GAS_ANA > OFF)
              int16_t showTrafficLight(int16_t inval, int16_t inthres);
            #endif

        // WS2812 LEDs
          #if (USE_WS2812_LINE > OFF)
              void FillLEDsFromPaletteColors( uint8_t colorIndex);
              void ChangePalettePeriodically();
              void SetupTotallyRandomPalette();
              void SetupBlackAndWhiteStripedPalette();
              void SetupPurpleAndGreenPalette();
            #endif

      // --- user input
        // --- keypad
          void startKeys();
          uint8_t getKey();
      // --- sensors
        // --- DS18B20
          #if (USE_DS18B20_1W > OFF)
              String getDS18D20Str();
            #endif
        // --- BME280
          #if ( USE_BME280_I2C > OFF )
              String getBME280Str();
            #endif
        // --- T-element type K
    // ------ network -------------------------
      // --- WIFI
        #if (USE_WIFI > OFF)
            void startWIFI(bool startup);
            void initNTPTime();
          #endif

      // --- webserver
        #if (USE_WEBSERVER > OFF)
            void configWebsite();
            void startWebServer();
          #endif

    // -------------------------

#endif // MAIN_H