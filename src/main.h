#ifndef _MAIN_H_
  #define _MAIN_H_

  #include <Arduino.h>
  #include <unity.h>
  #include <Wire.h>
  #include <string.h>
  #include <stdio.h>                                                        // Biblioteca STDIO
  #include <md_time.hpp>
  #include <md_defines.h>
  #include <md_util.h>
  #include <ip_list.hpp>
  #include <md_filter.hpp>
  #include <project.h>
  #include <prj_config.h>
  //#include <driver\gpio.h>
  #include <driver\adc.h>

  // --- system components
    #if (USE_PWM_OUT > OFF)
        #include <driver\ledc.h>
      #endif

  // --- user inputs
    #if (USE_TOUCHSCREEN > OFF)
        #include "md_touch.h"
      #endif // USE_TOUCHSCREEN

    #if (USE_KEYPADSHIELD > OFF)
        #include "md_keypadshield.h"
      #endif // USE_KEYPADSHIELD

    #if (USE_CTRL_POTI_ADC > OFF)
        // nothing to do
      #endif

    #if (USE_DIG_INP > OFF)
        //#include <driver\gpio.h>
      #endif

    #if (USE_CNT_INP > OFF)
        #include <driver\pcnt.h>

        #define PCNT_COUNT_UNIT       PCNT_UNIT_0                                 // Unidade 0 do Contador de pulso PCNT do ESP32
        #define PCNT_COUNT_CHANNEL    PCNT_CHANNEL_0                              // Canal 0 do Contador de pulso PCNT do ESP32
        //#define PCNT_INPUT_SIG_IO     GPIO_NUM_34                                 // Entrada do Frequencimetro -  GPIO 34
        //#define LEDC_HS_CH0_GPIO      GPIO_NUM_33                                 // Saida do LEDC - gerador de pulsos - GPIO_33
        //#define PCNT_INPUT_CTRL_IO    GPIO_NUM_35                                 // Pino de controle do PCNT - HIGH = count up, LOW = count down
        //#define OUTPUT_CONTROL_GPIO   GPIO_NUM_32                                 // Saida do timer - Controla a contagem - GPIO_32
        #define PCNT_H_LIM_VAL        overflow                                    // Limite superior de contagem
      #endif

    #if ((USE_ADC1 > OFF) || (USE_ADC2 > OFF))
        #include <driver\adc.h>
      #endif

  // --- user outputs
    #if (USE_RGBLED > OFF)
        typedef struct
          {
            uint16_t red;
            uint16_t green;
            uint16_t blue;
          } outRGBVal_t;
      #endif
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

    #if (USE_WS2812_LINE_OUT > OFF)
        #include <FastLED.h>
      #endif

    #if (USE_TFT > OFF)
        #include "md_lcd.h"
      #endif

  // --- memory
    #if (USE_FRAM_I2C > OFF)
        #include <md_FRAM.h>
      #endif
  // --- network
    #if (USE_WIFI > OFF)
      #include <md_webserver.h>
    #endif
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
          #if (USE_MQ135_GAS_ADC > OFF)
              int16_t showTrafficLight(int16_t inval, int16_t inthres);
            #endif

        // WS2812 LEDs
          #if (USE_WS2812_LINE_OUT > OFF)
              void FillLEDsFromPaletteColors( uint8_t colorIndex);
              void ChangePalettePeriodically();
              void SetupTotallyRandomPalette();
              void SetupBlackAndWhiteStripedPalette();
              void SetupPurpleAndGreenPalette();
            #endif

      // --- user input
        // --- keypad
          #if defined(KEYS)
              void startKeys();
              uint8_t getKey();
            #endif
        // --- digital input
          #if (USE_CTRL_SW_INP > OFF)
              uint8_t getSWIn();
            #endif
          #if (USE_CTRL_POTI_ADC > OFF)
              uint16_t getPotiIn();
            #endif
        // --- counter input
          #if (USE_CNT_INP > OFF)
              int16_t getCntValue();
            #endif
      // --- sensors
        // --- DS18B20
          #if (USE_DS18B20_1W > OFF)
              String getDS18D20Str();
            #endif
        // --- BME280
          #if ( USE_BME280_I2C > OFF )
              String getBME280Str();
            #endif
        // --- MQ135 gas sensor
          #if (USE_MQ135_GAS_ADC > OFF)
              int16_t getGasValue();
              int16_t getGasThres();
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