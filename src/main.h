#ifndef _MAIN_H_
  #define _MAIN_H_

  // --- includes
    #include <Arduino.h>
    #include <unity.h>
    #include <Wire.h>
    #include <string.h>
    #include <stdio.h>                                                        // Biblioteca STDIO
    #include <md_time.hpp>
    #include <md_defines.h>
    #include <md_util.h>
    #include <md_filter.hpp>
    #include <project.h>
      #if (PROJECT == TOUCHTEST_1)
          #include <prj_touchtest_1.h>
        #endif
      #if (PROJECT == MEASFREQ_1)
          #include <prj_measfreq_1.h>
        #endif
      #if (PROJECT == LIGHTSHOW_1)
          #include <prj_lightshow_1.h>
        #endif
      #if (PROJECT == GEN_ESP32_NODE)
          #include <prj_conf_gen_esp32_node.h>
        #endif
      #if (PROJECT == GEN_XIAO_ESP32C3)
          #include <prj_conf_gen_xiao_esp32c3.h>
        #endif
      #if (PROJECT == GEN_ESP32_D1_MINI)
          #include <prj_gen_esp32_d1_mini.h>
        #endif

    #include <prj_config.h>
        //#include <driver\gpio.h>
        //#include <driver\adc.h>
        //#include "freertos/task.h"
        //#include "freertos/queue.h"
        //#include "driver/ledc.h"
        //#include "driver/mcpwm.h"
    #include "driver/pcnt.h"
        //#include "esp_attr.h"
        //#include "esp_log.h"
  // --- system components
    #if (DEV_VSPI > OFF) || (DEV_HSPI > OFF)
        #include "spi.h"
      #endif // USE_TOUCHSCREEN

  // --- user inputs
    #if (USE_TOUCHSCREEN > OFF)
        #include "md_touch.h"
      #endif // USE_TOUCHSCREEN

    #if (USE_KEYPADSHIELD > OFF)
        #include "md_keypadshield.h"
      #endif // USE_KEYPADSHIELD

    #if (USE_CTRL_POTI > OFF)
        // nothing to do
      #endif

    #if (USE_DIG_INP > OFF)
        //#include <driver\gpio.h>
      #endif

    #if (USE_CNT_INP > OFF)
        #include <freertos/queue.h>
        #include <driver\pcnt.h>
        #include <esp_attr.h>
      #endif

    #if (USE_PWM_INP > OFF)
        #include <driver\mcpwm.h>
        #include <esp_attr.h>
      #endif

    #if (DEV_ADC_INT > OFF)
        #include <driver\adc.h>
      #endif
    #if (DEV_ADC_ADS1115 > OFF)
        #include <Adafruit_ADS1X15.h>
        #ifndef IRAM_ATTR
            #define IRAM_ATTR
          #endif
      #endif
  // --- user outputs
    // --- PWM
      /** ### Configure the project ------------------------

        - The example uses fixed PWM frequency of 5 kHz, duty cycle in 50%,
          and output GPIO pin.
          To change them, adjust `LEDC_FREQUENCY`, `LEDC_DUTY`,
          `LEDC_OUTPUT_IO` macros at the top of ledc_basic_example_main.c.

        - Depending on the selected `LEDC_FREQUENCY`,
          you will need to change the `LEDC_DUTY_RES`.

        - To dynamicaly set the duty and frequency,
          you can use the following functions:
          - To set the frequency to 2.5 kHZ i.e:
            ```c
            ledc_set_freq(LEDC_MODE, LEDC_TIMER, 2500);
            ```
          - Now the duty to 100% i.e:
            ```c
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 8191);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            ```
        - To change the duty cycle you need to calculate
          the duty range according to the duty resolution.
          - If duty resolution is 13 bits:
            Duty range: `0 to (2 ** 13) - 1 = 8191` where 0 is 0% and 8191 is 100%.
       **/
    #if (DEV_PWM_OUTPUT > OFF)
        #include <driver\ledc.h>
        #include <md_leds.h>
        typedef struct
          {
            uint16_t red;
            uint16_t green;
            uint16_t blue;
          } outRGBVal_t;
      #endif
    #if (BUZZER1 > OFF)
        #include "md_buzzer.h"
      #endif // USE_BUZZER_PWM

    #if (USE_OLED_I2C > OFF)
        #if !(OLED1_DRV ^ OLED_DRV_1106)
            #include "md_oled_SH1106.h"
        #else
            #include <md_oled.h>
          #endif
        #if (USE_OLED_I2C > 1)
            #if !(OLED2_DRV ^ OLED_DRV_1106)
                #include "md_oled_SH1106.h"
            #else
                #include <md_oled.h>
              #endif
          #endif
      #endif

    #if (USE_WS2812_MATRIX_OUT > OFF)
        #include <md_leds.h>
      #endif

    #if (USE_WS2812_LINE_OUT > OFF)
        #include <Adafruit_NeoPixel.h>
      #endif

    #if (USE_DISP_TFT > OFF)
        #if !(USE_DISP_TFT ^ MC_UO_TFT1602_GPIO_RO)
            #include "md_lcd.h"
          #endif
        #if !(DISP_TFT ^ MC_UO_TOUCHXPT2046_AZ)
            #include <md_touch.h>
          #endif
      #endif

    #if (USE_CNT_INP > OFF)
        static void initGenPCNT();
        void getCNTIn();
      #endif
  // --- user inputs
    #if (USE_CNT_INP > OFF)
      #endif
  // --- memory
    #if (USE_FLASH_MEM > OFF)
        #include <SPIFFS.h>
        #include <md_spiffs.h>
      #endif

    #if (USE_FRAM_I2C > OFF)
        #include <md_FRAM.h>
      #endif

    #if (USE_SD_SPI > OFF)
        #include <sd.h>
      #endif
  // --- network
    #if (USE_WIFI > OFF)
        #include <AsyncTCP.h>
        #include <ESPAsyncWebServer.h>
        #include <md_webserver.h>
        #include <ip_list.hpp>
        #if (USE_MQTT > OFF)
            //#include <espMqttClient.h>
            #include <md_eMQTT5.hpp>
          #endif
      #endif
  // --- sensors
    #if ( USE_DS18B20_1W_IO > OFF )
        #include <OneWire.h>
        #include <DallasTemperature.h>
      #endif

    #if ( USE_BME280_I2C > OFF )
        #include <Adafruit_Sensor.h>
        #include <Adafruit_BME280.h>
      #endif
    #if ( USE_TYPE_K_SPI > OFF )
        #include <md_31855_ktype.h>
      #endif

  // ---------------------------------------
  // --- prototypes
    // ------ system -------------------------
      // --- heap ------------------------
        void heapFree(const char* text);
    // ------ user interface -----------------
      // --- user output
        // --- display
          void clearDisp();
          void dispStatus(String msg, bool direct = false);
          void dispStatus(const char* msg, bool direct = false);
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
          #if (USE_MQ3_ALK_ANA > OFF)
              int16_t showTrafficLight(int16_t inval, int16_t inthres);
            #endif

        // WS2812 LED
          #if (USE_WS2812_LINE_OUT > OFF)
              void initWS2812Line();
              void FillLEDsFromPaletteColors(uint8_t lineNo, uint8_t colorIndex);
              void ChangePalettePeriodically(uint8_t lineNo);
              #ifdef XXXX
                  void SetupTotallyRandomPalette();
                  void SetupBlackAndWhiteStripedPalette();
                  void SetupPurpleAndGreenPalette();
                #endif
            #endif

          #if (USE_WS2812_MATRIX_OUT > OFF)
              void initWS2812Matrix();
            #endif

      // --- user input
        // --- keypad
          #if defined(KEYS)
              void startKeys();
              uint8_t getKey();
            #endif
        // --- digital input
          #if (USE_DIG_INP > OFF)
              void getDIGIn();
            #endif
          #if (USE_CTRL_POTI > OFF)
              void getADCIn();
            #endif
        // --- counter input
          #if (USE_CNT_INP > OFF)
              static void initPCNT();
              void getCNTIn();
            #endif
      // --- sensors
        // --- DS18B20
          #if (USE_DS18B20_1W_IO > OFF)
              String getDS18D20Str();
            #endif
          #if (USE_ADC1115_I2C > OFF)
              static void init1115_chan(uint8_t unit, uint8_t chan, uint8_t mode, uint8_t att);
            #endif
        // --- MQ135 gas sensor
          #if (USE_MQ135_GAS_ANA > OFF)
              int16_t getGasValue();
              int16_t getGasThres();
            #endif

        // --- T-element type K

    // ----- memory ---------------------------
          #if (USE_FLASH_MEM > OFF)
              void testFlash();
            #endif

    // ------ network -------------------------
      // --- WIFI
        #if (USE_WIFI > OFF)
            uint8_t startWIFI(bool startup);
            #if (USE_NTP_SERVER > OFF)
                void    initNTPTime();
              #endif
          #endif

      // --- webserver
        #if (USE_WEBSERVER > OFF)
            //void handlingIncomingData(AsyncWebSocketClient *client, void *arg, uint8_t *data, size_t len);
            //void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type,
            //             void * arg, uint8_t *data, size_t len);
            //void configWebsite();
            void startWebServer();
            void readMessage();
            void sendMessage();
          #endif
      // --- MQTT
        #if (USE_MQTT > OFF)
            void connectToMqtt();
            void onMqttConnect(bool sessionPresent);
            //void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
            void onMqttSubscribe(uint16_t packetId, uint8_t qos);
            void onMqttUnsubscribe(uint16_t packetId);
            //void onMqttMessage(char* topic, char* payload,
                               //AsyncMqttClientMessageProperties properties,
                               //size_t len, size_t index, size_t total);
            void onMqttPublish(uint16_t packetId);
          #endif
    // -------------------------

#endif // MAIN_H