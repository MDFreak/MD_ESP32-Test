#ifndef _PROJECT_H_
  #define _PROJECT_H_

  #include <Arduino.h>
  #include <md_defines.h>

  // ******************************************
  // --- project configuration
    #define PROJ_TITLE "ESP32-TEST"
    // --- debugging
      #define DEBUG_MODE      CFG_DEBUG_STARTUP
        //#define DEBUG_MODE      CFG_DEBUG_NONE
        //#define DEBUG_MODE      CFG_DEBUG_ACTIONS
        //#define DEBUG_MODE      CFG_DEBUG_DETAILS

    // --- board
      #define BOARD   MC_ESP32_Node     // platform=espressiv32, env=env:esp32dev, az-delivery-devkit-v4
        //#define BOARD   MC_ESP32_D1_R32     // platform=espressiv32, env=env:esp32dev, az-delivery-devkit-v4
        //#define BOARD   MC_ESP32_D1_MINI  // platform=espressiv32, env=env:esp32dev, az-delivery-devkit-v4
        //#define BOARD   MC_AV_NANO_V3
        //#define BOARD   MC_AV_UNO_V3

    // --- system components
      #define USE_TASKING           ON
      #define USE_LED_BLINK         OFF //ON
      #define USE_I2C                1     // [0, 1, 2] limited by board
      #define USE_SPI                1
    // --- network  components
      #define USE_WIFI              TRUE
      #define USE_NTP_SERVER        TRUE
      #define USE_LOCAL_IP          TRUE
      #define USE_WEBSERVER         TRUE
    // --- user output components
      #define USE_LEDS               3
      #define USE_TRAFFIC_LIGHT     OFF
      #define USE_RGBLED             0
      #define USE_DISP               1
        #if (USE_DISP > 0)
          // --- displays
              #define USE_OLED_I2C   1 // [0, 1, 2] are possible
                // OLEDs     MC_UO_OLED_066_AZ, MC_UO_OLED_091_AZ
                          // MC_UO_OLED_096_AZ, MC_UO_OLED_130_AZ
                #if (USE_OLED_I2C > OFF)
                    #define OLED1   MC_UO_OLED_130_AZ
                  #endif
                #if (USE_OLED_I2C > 1)
                    #define OLED2   TRUE
                    #define OLED2_MC_UO_OLED_130_AZ
                    #define OLED2_GEO    GEO_128_64
                  #endif

              #define USE_TFT        0
                // TFTs
                #if (USE_TFT > 0)
                    //#define DISP_TFT  MC_UO_TFT1602_GPIO_RO
                    //#define DISP_TFT  MC_UO_TOUCHXPT2046_AZ
                    //#define DISP_TFT  MC_UO_TFT1602_I2C_XA
                  #endif
            #endif
      #define USE_AOUT              OFF
        #if (USE_AOUT > OFF)
          // --- speakers ...
              #define USE_BUZZER     1     // [0, 1, ...] limited by PWM outputs
                #if (USE_BUZZER > OFF)
                    #define BUZZER1  AOUT_PAS_BUZZ_3V5V
                  #endif
            #endif

    // --- user input components
      #define USE_TOUCHSCREEN       FALSE
        #if (USE_TOUCHSCREEN > OFF)
            #define TOUCHSCREEN1     TOUCHXPT2046_AZ_3V3
            #define TOUCHKEYS1       KEYS_TOUCHXPT2046_AZ_3V3
          #endif // USE_TOUCHSCREEN

      #define USE_KEYPADSHIELD      FALSE
        #if (USE_KEYPADSHIELD > OFF)
            #define USE_TFT1602_GPIO_RO_V5  // used by KEYPADSHIELD
            #define KEYS_Keypad_ANA0_RO_V5        // used by KEYPADSHIELD
            #define KEYS            ?
          #endif // USE_KEYPADSHIELD

    // --- sensors
      #define USE_DS18B20_1W        0   // [0, 1, ....] limited by 1W connections
      #define USE_BME280_I2C        1   // [0, 1, ....] limited by I2C channels/addr
      #define USE_TYPE_K            0   // [0, 1, ....] limited by Pins
      #define USE_MQ135_GAS_ANA     0   // [0, 1, ....] limited by analog inputs
      #define USE_WS2812_LINE       1   // [0, 1, ....] limited by
    // --- memory components
      #define USE_FRAM_I2C          0   // [0, 1, ...] limited by I2C channel/addr

#endif // _PRJ_CONFIG_H_