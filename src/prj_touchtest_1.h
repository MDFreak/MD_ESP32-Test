#ifndef _PRJ_TOUCHTEST_1_H_
  #define _PRJ_TOUCHTEST_1_H_

  #include <Arduino.h>
  #include <md_defines.h>

  // ******************************************
  // --- project configuration
    // PROJECT TOUCHTEST_1
      /* AZ-touch kit 2,4"
       * ESP32_Node
       * BME280 (I2C)
       * OLED 091 (I2C)
       * I2C  SDA=32 SCL=26
       */
    // --- board
      #define BOARD   MC_ESP32_Node     // platform=espressiv32, env=env:esp32dev, az-delivery-devkit-v4
      #if (BOARD == MC_ESP32_Node)
          #define PROJ_TITLE "ESP32-TEST using ESP32-Device-KitC 38 pins"
        #endif
    // --- debugging
      #define DEBUG_MODE      CFG_DEBUG_STARTUP
        //#define DEBUG_MODE      CFG_DEBUG_NONE
        //#define DEBUG_MODE      CFG_DEBUG_ACTIONS
        //#define DEBUG_MODE      CFG_DEBUG_DETAILS

    // --- SW config
      #define USE_TASKING           ON
      #define USE_LED_BLINK_OUT     ON
    // --- user output components
      #define USE_TRAFFIC_LED_OUT   OFF
      #define USE_RGBLED_PWM        OFF // 1
      #define USE_DISP_I2C1         1   // OFF // 1
      #define USE_DISP_I2C2         OFF
      #define USE_DISP_SPI          OFF
      #define USE_BUZZER_PWM        1
      #define USE_FAN_PWM           OFF // 2
      #define USE_OUT_FREQ_PWM      OFF // 1
      #define USE_WS2812_MATRIX_OUT OFF   // [0, 1..4]
      #define USE_WS2812_LINE_OUT   OFF     // [0, 1..4]
      #define USE_WS2812_PWR_IN_SW  OFF                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         // some time matrix and line not allowed
    // --- user input components
      #define USE_TOUCHSCREEN_SPI   1   // OFF
      #define USE_TOUCHSCREEN_IO    OFF
      #define USE_KEYPADSHIELD_ADC  OFF
      #define USE_GEN_CNT_INP       OFF // 1
      #define USE_GEN_PWM_INP       OFF // 2
    // --- sensors
      #define USE_DS18B20_1W_IO     OFF   // [0, 1, ....] limited by 1W connections
      #define USE_BME280_I2C        1     // [0, 1, ....] limited by I2C channels/addr
      #define USE_TYPE_K_SPI        OFF   // [0, 1, ....] limited by Pins
      #define USE_MQ135_GAS_ADC     OFF   // [0, 1, ....] limited by analog inputs
      #define USE_PHOTO_SENS        OFF // ON
    // --- network  components
      #define USE_WIFI              ON
      #define USE_NTP_SERVER        ON
      #define USE_LOCAL_IP          ON
      #define USE_WEBSERVER         ON
    // --- memory components
      #define USE_FLASH_MEM         ON
      #define USE_FRAM_I2C          OFF // 1   // [0, 1, ...] limited by I2C channel/addr
    // --- test components
      #define USE_CTRL_POTI_ADC     OFF   // [0, 1, ....] limited by analog inputs
      #define USE_CTRL_SW_INP       OFF // 1   // [0, 1, ....] limited by digital pins
    // --- system components
      #define USE_DISP_I2C          USE_DISP_I2C1 + USE_DISP_I2C2
    // usage of peripherals
      #define USE_I2C       USE_DISP_I2C
      #define USE_SPI       USE_DISP_SPI + USE_TOUCHSCREEN_SPI + USE_TYPE_K_SPI
      #define USE_PWM_OUT   3 * USE_RGBLED_PWM + USE_GEN_PWM_OUT + USE_OUT_FREQ_PWM + USE_BUZZER_PWM // max 16
      #define USE_CNT_INP   USE_GEN_CNT_INP     // max 2 * 8 independent
      #define USE_PWM_INP   USE_GEN_PWM_INP
      #define USE_ADC1      USE_KEYPADSHIELD_ADC + USE_MQ135_GAS_ADC + USE_CTRL_POTI_ADC + USE_PHOTO_SENS
      #define USE_ADC2      OFF // not to use
      #define USE_DIG_INP   USE_CTRL_SW_INP + USE_WS2812_PWR_IN_SW    //
      #define USE_DIG_OUT   USE_WS2812_LINE_OUT + USE_LED_BLINK_OUT //
      #define USE_DIG_IO    USE_DS18B20_1W_IO     //
      #if (USE_SPI > OFF)
          #define USED_SPI_PINS     USE_SPI + 3
        #else
          #define USED_SPI_PINS   OFF
        #endif
      #define USED_IOPINS         USE_DIG_INP + USE_DIG_OUT + USE_DIG_IO + (2 * USE_I2C) + USED_SPI_PINS + USE_PWM_OUT + USE_CNT_INP + USE_ADC1
      #if (USED_IOPINS > 15)
          #define ERROR !!! zuviele IOs verwendet !!!
          ERROR
        #endif
    // to be reorganised
      #define USE_DISP            USE_DISP_I2C + USE_DISP_SPI
        #if (USE_DISP > 0)
          // --- displays
              #define USE_OLED_I2C   1 // [0, 1, 2] are possible
                // OLEDs     MC_UO_OLED_066_AZ, MC_UO_OLED_091_AZ
                          // MC_UO_OLED_096_AZ, MC_UO_OLED_130_AZ
                #if (USE_OLED_I2C > OFF)
                    #define OLED1   MC_UO_OLED_096_AZ
                  #endif
                #if (USE_OLED_I2C > 1)
                    #define OLED2   TRUE
                    #define OLED2_MC_UO_OLED_130_AZ
                    #define OLED2_GEO    GEO_128_64
                  #endif

              #define USE_TFT        1
                // TFTs
                #if (USE_TFT > 0)
                    //#define DISP_TFT  MC_UO_TFT1602_GPIO_RO
                    #define DISP_TFT  MC_UO_TOUCHXPT2046_AZ
                    //#define DISP_TFT  MC_UO_TFT1602_I2C_XA
                  #endif
            #endif
      #define USE_AOUT            USE_BUZZER_PWM
        #if (USE_AOUT > OFF)
            // --- speakers ...
              #define USE_BUZZER_PWM     1     // [0, 1, ...] limited by PWM outputs
                #if (USE_BUZZER_PWM > OFF)
                    #define BUZZER1  AOUT_PAS_BUZZ_3V5V
                  #endif
          #endif
      #define USE_KEYPADSHIELD    USE_KEYPADSHIELD_ADC
        #if (USE_KEYPADSHIELD > OFF)
            #define USE_TFT1602_GPIO_RO_V5  // used by KEYPADSHIELD
            #define KEYS_Keypad_ANA0_RO_V5        // used by KEYPADSHIELD
            #define KEYS            ?
          #endif // USE_KEYPADSHIELD

      #define USE_TOUCHSCREEN     (3 * USE_TRAFFIC_LED_OUT) +USE_TOUCHSCREEN_SPI + USE_TOUCHSCREEN_OUT
        #if (USE_TOUCHSCREEN > OFF)
            #define TOUCHSCREEN1     TOUCHXPT2046_AZ_3V3
            #define TOUCHKEYS1       KEYS_TOUCHXPT2046_AZ_3V3
          #endif // USE_TOUCHSCREEN

#endif // _PRJ_CONFIG_H_