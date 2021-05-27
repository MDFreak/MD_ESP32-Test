#ifndef _PRJ_CONFIG_H_
  #define _PRJ_CONFIG_H_

  #include <Arduino.h>
  #include <md_defines.h>

  #define PROJ_TITLE "   ESP32 TEST"

  // ******************************************
  // --- system defines
    #define SER_BAUDRATE  115200ul
    #define GEO_128_64    0
    #define GEO_128_32    1
    #define GEO_64_48     2
    #define GEO_64_32     3
    #define GEO_RAWMODE   4

  // ******************************************
  // --- configuration selections
    // --- debugging
      //#define DEBUG_MODE      CFG_DEBUG_NONE
      #define DEBUG_MODE      CFG_DEBUG_STARTUP
      //#define DEBUG_MODE      CFG_DEBUG_ACTIONS
      //#define DEBUG_MODE      CFG_DEBUG_DETAILS

    // --- board
      //#define BOARD   MC_ESP32_D1_R32     // platform=espressiv32, env=env:esp32dev, az-delivery-devkit-v4
      //#define BOARD   MC_AV_NANO_V3
      //#define BOARD   MC_AV_UNO_V3
      #define BOARD   MC_ESP32_Node     // platform=espressiv32, env=env:esp32dev, az-delivery-devkit-v4
      //#define BOARD   MC_ESP32_D1_MINI  // platform=espressiv32, env=env:esp32dev, az-delivery-devkit-v4

    // --- components
      // --- system
        #define USE_TASKING           ON
        #define USE_LED_BLINK         ON
        #define USE_I2C                1     // [0, 1, 2] limited by board
        #define USE_SPI                1
      // --- network
        #define USE_WIFI              TRUE
        #define USE_NTP_SERVER        TRUE
        #define USE_LOCAL_IP          TRUE
        #define USE_WEBSERVER         TRUE
      // --- user output
        #define USE_LEDS               3
        #define USE_TRAFFIC_LIGHT     OFF
        #define USE_RGBLED             1
        #define USE_DISP               1
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

      // --- user input
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
        #define USE_DS18B20_1W        1   // [0, 1, ....] limited by 1W connections
        #define USE_BME280_I2C        1   // [0, 1, ....] limited by I2C channels/addr
        #define USE_TYPE_K            2   // [0, 1, ....] limited by Pins
        #define USE_MQ135_GAS_ANA     1   // [0, 1, ....] limited by analog inputs

      // --- memories
        #define USE_FRAM_I2C        1  // [0, 1, ...] limited by I2C channel/addr

  // ******************************************
  // --- board management
    #if !(BOARD ^ MC_ESP32_Node)
      // --- system
      // --- network
      // --- user output
        // --- LEDs
        // --- display
        // --- acustic output
      // --- user input
      // --- sensors
      // --- memories
      // --- pins, connections
        // --- system
          #if (USE_LED_BLINK > 0)
              #define PIN_BOARD_LED   1
            #endif
        // --- user output
          #if (USE_LEDS > 0)
              // traffic lamp
              #if (USE_TRAFFIC_LIGHT > 0)
                  #define PIN_TL_RED    26   // RGB red
                  #define PIN_TL_YELLOW 25   // RGB green
                  #define PIN_TL_GREEN  33   // RGB blue
                #endif
              #if (USE_RGBLED > 0)
                  #define PIN_RGB_RED    26   // RGB red
                  #define PIN_RGB_GREEN  25   // RGB blue
                  #define PIN_RGB_BLUE   33   // RGB blue
                #endif
            #endif
          #if (USE_TFT > 0)
              #if !(DISP_TFT ^ MC_UO_TFT1602_GPIO_RO)
                  #define LCD_BL      5    // D10/SS  ARDUINO
                  #define LCD_EN      13   // D9
                  #define LCD_RS      12   // D8
                  #define LCD_D7      14   // D7
                  #define LCD_D6      27   // D6
                  #define LCD_D5      16   // D5
                  #define LCD_D4      17   // D4
                #endif
            #endif
          #if (USE_BUZZER > OFF)
              #define PIN_BUZZ      32
            #endif
        // --- sensors
          #if (USE_DS18B20_1W > OFF)
              #define DS_ONEWIRE_PIN 27
            #endif
          #if (USE_TYPE_K > 0)
              #define TYPEK_DATA_PIN 19   // SPI MISO
              #define TYPEK_CLK_PIN  18   // SPI CLK
              #define TYPEK1_CS_PIN  16
              #define TYPEK2_CS_PIN  17
            #endif
          #if (USE_MQ135_GAS_ANA > OFF)
              #define PIN_MQ135     36
            #endif

        // --- PWM
          // --- channels
            #define PWM_BUZZ          0
            #if (USE_TRAFFIC_LIGHT > 0)
                #define PWM_TL_GREEN      1
                #define PWM_TL_YELLOW     2
                #define PWM_TL_RED        3
              #endif
            #if(USE_RGBLED >0)
                #define PWM_RGB_RED       4
                #define PWM_RGB_GREEN     5
                #define PWM_RGB_BLUE      6
              #endif
        // --- I2C
          // --- board connection
            #define USE_I2C1          1
            #define PIN_I2C1_SDA      21
            #define PIN_I2C1_SCL      22
            #if ( USE_I2C > 1 )
                #define USE_I2C2      2
                #define PIN_I2C2_SDA  25
                #define PIN_I2C2_SCL  26
              #endif

      // --- I2C
        // --- board connection
          #define USE_I2C1          1
          #define PIN_I2C1_SDA      21
          #define PIN_I2C1_SCL      22
          #if ( USE_I2C > 1 )
              #define USE_I2C2      2
              #define PIN_I2C2_SDA  25
              #define PIN_I2C2_SCL  26
            #endif
      #endif

    #if !(BOARD ^ MC_ESP32_D1_MINI)
      // --- system
      // --- network
      // --- user output
        // --- LEDs
        // --- display
        // --- acustic output
      // --- user input
      // --- sensors
      // --- memories
      // --- pins, connections
        // --- system
          #if (USE_LED_BLINK > 0)
              #define PIN_BOARD_LED   1
            #endif

        // --- user output
          #if (USE_LEDS > 0)
              // traffic lamp
              #define PIN_TL_RED    26
              #define PIN_TL_YELLOW 25
              #define PIN_TL_GREEN  33
            #endif
          #if (USE_TFT > 0)
              #if !(DISP_TFT ^ MC_UO_TFT1602_GPIO_RO)
                  #define LCD_BL      5    // D10/SS  ARDUINO
                  #define LCD_EN      13   // D9
                  #define LCD_RS      12   // D8
                  #define LCD_D7      14   // D7
                  #define LCD_D6      27   // D6
                  #define LCD_D5      16   // D5
                  #define LCD_D4      17   // D4
                #endif
            #endif
          #if (USE_BUZZER > OFF)
              #define PIN_BUZZ      32
            #endif
        // --- sensors
          #if (USE_DS18B20_1W > OFF)
              #define DS_ONEWIRE_PIN 27
            #endif
          #if (USE_MQ135_GAS_ANA > OFF)
              #define PIN_MQ135     36
            #endif

        // --- PWM
          // --- channels
            #define PWM_BUZZ          0
            #define PWM_TL_GREEN      1
            #define PWM_TL_YELLOW     2
            #define PWM_TL_RED        3

        // --- I2C
          // --- board connection
            #define USE_I2C1          1
            #define PIN_I2C1_SDA      21
            #define PIN_I2C1_SCL      22
            #if ( USE_I2C > 1 )
                #define USE_I2C2      2
                #define PIN_I2C2_SDA  25
                #define PIN_I2C2_SCL  26
              #endif
      #endif

    #if !(BOARD ^ MC_ESP32_D1_R32)
      // --- system
      // --- network
      // --- user output
        // --- LEDs
        // --- display
        // --- acustic output
      // --- user input
      // --- sensors
      // --- memories
      // --- pins, connections
        // --- system
          #if (USE_LED_BLINK > 0)
              #define PIN_BOARD_LED   2
            #endif

        // --- user output
          #if (USE_LEDS > 0)
              // traffic lamp
              #define PIN_TL_RED    17
              #define PIN_TL_YELLOW 25
              #define PIN_TL_GREEN  26
            #endif
          #ifdef USE_TFT
              #if !(DISP_TFT ^ MC_UO_TFT1602_GPIO_RO)
                  #define LCD_BL      5    // D10/SS  ARDUINO
                  #define LCD_EN      13   // D9
                  #define LCD_RS      12   // D8
                  #define LCD_D7      14   // D7
                  #define LCD_D6      27   // D6
                  #define LCD_D5      16   // D5
                  #define LCD_D4      17   // D4
                #endif
            #endif
          #if defined(USE_BUZZER)
              #define PIN_BUZZ      27
            #endif
        // --- sensors
          #if (USE_DS18B20_1W > OFF)
              #define DS_ONEWIRE_PIN 27
            #endif
          #if (USE_MQ135_GAS_ANA > OFF)
              #define PIN_MQ135     34
              #define PIN_CO2_THOLD 35     // analog threshold for evaluation
            #endif

        // --- PWM
          // --- channels
            #define PWM_BUZZ          0
            #define PWM_TL_GREEN      1
            #define PWM_TL_YELLOW     2
            #define PWM_TL_RED        3

        // --- I2C
          // --- board connection
            #define USE_I2C1          1
            #define PIN_I2C1_SDA      21
            #define PIN_I2C1_SCL      22
            #if ( USE_I2C > 1 )
                #define USE_I2C2      2
                #define PIN_I2C2_SDA  25
                #define PIN_I2C2_SCL  26
              #endif
      #endif

    #if !(BOARD ^ MC_AV_NANO_V3)
      #endif

    #if !(BOARD ^ MC_AV_UNO_V3)
      #endif // BOARD

  // ******************************************
  // --- specification
    // --- system
      // --- error status bits
        #define ERRBIT_TOUCH     0x00000001     // touchscreen
        #define ERRBIT_SERVER    0x00000002     // webserver
        #define ERRBIT_WIFI      0x00000004     // WIFI connection
        #define ERRBIT_NTPTIME   0x00000008     // NTP timeserver connection
      // --- generic
        #define SCAN_I2C   128
        #define CHECK_I2C_DEVICES
        //#define UTC_SEASONTIME UTC_WINTERTIME
        #define UTC_SEASONTIME UTC_SUMMERTIME
        #if (USE_TASKING > 0)
            #define USE_SONGTASK
            #define TASK_SLICE_T  5000ul   // task beat [us]
          #endif // USE_TASKING
        #if (USE_LED_BLINK > 0)
            #define BLINKTIME_MS  1500ul
          #endif

    // --- I2C interface
      // --- address configuration
        #if (USE_OLED_I2C > OFF)
            #define I2C_ADDR_OLED1      I2C_OLED_3C
            #define I2C_OLED1_USE_I2C1
            #if defined( I2C_OLED1_USE_I2C1 )
                #define I2C_OLED1       I2C1
                #define I2C_SCL_OLED1   PIN_I2C1_SCL
                #define I2C_SDA_OLED1   PIN_I2C1_SDA
              #else
                  #define I2C_OLED1     I2C
                  #define I2C_SCL_OLED1 PIN_I2C2_SCL
                  #define I2C_SDA_OLED1 PIN_I2C2_SDA
              #endif
            #if (( USE_I2C > 1 ) && ( USE_OLED_I2C > 1 ))
                #define I2C_ADDR_OLED2      I2C_OLED
                #define I2C_OLED2_USE_I2C2
                  #if defined( I2C_OLED2_USE_I2C1 )
                    #define I2C_OLED2       I2C1
                    #define I2C_SCL_OLED2   PIN_I2C1_SCL
                    #define I2C_SDA_OLED2   PIN_I2C1_SDA
                  #else
                      #define I2C_OLED2     I2C2
                      #define I2C_SCL_OLED2 PIN_I2C2_SCL
                      #define I2C_SDA_OLED2 PIN_I2C2_SDA
                    #endif
              #endif
          #endif

        #if (USE_FRAM_I2C > OFF)
            #define I2C_ADDR_FRAM1          I2C_FRAM_50
            #define I2C_FRAM1_USE_I2C1
              #if defined( I2C_FRAM1_USE_I2C1 )
                #define I2C_FRAM1           I2C1
                #define I2C_SCL_FRAM1       PIN_I2C1_SCL
                #define I2C_SDA_FRAM1       PIN_I2C1_SDA
              #else
                  #define I2C_FRAM1         I2C2
                  #define I2C_SCL_FRAM1     PIN_I2C2_SCL
                  #define I2C_SDA_FRAM1     PIN_I2C2_SDA
                #endif
            #if (( USE_I2C > 1 ) && ( USE_FRAM_I2C > 1 ))
              #define I2C_ADDR_FRAM2        I2C_FRAM_50
                #define I2C_FRAM2_USE_I2C2
                  #if defined( I2C_FRAM2_USE_I2C1 )
                    #define I2C_FRAM2       I2C1
                    #define I2C_SCL_FRAM2   I2C1_SCL
                    #define I2C_SDA_FRAM2   I2C1_SDA
                  #else
                      #define I2C_FRAM2     I2C2
                      #define I2C_SCL_FRAM2 I2C2_SCL
                      #define I2C_SDA_FRAM2 I2C2_SDA
                    #endif
              #endif
          #endif

        #if ( USE_BME280_I2C > OFF )
            #define I2C_ADDR_BME2801        I2C_BME280
            #define I2C_BME2801_USE_I2C1
              #if defined( I2C_BME2801_USE_I2C1 )
                #define I2C_BME2801         I2C1
                #define I2C_SCL_BME2801     PIN_I2C1_SCL
                #define I2C_SDA_BME2801     PIN_I2C1_SDA
              #else
                  #define I2C_BME2801       I2C2
                  #define I2C_SCL_BME2801   PIN_I2C2_SCL
                  #define I2C_SDA_BME2801   PIN_I2C2_SDA
                #endif
            #if (( USE_I2C > 1 ) && ( USE_BME280_I2C > 1 ))
                #define I2C_ADDR_BME2802      I2C_BME280
                #define I2C_BME2802_USE_I2C2
                  #if defined( I2C_BME2802_USE_I2C1 )
                    #define I2C_BME2802       I2C1
                    #define I2C_SCL_BME2802   I2C1_SCL
                    #define I2C_SDA_BME2802   I2C1_SDA
                  #else
                      #define I2C_BME2802     I2C2
                      #define I2C_SCL_BME2802 I2C2_SCL
                      #define I2C_SDA_BME2802 I2C2_SDA
                    #endif
              #endif
          #endif

    // --- user interface
      // --- display output
        #define DISP_CYCLE       500ul   // Intervallzeit [us]
        // output status line
        #define STAT_TIMEDEF     5000u    // default time to clear status

    // --- network
      // --- WIFI
        #if (USE_WIFI > OFF)
            #define WIFI_ANZ_LOGIN  3
            #define WIFI_SSID0      "MAMD-HomeG" // WLAN Moosgrabenstrasse 26
            #define WIFI_SSID0_PW   "ElaNanniRalf3"
            #define WIFI_SSID1      "HS-HomeG" // WLAN Am Jungberg 9
            #define WIFI_SSID1_PW   "ElaNanniRalf3"
            #define WIFI_SSID2      "FairHandeln" //Weltladen
            #define WIFI_SSID2_PW   "WL&Fair2Live#"
            #define WIFI_CONN_DELAY 500000ul // Scan-Abstand [us]
            #define WIFI_CONN_REP   5        // Anzahle der Connect-Schleifen
            #define WIFI_CONN_CYCLE 4000ul   // Intervallzeit fuer Recoonect [us]
            #define NTPSERVER_CYCLE 1000ul   // Intervallzeit [us]

            #define WIFI_ANZ_LOCIP  WIFI_ANZ_LOGIN
            #if !(BOARD ^ MC_ESP32_Node)
                #define WIFI_FIXIP0     0x1400000Aul // 10.0.0.20   lowest first
            #elif !(BOARD ^ MC_ESP32_D1_MINI)
                #define WIFI_FIXIP0     0x1600000Aul // 10.0.0.22   lowest first
            #elif !(BOARD ^ MC_ESP32_D1_R32)
                #define WIFI_FIXIP0     0x1A00000Aul // 10.0.0.26   lowest first
              #endif
            #define WIFI_GATEWAY0   0x8B00000Aul // 10.0.0.139
            #define WIFI_FIXIP1     0x1400000Aul // 10.0.0.20
            #define WIFI_GATEWAY1   0x8B00000Aul // 10.0.0.139
            #define WIFI_FIXIP2     0x1400000Aul // 10.0.0.20
            #define WIFI_GATEWAY2   0x8a00000Aul // 10.0.0.138
            #define WIFI_SUBNET     0x00FFFFFFul // 255.255.255.0
          #endif

      // --- webserver
        #if (USE_WEBSERVER > OFF)
            #define WEBSERVER_CYCLE 1000ul  // Intervallzeit [us]
          #endif

    // --- user output
      // --- system
        #if (USE_LEDS > 0)
            #define PWM_LEDS_FREQ      5000u
            #define PWM_LEDS_RES       8
          #endif
      // --- display
        #if (USE_DISP > 0)
            #define USE_STATUS
            #if (USE_OLED_I2C > OFF)
                #if ( OLED1  > OFF )
                    #define USE_STATUS1
                          // select OLED - device & GEO   GEO_RAWMODE?
                          // MC_UO_OLED_066_AZ   GEO_64_48    OLED_DRV_1306
                          // MC_UO_OLED_091_AZ   GEO_128_32   OLED_DRV_1306
                          // MC_UO_OLED_096_AZ   GEO_128_64   OLED_DRV_1306
                          // MC_UO_OLED_130_AZ   GEO_128_64   OLED_DRV_1106
                    #if !(OLED1 ^ MC_UO_OLED_066_AZ)
                        #define DISP1_MAXCOLS  OLED_066_MAXCOLS
                        #define DISP1_MAXROWS  OLED_066_MAXROWS
                        #define OLED1_GEO      GEO_64_48
                        #define OLED1_DRV      OLED_DRV_1306
                      #endif
                    #if !(OLED1 ^ MC_UO_OLED_091_AZ)
                        #define DISP1_MAXCOLS  OLED_091_MAXCOLS
                        #define DISP1_MAXROWS  OLED_091_MAXROWS
                        #define OLED1_GEO      GEO_128_32
                        #define OLED1_DRV      OLED_DRV_1306
                      #endif
                    #if !(OLED1 ^ MC_UO_OLED_096_AZ)
                        #define DISP1_MAXCOLS  OLED_096_MAXCOLS
                        #define DISP1_MAXROWS  OLED_096_MAXROWS
                        #define OLED1_GEO      GEO_128_64
                        #define OLED1_DRV      OLED_DRV_1306
                      #endif
                    #if !(OLED1 ^ MC_UO_OLED_130_AZ)
                        #define DISP1_MAXCOLS  OLED_130_MAXCOLS
                        #define DISP1_MAXROWS  OLED_130_MAXROWS
                        #define OLED1_GEO      GEO_128_64
                        #define OLED1_DRV      OLED_DRV_1106
                      #endif
                  #endif
                #if defined( OLED2 )
                    #define USE_STATUS2
                    #if !(OLED2 ^ MC_UO_OLED_066_AZ)
                        #define DISP2_MAXCOLS  OLED_066_MAXCOLS
                        #define DISP2_MAXROWS  OLED_066_MAXROWS
                        #define OLED2_GEO      GEO_64_48
                        #define OLED2_DRV      OLED_DRV_1306
                      #endif
                    #if !(OLED2 ^ MC_UO_OLED_091_AZ)
                        #define DISP2_MAXCOLS  OLED_091_MAXCOLS
                        #define DISP2_MAXROWS  OLED_091_MAXROWS
                        #define OLED2_GEO      GEO_128_32
                        #define OLED2_DRV      OLED_DRV_1306
                      #endif
                    #if !(OLED2 ^ MC_UO_OLED_096_AZ)
                        #define DISP2_MAXCOLS  OLED_096_MAXCOLS
                        #define DISP2_MAXROWS  OLED_096_MAXROWS
                        #define OLED2_GEO      GEO_128_64
                        #define OLED2_DRV      OLED_DRV_1306
                      #endif
                    #if !(OLED2 ^ MC_UO_OLED_130_AZ)
                        #define DISP2_MAXCOLS  OLED_130_MAXCOLS
                        #define DISP2_MAXROWS  OLED_130_MAXROWS
                        #define OLED2_GEO      GEO_128_64
                        #define OLED2_DRV      OLED_DRV_1106
                      #endif
                  #endif

              #endif
            #if ( USE_TOUCHSCREEN > 0 )
              #endif // USE_TOUCHSCREEN

            #if ( USE_TFT > 0 )
                #if !(DISP_TFT ^ MC_UO_TOUCHXPT2046_AZ)
                    #define DISP_ORIENT    0      // 0:USB oben, 1:USB links, 2:USB unten, 3:USB rechts
                    #define DATE_DISP_COL  0
                    #define DATE_DISP_LINE 9      // line on display for date / time
                    // text display area
                    #define DISP_X         0
                    #define DISP_Y         0
                    #define DISP_W         240
                    #define DISP_H         300
                    #define DISP_BCOL      0x000F // TFT_NAVY
                    #define DISP_ANZ_SP    20
                    #define DISP_ANZ_ZE    12
                    #define DISP_Hoe_ZE    25
                    #define DISP_TX_FCOL   0xB7E0 // TFT_GREENYELLOW
                    #define DISP_TX_BCOL   DISP_BCOL
                    // status line for messages
                    #define STATUS_XCENT   120 // Centred on this
                    #define STATUS_YCENT   315
                    #define STATUS_XLI     0
                    #define STATUS_XRE     239
                    #define STATUS_YOB     DISP_H // 300
                    #define STATUS_YUN     319
                    #define STATUS_FCOL    0xF81F // TFT_MAGENTA
                    #define STATUS_BCOL    0x0000 // TFT_BLACK
                  #endif
                //
                #if !(DISP_TFT ^ MC_UO_TFT1602_GPIO_RO)
                    #define DATE_DISP_COL   0
                    #define DATE_DISP_LINE  0    // line on display for date / time
                    #define LCD_ROWS        2
                    #define LCD_CULS        2
                  #endif
              #endif
          #endif // DISP

      // --- acoustic output
        #if (USE_BUZZER > OFF)
            #if !(BUZZER1 ^ AOUT_PAS_BUZZ_3V5V)
                #define PLAY_MUSIC
                #define MUSIC_BASE_OCTA 5        // base oktave for musik
                //#define PLAY_START_MUSIC
                #define PLAY_START_DINGDONG
              #endif
          #endif // USE_BUZZER

    // --- user input
      #if defined(KEYS)
        // --- keypad
          #if !(KEYS ^ MC_UI_Keypad_ANA0_RO)
              #define USE_KEYPADSHIELD
              #define KEYS_ADC        34   // ADC Pin GPIO02
              #define ADC_STD_RES     12   // default resolution 12 Bit
              #define ADC_RES         12   // active resolution
              #define KP_NUM_KEYS     5
              #define KP_KEY_VAL_1    200  // max ADC value of button 0
              #define KP_KEY_VAL_2    750
              #define KP_KEY_VAL_3    1470
              #define KP_KEY_VAL_4    2330
              #define KP_KEY_VAL_5    3200
            #endif // keypad

        // --- touchpad
          #if !(KEYS ^ MC_UI_TOUCHXPT2046_AZ)
              // Keypad start position, key sizes and spacing
              #define KEY_X          40 // Centre of key
              #define KEY_Y          287
              #define KEY_W          62 // Width and height
              #define KEY_H          26
              #define KEY_SPACING_X  18 // X and Y gap
              #define KEY_SPACING_Y  20
              #define KEY_TEXTSIZE   1   // Font size multiplier
              #define KEY_NUM_LEN    3 // Anzahl der Tasten
            #endif // touchpad
        #endif

    // --- memories
      #if (USE_FRAM_I2C > OFF)
          #define SIZE_FRAM     0x8000
        #endif
    // --- sensors
      #if (USE_DS18B20_1W > OFF)
          #define DS_T_PRECISION   9
          #define DS18B20_ANZ      1
          #ifndef USE_MEASURE_CYCLE
              //#define USE_MEASURE_CYCLE
            #endif
        #endif

      #if (USE_BME280_I2C > OFF)
        #endif

      #if (USE_MQ135_GAS_ANA > OFF)
          #ifndef USE_MEASURE_CYCLE
              #define USE_MEASURE_CYCLE
            #endif
          #define MQ135_FILT      15       // floating  measure filtering
          //#define MQ135_ThresFilt 25       // threshold measure filtering
          #define MQ135_EM_WIN    100      // window for traffic light
          //#define MQ135_EM_MID    2350    // green < (MID-(WIN/2) < yellow < (MID+(WIN/2) < red
        #endif

      #if (USE_TYPE_K > OFF)
          #ifndef USE_MEASURE_CYCLE
              #define USE_MEASURE_CYCLE
            #endif
          #define TYPEK_FILT      11       // floating  measure filtering
          #define TYPEK_DROP_PEEK 2        // drop biggest / lowest
          #define TYPEK1_OFFSET   0.       // offset unit °C
          #define TYPEK1_GAIN     1.       // result = (measure * gain) + offset
          #define TYPEK2_OFFSET   0.       // offset unit °C
          #define TYPEK2_GAIN     1.       // result = (measure * gain) + offset
        #endif

      #ifdef USE_MEASURE_CYCLE
          #define MEASURE_CYCLE_MS  500u
        #endif

  // ******************************************
#endif // _PRJ_CONFIG_H_