#ifndef _PRJ_CONFIG_H_
  #define _PRJ_CONFIG_H_

  #include <Arduino.h>
  #include <md_defines.h>

  #define PROJ_TITLE "   ESP32 TEST"

  // --- system defines
    #define SER_BAUDRATE  115200ul
    #define GEO_128_64    0
    #define GEO_128_32    1
    #define GEO_64_48     2
    #define GEO_64_32     3
    #define GEO_RAWMODE   4

  //
  // ******************************************
  // --- debugging selection
    //#define DEBUG_MODE      CFG_DEBUG_STARTUP
    #define DEBUG_MODE      CFG_DEBUG_ACTIONS
    //#define DEBUG_MODE      CFG_DEBUG_DETAILS

  //
  // ******************************************
  // --- board selection
    //#define BOARD   MC_ESP32_D1_R32
    //#define BOARD   MC_AV_NANO_V3
    //#define BOARD   MC_AV_UNO_V3
    //#define BOARD    MC_ESP32_Node     // AZ_ESP32_DevKit_C_V4
    #define BOARD    MC_ESP32_D1_MINI
  //
  // ******************************************
  // --- board management
    #if !(BOARD ^ MC_ESP32_Node)
      // --- system
        //#define USE_TASKING
      //
      // --- network
        #define USE_WIFI
        #define USE_WEBSERVER
      //
      // --- user output
        // --- display
            #define USE_DISP
            #if defined(USE_DISP)
                #define USE_OLED_I2C
                #ifdef USE_OLED_I2C
                    #define ANZ_OLED   1 // 2 are possible
                        // select OLED device  GEO_RAWMODE?
                        // OLEDx_  MC_UO_OLED_091_AZ   GEO_128_32
                        // OLEDx_  MC_UO_OLED_096_AZ   GEO_128_64
                        // OLEDx_  MC_UO_OLED_130_AZ   GEO_128_64
                    #define OLED1
                    #define OLED1_MC_UO_OLED_130_AZ
                    #define OLED1_GEO    GEO_128_64
                    #if (ANZ_OLED > 1)
                        #define OLED2
                        #define OLED2_MC_UO_OLED_130_AZ
                        #define OLED2_GEO    GEO_128_64
                      #endif
                  #endif //USE_OLED_I2C

                //#define USE_TFT
                #ifdef USE_TFT
                    //#define DISP_TFT  MC_UO_TFT1602_GPIO_RO
                    //#define DISP_TFT  MC_UO_TOUCHXPT2046_AZ
                    //#define DISP_TFT  MC_UO_TFT1602_I2C_XA
                  #endif
              #endif
        //
        // --- acustic output
          //#define AOUT AOUT_PAS_BUZZ_3V5V
      //
      // --- user input
        //#define USE_TOUCHSCREEN
        #ifdef  USE_TOUCHSCREEN
            #define USE_TOUCHXPT2046_AZ_3V3
            #define KEYS KEYS_TOUCHXPT2046_AZ_3V3
          #endif // USE_TOUCHSCREEN

      //
      // --- sensors
        #define USE_DS18B20_1W
        #if defined(USE_DS18B20_1W)
            #define ANZ_DS18B20     1
          #endif
        #define USE_BME280_I2C
        #if defined(USE_BME280_I2C)
            #define ANZ_BME280     1
          #endif
      //
      // --- memories
        #define USE_FRAM_I2C
        #if defined( USE_FRAM_I2C )
            #define ANZ_FRAM        1
          #endif
      //
      // --- pins, connections
        #define PIN_BOARD_LED         2

        // --- user output
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

            //#define PIN_BUZZ            21
        // --- sensors
          #ifdef USE_DS18B20_1W
              #define DS_ONEWIRE_PIN  27
            #endif
      #endif

      // --- I2C
        // --- board connection
          #define ANZ_I2C           1
          #define USE_I2C1          1
          #define PIN_I2C1_SDA      21
          #define PIN_I2C1_SCL      22
          #if ( ANZ_I2C > 1 )
              #define USE_I2C2      2
              #define PIN_I2C2_SDA  25
              #define PIN_I2C2_SCL  26
            #endif
    //
    #if !(BOARD ^ MC_ESP32_D1_MINI)
      // --- system
        //#define USE_TASKING
      //
      // --- network
        #define USE_WIFI
        #define USE_WEBSERVER
      //
      // --- user output
        // --- display
            #define USE_DISP
            #if defined(USE_DISP)
                #define USE_OLED_I2C
                #ifdef USE_OLED_I2C
                    #define ANZ_OLED   1 // 2 are possible
                        // select OLED device  GEO_RAWMODE?
                        // OLEDx_  MC_UO_OLED_091_AZ   GEO_128_32
                        // OLEDx_  MC_UO_OLED_096_AZ   GEO_128_64
                        // OLEDx_  MC_UO_OLED_130_AZ   GEO_128_64
                        // OLEDx_  MC_UO_OLED_066_AZ   GEO_64_48
                    #define OLED1
                    #define OLED1_MC_UO_OLED_130_AZ
                    #define OLED1_GEO    GEO_128_64
                    #if (ANZ_OLED > 1)
                        #define OLED2
                        #define OLED2_MC_UO_OLED_130_AZ
                        #define OLED2_GEO    GEO_128_64
                      #endif
                  #endif //USE_OLED_I2C

                //#define USE_TFT
                #ifdef USE_TFT
                    //#define DISP_TFT  MC_UO_TFT1602_GPIO_RO
                    //#define DISP_TFT  MC_UO_TOUCHXPT2046_AZ
                    //#define DISP_TFT  MC_UO_TFT1602_I2C_XA
                  #endif
              #endif
        //
        // --- acustic output
          //#define AOUT AOUT_PAS_BUZZ_3V5V
      //
      // --- user input
        //#define USE_TOUCHSCREEN
        #ifdef  USE_TOUCHSCREEN
            #define USE_TOUCHXPT2046_AZ_3V3
            #define KEYS KEYS_TOUCHXPT2046_AZ_3V3
          #endif // USE_TOUCHSCREEN

      //
      // --- sensors
        #define USE_DS18B20_1W
        #if defined(USE_DS18B20_1W)
            #define ANZ_DS18B20     1
          #endif
        #define USE_BME280_I2C
        #if defined(USE_BME280_I2C)
            #define ANZ_BME280     1
          #endif
      //
      // --- memories
        //#define USE_FRAM_I2C
        #if defined( USE_FRAM_I2C )
            #define ANZ_FRAM        1
          #endif
      //
      // --- pins, connections
        #define PIN_BOARD_LED         2

        // --- user output
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

            //#define PIN_BUZZ            21
        // --- sensors
          #ifdef USE_DS18B20_1W
              #define DS_ONEWIRE_PIN  27
            #endif
      #endif

      // --- I2C
        // --- board connection
          #define ANZ_I2C           1
          #define USE_I2C1          1
          #define PIN_I2C1_SDA      21
          #define PIN_I2C1_SCL      22
          #if ( ANZ_I2C > 1 )
              #define USE_I2C2      2
              #define PIN_I2C2_SDA  25
              #define PIN_I2C2_SCL  26
            #endif
    //
    #if !(BOARD ^ MC_ESP32_D1_R32)
          #define USE_TASKING
        // --- network
          #define USE_WIFI
          #define USE_WEBSERVER
        // --- user interface
          #define USE_DISP
          #ifdef USE_DISP
              #define USE_OLED_I2C
              #ifdef USE_OLED_I2C
                  // select I2C device
                  #define OLED1 DISP_OLED_130_AZ_3V3
                    #define OLED1_I2C_ADDR  0x3C
                  //#define OLED2 DISP_OLED_091_AZ_3V3
                    #define OLED2_I2C_ADDR  0x3C
                  //#define OLED?  DISP_OLED_096_AZ_3V3
                #endif //USE_OLED_I2C

              //#define USE_TFT
              #ifdef USE_TFT
                      //#define USE_TFT1602_I2C_XA_3V3
                      //#define USE_TOUCHXPT2046_AZ_3V3
                #endif
            #endif
          //#define KEYS KEYS_TOUCHXPT2046_AZ_3V3
          //#define USE_KEYPADSHIELD
          //#define OUT1 AOUT_PAS_BUZZ_3V5V
          #ifdef USE_KEYPADSHIELD
              #define USE_TFT1602_GPIO_RO_3V3 // used by KEYPADSHIELD
              #define KEYS KEYS_Keypad_ANA0_RO_3V3  // used by KEYPADSHIELD
            #endif // USE_KEYPADSHIELD
        // --- connections
          #define I2C1_SDA   26 //21
          #define I2C1_SCL   25 //22
          #define I2C2_SDA   17
          #define I2C2_SCL   16
      #endif

    //
    #if !(BOARD ^ MC_AV_NANO_V3)
        #define USE_KEYPADSHIELD
        #ifdef USE_KEYPADSHIELD
            #define USE_TFT1602_GPIO_RO_V5  // used by KEYPADSHIELD
            #define KEYS_Keypad_ANA0_RO_V5        // used by KEYPADSHIELD
          #endif // USE_KEYPADSHIELD
      #endif

    //
    #if !(BOARD ^ MC_AV_UNO_V3)
        #define USE_KEYPADSHIELD
        #ifdef USE_KEYPADSHIELD
            #define USE_TFT1602_GPIO_RO_V5  // used by KEYPADSHIELD
            #define KEYS_Keypad_ANA0_RO_V5        // used by KEYPADSHIELD
          #endif // USE_KEYPADSHIELD
      #endif // BOARD
  //
  // ******************************************
  // --- specification
    // --- system
      // --- error status bits
        #define ERRBIT_TOUCH     0x00000001     // touchscreen
        #define ERRBIT_SERVER    0x00000002     // webserver
        #define ERRBIT_WIFI      0x00000004     // WIFI connection
        #define ERRBIT_NTPTIME   0x00000008     // NTP timeserver connection
      //
      // --- generic
        #define SCAN_I2C   128
        #define CHECK_I2C_DEVICES
        #define UTC_SEASONTIME UTC_WINTERTIME
        //#define UTC_SEASONTIME UTC_SUMMERTIME
        #ifdef USE_TASKING
            #define USE_SONGTASK
            #define TASK_SLICE_T  5000ul   // task beat [us]
          #endif // USE_TASKING
    //
    // --- I2C interface
      // --- address configuration
        #if defined( USE_OLED_I2C )
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
            #if (( ANZ_I2C > 1 ) && ( ANZ_OLED > 1 ))
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
        //
        #if defined( USE_FRAM_I2C )
            #define I2C_ADDR_FRAM1          I2C_FRAM
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
            #if (( ANZ_I2C > 1 ) && ( ANZ_FRAM > 1 ))
              #define I2C_ADDR_FRAM2        I2C_FRAM
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
        //
        #if defined( USE_BME280_I2C )
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
            #if (( ANZ_I2C > 1 ) && ( ANZ_BME280 > 1 ))
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

    //
    // ---------------------------------------------
    // --- user interface
      // --- display output
        #define DISP_CYCLE       1000ul   // Intervallzeit [us]
        // output status line
        #define STAT_TIMEDEF     5000u    // default time to clear status
    //
    // --- network
      // --- WIFI
        #if defined(USE_WIFI)
            #define USE_NTP_SERVER
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

            #define USE_LOCAL_IP    TRUE
            #define WIFI_ANZ_LOCIP  WIFI_ANZ_LOGIN
            #define WIFI_FIXIP0     0x1400000Aul // 10.0.0.20   lowest first
            #define WIFI_GATEWAY0   0x8B00000Aul // 10.0.0.139
            #define WIFI_FIXIP1     0x1400000Aul // 10.0.0.20
            #define WIFI_GATEWAY1   0x8B00000Aul // 10.0.0.139
            #define WIFI_FIXIP2     0x1400000Aul // 10.0.0.20
            #define WIFI_GATEWAY2   0x8a00000Aul // 10.0.0.138
            #define WIFI_SUBNET     0x00FFFFFFul // 255.255.255.0
          #endif

      //
      // --- webserver
        #if defined(USE_WEBSERVER)
            #define WEBSERVER_CYCLE 1000ul  // Intervallzeit [us]
          #endif
    //
    // --- user output
      // --- display
        #ifdef USE_DISP
            #define USE_STATUS
            #if defined( USE_OLED_I2C )
                #if defined( OLED1 )
                    #define USE_STATUS1
                    #if defined( OLED1_MC_UO_OLED_091_AZ )
                       #define DISP1_MAXCOLS  OLED_091_MAXCOLS
                       #define DISP1_MAXROWS  OLED_091_MAXROWS
                      #endif
                    #if defined( OLED1_MC_UO_OLED_096_AZ )
                        #define DISP1_MAXCOLS  OLED_096_MAXCOLS
                        #define DISP1_MAXROWS  OLED_096_MAXROWS
                      #endif
                    #if defined( OLED1_MC_UO_OLED_130_AZ )
                        #define DISP1_MAXCOLS  OLED_130_MAXCOLS
                        #define DISP1_MAXROWS  OLED_130_MAXROWS
                      #endif
                  #endif
                #if defined( OLED2 )
                    #define USE_STATUS2
                    #if defined( OLED2_MC_UO_OLED_091_AZ )
                        #define DISP2_MAXCOLS  OLED_091_MAXCOLS
                        #define DISP2_MAXROWS  OLED_091_MAXROWS
                      #endif
                    #if defined( OLED2_MC_UO_OLED_096_AZ )
                        #define DISP2_MAXCOLS  OLED_096_MAXCOLS
                        #define DISP2_MAXROWS  OLED_096_MAXROWS
                      #endif
                    #if defined( OLED2_MC_UO_OLED_130_AZ )
                        #define DISP2_MAXCOLS  OLED_130_MAXCOLS
                        #define DISP2_MAXROWS  OLED_130_MAXROWS
                      #endif
                  #endif

              #endif
            #if defined( USE_TOUCHSCREEN )
              #endif // USE_TOUCHSCREEN

            #if defined( USE_TFT )
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
        //
      //
      // --- acoustic output
        #ifdef AOUT
            #if !(AOUT ^ AOUT_PAS_BUZZ_3V5V)
                #define PLAY_MUSIC
                #define PLAY_START_MUSIC
                #define PWM_BUZZ      1
                #define MUSIC_BASE_OCTA 5        // base oktave for musik
              #endif // AOUT
          #endif

    //
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
        //
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

    //
    // --- memories
      #ifdef USE_FRAM_I2C
          #define SIZE_FRAM     0x8000
        #endif

    //
    // --- sensors
      #ifdef USE_DS18B20_1W
          #define DS_T_PRECISION  9
          #define DS18B20_ANZ     1
        #endif

      #ifdef USE_BME280_I2C

        #endif
  //
  // ******************************************
#endif // _PRJ_CONFIG_H_