#include <main.h>
#include <prj_config.h>

// ----------------------------------------------------------------
// --- declarations
// ----------------------------------------------------------------
  // ------ system -----------------------
    static bool firstrun = true;
    static uint16_t     md_error  = 0    // Error-Status bitkodiert -> 0: alles ok
                             #if (USE_WIFI > OFF)
                               + ERRBIT_WIFI
                               #if (USE_NTP_SERVER > OFF)
                                 + ERRBIT_NTPTIME
                               #endif
                             #endif
                             #if (USE_WEBSERVER > OFF)
                               + ERRBIT_SERVER
                             #endif
                             #if (USE_TOUCHSCREEN > OFF)
                               + ERRBIT_TOUCH
                             #endif
                             ;
	    // cycletime measurement
    static uint64_t anzUsCycles = 0ul;
    static uint64_t usLast      = 0ul;
    static uint64_t usTmp       = 0ul;
    static uint64_t usPerCycle  = 0ul;
    static uint32_t freeHeap    = 10000000;
    static char     cmsg[MSG_MAXLEN+1] = "";
    static String   tmpStr;
    static uint16_t tmpval16;
    static uint32_t tmpval32;
      //static uint64_t anzMsCycles = 0;
	    //static uint64_t msLast = 0;
  	  //static uint64_t msPerCycle = 0;

	    //static uint32_t anzMsCycles = 0;
	    //static uint64_t msLast      = 0;
    #if ( DEV_I2C1 > OFF )
        TwoWire i2c1 = TwoWire(0);
      #endif
    #if ( DEV_I2C2 > OFF )
        TwoWire i2c2 = TwoWire(1);
      #endif
    #if ( DEV_VSPI > OFF )
        //SPIClass pVSPI(VSPI);
      #endif
    #if ( DEV_HSPI > OFF )
        //SPIClass pHSPI(HSPI);
      #endif

    #if ( USE_LED_BLINK_OUT > 0 )
        msTimer ledT = msTimer(BLINKTIME_MS);
        uint8_t SYS_LED_ON = ON;
      #endif

    #if ( USE_DISP > 0 )
        uint32_t      ze     = 1;      // aktuelle Schreibzeile
        char          outBuf[OLED1_MAXCOLS + 1] = "";
        String        outStr;
      #endif
    #ifdef USE_STATUS
        msTimer     statT  = msTimer(STAT_DELTIME_MS);
        msTimer     statN  = msTimer(STAT_NEWTIME_MS);
        char        statOut[60 + 1] = "";
        bool        statOn = false;
        bool        statDate = false;
          //char        timeOut[STAT_LINELEN + 1] = "";
      #endif
    #if (USE_ESPHALL > OFF)
        int32_t valHall = 0;
      #endif
// ------ system cycles ---------------
    #ifdef USE_INPUT_CYCLE
        msTimer inputT           = msTimer(INPUT_CYCLE_MS);
        uint8_t inpIdx   = 0;
      #endif
    #ifdef USE_OUTPUT_CYCLE
        msTimer outpT            = msTimer(OUTPUT_CYCLE_MS);
        uint8_t outpIdx  = 0;
      #endif
    #if (USE_DISP > OFF)
        msTimer dispT            = msTimer(DISP_CYCLE_MS);
        uint8_t dispIdx  = 0;
      #endif
// ------ user input ---------------
    #if (USE_TOUCHSCREEN > OFF)
        md_touch  touch  =  md_touch(TOUCH_CS, TFT_CS, TFT_DC, TFT_RST, TFT_LED, LED_ON);
        md_touch* ptouch =  &touch;
      #endif

    #if (USE_KEYPADSHIELD > OFF)
        md_kpad kpad(KEYS_ADC);
        uint8_t key;
      #endif // USE_KEYPADSHIELD

    #if (USE_CTRL_POTI > OFF)
        uint16_t inpValADC[USE_CTRL_POTI];
      #endif

    #if (USE_DIG_INP > OFF)
        uint8_t  valInpDig[USE_DIG_INP];
        uint8_t  pinInpDig[USE_DIG_INP];
        uint8_t  polInpDig[USE_DIG_INP];
        uint8_t  modInpDig[USE_DIG_INP];
      #endif

    #if (USE_CNT_INP > OFF)
        const pcnt_config_t config_cnt[USE_CNT_INP] =
          {
            {
              PCNT1_INP_SIG_IO,      // Pulse input GPIO number, if you want to use GPIO16, enter pulse_gpio_num = 16, a negative value will be ignored
              PCNT_PIN_NOT_USED,     // Control signal input GPIO number, a negative value will be ignored
              PCNT_MODE_KEEP,        // PCNT low control mode
              PCNT_MODE_KEEP,        // PCNT high control mode
              PCNT_COUNT_DIS,        // PCNT positive edge count mode
              PCNT_COUNT_INC,        // PCNT negative edge count mode
              PCNT_H_LIM_VAL,        // Maximum counter value
              PCNT_L_LIM_VAL,        // Minimum counter value
              (pcnt_unit_t)    PCNT1_UNIT,  // PCNT unit number
              (pcnt_channel_t) PCNT1_CHAN  // the PCNT channel
            },
            #if (USE_CNT_INP > 1)
                {
                  PCNT2_INP_SIG_IO,   //< Pulse input GPIO number, if you want to use GPIO16, enter pulse_gpio_num = 16, a negative value will be ignored
                  PCNT_PIN_NOT_USED,  //< Control signal input GPIO number, a negative value will be ignored
                  PCNT_MODE_KEEP,     //< PCNT low control mode
                  PCNT_MODE_KEEP,     // PCNT high control mode
                  PCNT_COUNT_DIS,     // PCNT positive edge count mode
                  PCNT_COUNT_INC,     // PCNT negative edge count mode
                  PCNT_H_LIM_VAL,     // Maximum counter value
                  PCNT_L_LIM_VAL,     // Minimum counter value
                  (pcnt_unit_t)    PCNT2_UNIT, // PCNT unit number
                  (pcnt_channel_t) PCNT2_CHAN  // the PCNT channel
                },
              #endif
            #if (USE_CNT_INP > 2)
                {
                  PCNT2_SIO,         // Pulse input GPIO number, if you want to use GPIO16, enter pulse_gpio_num = 16, a negative value will be ignored
                  PCNT2_CIO,         // Control signal input GPIO number, a negative value will be ignored
                  PCNT2_LCTRL_MODE,  // PCNT low control mode
                  PCNT2_HCTRL_MODE,  // PCNT high control mode
                  PCNT2_POS_MODE,    // PCNT positive edge count mode
                  PCNT2_NEG_MODE,    // PCNT negative edge count mode
                  PCNT2_H_NUM,       // Maximum counter value
                  PCNT2_L_NUM,       // Minimum counter value
                  (pcnt_unit_t)    PCNT2_UNIT,  // PCNT unit number
                  (pcnt_channel_t) PCNT2_CHAN   // the PCNT channel
                },
              #endif
            #if (USE_CNT_INP > 3)
                {
                  PCNT3_SIO,         // Pulse input GPIO number, if you want to use GPIO16, enter pulse_gpio_num = 16, a negative value will be ignored
                  PCNT3_CIO,         // Control signal input GPIO number, a negative value will be ignored
                  PCNT3_LCTRL_MODE,  // PCNT low control mode
                  PCNT3_HCTRL_MODE,  // PCNT high control mode
                  PCNT3_POS_MODE,    // PCNT positive edge count mode
                  PCNT3_NEG_MODE,    // PCNT negative edge count mode
                  PCNT3_H_NUM,       // Maximum counter value
                  PCNT3_L_NUM,       // Minimum counter value
                  (pcnt_unit_t)    PCNT3_UNIT,  // PCNT unit number
                  (pcnt_channel_t) PCNT3_CHAN   // the PCNT channel
                },
              #endif
          };
        //typedef intr_handle_t pcnt_isr_handle_t;

        /* A sample structure to pass events from the PCNT
         * interrupt handler to the main program.
         */
        typedef struct
          {
            int16_t  pulsCnt;   // the PCNT unit that originated an interrupt
            uint16_t tres;
            uint32_t freq;   // information on the event type that caused the interrupt
            uint64_t usCnt;
          } pcnt_evt_t;

        static xQueueHandle pcnt_evt_queue[USE_CNT_INP];   // A queue to handle pulse counter events
        static pcnt_evt_t cntErg[USE_CNT_INP];
        static int8_t     cntFilt[USE_CNT_INP];
        static float      cntFakt[USE_CNT_INP];
        static uint16_t   cntThresh[USE_CNT_INP];

          //static pcnt_evt_t tmpErg;
          //static uint64_t oldClk[USE_CNT_INP] = {NULL};
        static uint64_t oldUs[USE_CNT_INP];

        /* Decode what PCNT's unit originated an interrupt
         * and pass this information together with the event type
         * the main program using a queue.
         */

        #ifndef USE_INT_EVTHDL
            #define USE_INT_EVTHDL
          #endif
        portBASE_TYPE pcnt_res;
        //static const char *PCNTTAG = "pcnt_int ";

        static void IRAM_ATTR pcnt0_intr_hdl(void *arg)
          {
            BaseType_t port_status;
            pcnt_evt_t event;
            port_status = pcnt_get_counter_value((pcnt_unit_t) PCNT0_UNIT, &(event.pulsCnt));
            event.usCnt = micros() - oldUs[PCNT0_UNIT];
            //event.count = isrCnt[PCNT0_UNIT];
            pcnt_res = xQueueSendToBackFromISR(pcnt_evt_queue[PCNT0_UNIT], &event, &port_status);
            oldUs[PCNT0_UNIT] = micros();
            port_status = pcnt_counter_clear((pcnt_unit_t) PCNT0_UNIT);
          }

        #if (USE_CNT_INP > 1)
            static void IRAM_ATTR pcnt1_intr_hdl(void *arg)
              {
                BaseType_t port_status;
                pcnt_evt_t event;
                port_status = pcnt_get_counter_value((pcnt_unit_t) PCNT1_UNIT, &(event.pulsCnt));
                event.usCnt = micros() - oldUs[PCNT1_UNIT];
                //event.count = isrCnt[PCNT1_UNIT];
                pcnt_res = xQueueSendToBackFromISR(pcnt_evt_queue[PCNT1_UNIT], &event, &port_status);
                oldUs[PCNT1_UNIT] = micros();
                port_status = pcnt_counter_clear((pcnt_unit_t) PCNT1_UNIT);
              }
          #endif
        #if (USE_CNT_INP > 2)
            static void IRAM_ATTR pcnt2_intr_hdl(void *arg)
              {
                BaseType_t port_status = pdFALSE;
                pcnt_evt_t event;
                port_status = pcnt_get_counter_value((pcnt_unit_t) PCNT2_UNIT, &(event.pulsCnt));
                event.usCnt  = micros();
                //event.intCnt = intCnt[PCNT2_UNIT];
                xQueueSendFromISR(pcnt_evt_queue[PCNT2_UNIT], &event, &port_status);
                //xQueueOverwriteFromISR(pcnt_evt_queue[PCNT2_UNIT], &event, &port_status);
                port_status = pcnt_counter_clear((pcnt_unit_t) PCNT2_UNIT);
                //intCnt[0] = 0;
              }
          #endif
        #if (USE_CNT_INP > 3)
            static void IRAM_ATTR pcnt3_intr_hdl(void *arg)
              {
                BaseType_t port_status = pdFALSE;
                pcnt_evt_t event;
                port_status = pcnt_get_counter_value((pcnt_unit_t) PCNT3_UNIT, &(event.pulsCnt));
                event.usCnt  = micros();
                //event.intCnt = intCnt[PCNT3_UNIT];
                xQueueSendFromISR(pcnt_evt_queue[PCNT3_UNIT], &event, &port_status);
                //xQueueOverwriteFromISR(pcnt_evt_queue[PCNT3_UNIT], &event, &port_status);
                port_status = pcnt_counter_clear((pcnt_unit_t) PCNT3_UNIT);
                //intCnt[0] = 0;
              }
          #endif
      #endif
    #if (USE_PWM_INP > OFF)
        typedef struct
          {
            uint32_t lowVal;
            uint32_t highVal;
          } pwm_val_t;
        pwm_val_t pwmInVal[USE_PWM_INP];
      #endif
  // ------ user output ---------------
    #if (USE_RGBLED_PWM > OFF)
        msTimer      rgbledT   = msTimer(PWM_LEDS_CYCLE_MS);
        outRGBVal_t  outValRGB[USE_RGBLED_PWM];
        md_LEDPix24* RGBLED[2] = { new md_LEDPix24((uint32_t) COL24_RGBLED_1), new md_LEDPix24((uint32_t) COL24_RGBLED_1) };
        uint8_t      LEDout    = FALSE;
        #if (TEST_RGBLED_PWM > OFF)
            //uint8_t  colRGBLED = 0;
            //uint16_t incRGBLED = 10;
            //uint32_t RGBLED_gr = 64;
            //uint32_t RGBLED_bl = 128;
            //uint32_t RGBLED_rt = 192;
          #endif
      #endif
    #if (USE_WS2812_MATRIX_OUT > OFF)
        #if (ANZ_TILES_M1 > OFF)
            md_ws2812_matrix matrix_1 = md_ws2812_matrix
              ( COLPIX_2812_T1, ROWPIX_2812_T1,
                COLTIL_2812_M1, ROWTIL_2812_M1, PIN_WS2812_M1,
                ROW1_2812_T1  + COL1_2812_T1 +
                DIR_2812_T1   + ORI_2812_T1,
                (neoPixelType) COLORD_2812_M1 + NEO_KHZ800 );
            //md_LEDPix24* matrix2812[2] = { new md_LEDPix24(), new md_LEDPix24() };
        #else
            md_ws2812_matrix matrix_1 = md_ws2812_matrix
              ( COLPIX_2812_M1, ROWPIX_2812_M1,
                0, 0, PIN_WS2812_M1,
                ROW1_2812_M1  + COL1_2812_M1 +
                DIR_2812_M1   + ORI_2812_M1,
                (neoPixelType) COLORD_2812_M1 + NEO_KHZ800 );
          #endif
        msTimer ws2812T1   = msTimer(UPD_2812_M1_MS);
        #if (USE_WS2812_MATRIX_OUT > 1)
            md_ws2812_matrix matrix_2 = md_ws2812_matrix
              ( COLPIX_2812_M2, ROWPIX_2812_M2,
                COLTIL_2812_M2, ROWTIL_2812_M2, PIN_WS2812_M2,
                #if (ANZ_TILES_M2 > OFF)
                    NEO_TILE_TOP       + NEO_TILE_LEFT +
                    NEO_TILE_ROWS      + NEO_TILE_PROGRESSIVE +
                  #endif
                NEO_MATRIX_TOP     + NEO_MATRIX_LEFT +
                NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
                (neoPixelType) COLORD_2812_M1 + NEO_KHZ800 );
            msTimer ws2812MT   = msTimer(UPD_2812_M1_MS);
          #endif
        // const char text2812[] = " # YES We Care !!  Happy 2022 ";
        const char text2812[] = " Im Herzen die Sonne - willkommen im Weltladen ";
        static scroll2812_t outM2812[2] = { scroll2812_t(), scroll2812_t() } ;
        static int16_t posM2812 = (int16_t) (COLPIX_2812_M1 + OFFBEG_2812_M1);
        unsigned long  ws2812_Malt = 0;
        uint32_t       ws2812_Mcnt = 0;
        uint32_t       ws2812_Mv   = 0;
        bool           ws2812_Mpwr = false;
      #endif

    #if (USE_WS2812_LINE_OUT > OFF)
        msTimer         ws2812LT    = msTimer(UPD_2812_L1_MS);
        Adafruit_NeoPixel strip = Adafruit_NeoPixel(COLPIX_2812_L1, PIN_WS2812_L1, (neoPixelType) COLORD_2812_L1 + NEO_KHZ800);
          //static uint8_t ws2812_Lbright = (uint8_t) BRIGHT_2812_L1;
          //static uint8_t ws2812_Lrt  = 100;
          //static uint8_t ws2812_Lbl  = 100;
          //static uint8_t ws2812_Lgr  = 100;
        unsigned long  ws2812_Lalt = 0;
        uint32_t       ws2812_Lcnt = 0;
        uint32_t       ws2812_Lv   = 0;
        bool           ws2812_pwr  = false;
        uint16_t       idx2812L1   = 0;
        md_LEDPix24*   line2812[2] = {NULL, NULL};
      #endif

    #if (USE_BUZZER_PWM > OFF)
        md_buzzer     buzz       = md_buzzer();
        #ifdef PLAY_MUSIC
            tone_t test = {0,0,0};
          #endif
      #endif // USE_BUZZER_PWM

    #if (USE_FAN_PWM > OFF)
        msTimer      fanT   = msTimer(PWM_FAN_CYCLE_MS);
        #if (USE_POTICTRL_FAN > OFF)
          #endif
        uint32_t valFanPWM[USE_FAN_PWM];
        uint16_t fanIdx = 0;
      #endif

    #if (OLED1_I2C > OFF)
        #if (OLED1_I2C > OFF)
            #if !(OLED1_DRV ^ OLED_DRV_1106)
                md_oled_1106 oled1 = md_oled_1106((uint8_t) OLED1_I2C_ADDR, (uint8_t) OLED1_I2C_SDA,
                                        (uint8_t) OLED1_I2C_SCL, (OLEDDISPLAY_GEOMETRY) OLED1_GEO);
            #else
                md_oled_1306 oled1 = md_oled_1306((uint8_t) OLED1_I2C_ADDR, (uint8_t) OLED1_I2C_SDA,
                                        (uint8_t) OLED1_I2C_SCL, (OLEDDISPLAY_GEOMETRY) OLED1_GEO);
              #endif
          #endif
        #if (DISP_I2C21 > OFF)
            #if !(OLED2_DRV ^ OLED_DRV_1106)
                md_oled_1106 oled2 = md_oled_1106((uint8_t) OLED2_I2C_ADDR, (uint8_t) OLED2_I2C_SDA,
                                        (uint8_t) OLED2_I2C_SCL, (OLEDDISPLAY_GEOMETRY) OLED2_GEO);
              #else
                md_oled_1306 oled2 = md_oled_1306((uint8_t) OLED2_I2C_ADDR, (uint8_t) OLED2_I2C_SDA,
                                        (uint8_t) OLED2_I2C_SCL, (OLEDDISPLAY_GEOMETRY) OLED2_GEO);
              #endif
          #endif
        msTimer oledT   = msTimer(DISP_CYCLE_MS);
      #endif
    #if (defined(USE_TFT1602_GPIO_RO_3V3) || defined(USE_TFT1602_GPIO_RO_3V3))
        LiquidCrystal  lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
        void*          plcd = (void*) &lcd;
        md_lcd         mlcd(plcd);
      #endif
  // ------ network ----------------------
    #if (USE_WIFI > OFF)
        md_wifi wifi  = md_wifi();
        msTimer wifiT = msTimer(WIFI_CONN_CYCLE);
        #if (USE_LOCAL_IP > OFF)
          #endif // USE_LOCAL_IP
        #if (USE_NTP_SERVER > OFF)
            msTimer ntpT    = msTimer(NTPSERVER_CYCLE);
            time_t  ntpTime = 0;
            bool    ntpGet  = true;
          #endif // USE_WEBSERVER
      #endif
    #if (USE_WEBSERVER > OFF)
        #if (TEST_SOCKET_SERVER > OFF)
          /*
            const char index_html[] PROGMEM = R"rawliteral(
            <!DOCTYPE html>
            <html>
            <head>
              <meta name="viewport" content="width=device-width, initial-scale=1">  <title>ESP32 Websocket</title>
              <script src="https://ajax.googleapis.com/ajax/libs/jquery/3.5.1/jquery.min.js">
              <script src="https://cdn.jsdelivr.net/npm/spectrum-colorpicker2/dist/spectrum.min.js"></script>

              <link rel="stylesheet" type="text/css" href="https://cdn.jsdelivr.net/npm/spectrum-colorpicker2/dist/spectrum.min.css">
              <script language="javascript">

                window.alert(location.host);
                var gwUrl = "ws://" + location.host + "/ws";
                var webSocket = new WebSocket(gwUrl);
                webSocket.onopen = function(e) {
                    console.log("open");
                }
                webSocket.onclose = function(e) {
                    console.log("close");
                }

               webSocket.onmessage = function(e) {
                    console.log("message");
                }
                function handleColor() {
                  var val = document.getElementById('type-color-on-page').value;
                  webSocket.send(val.substring(1));
                }
              </script>

              <style>
                h2 {background: #3285DC;
                    color: #FFFFFF;
                    align:center;
                }

                .content {
                    border: 1px solid #164372;
                    padding: 5px;
                }

                .button {
                   background-color: #00b300;
                   border: none;
                   color: white;
                   padding: 8px 10px;
                   text-align: center;
                   text-decoration: none;
                   display: inline-block;
                   font-size: 14px;
              }
              </style>
            </head>
            <body>
              <h2>ESP32 Websocket</h2>
              <div class="content">
              <p>Pick a color</p>
              <div id="qunit"></div>

              <input type="color" id="type-color-on-page"  />
               <p>
                 <input type="button" class="button" value="Send to ESP32" id="btn" onclick="handleColor()" />
               </p>

              </div>
            </body>
            </html>
            )rawliteral";

            // Web server running on port 80
            AsyncWebServer serv(80);
            // Web socket
            AsyncWebSocket socket("/ws");
          */
        #else
            md_server*   pmdServ   = new md_server();
            static bool  newClient = false;
          #endif
        msTimer   servT = msTimer(WEBSERVER_CYCLE);
      #endif // USE_WEBSERVER
    #if (USE_MQTT > OFF)
        //AsyncMqttClient  mqttClient;
        TimerHandle_t    mqttReconnectTimer;
        char tmpMQTT[40];
        char tmpOut[40];
        void* mqttID = NULL;
      #endif
  // ------ sensors ----------------------
    #if (USE_BME280_I2C > OFF)
        Adafruit_BME280  bme1;
        #if (BME2801_I2C == I2C1)
            TwoWire* pbme1i2c = &i2c1;
          #else
            TwoWire* pbme1i2c = &i2c2;
          #endif
        md_val<int16_t>  bme1T;
        md_val<uint16_t> bme1P;
        md_val<uint16_t> bme1H;
        #if (USE_BME280_I2C > 1)
            #if (BME2802_I2C == I2C1)
                TwoWire* pbme2i2c = &i2c1;
              #else
                TwoWire* pbme2i2c = &i2c2;
              #endif
            Adafruit_BME280 bme2;
            md_val<int16_t> bme2T;
            md_val<uint16_t> bme2P;
            md_val<uint16_t> bme2H;
          #endif
      #endif
    #if (USE_DS18B20_1W_IO > OFF)
        OneWire dsOneWire(DS_ONEWIRE_PIN);
        DallasTemperature dsSensors(&dsOneWire);
        DeviceAddress     dsAddr[DS18B20_ANZ];
        float dsTemp[DS18B20_ANZ];
      #endif
    #if (USE_ADC1115_I2C > OFF)
        Adafruit_ADS1115 ads[USE_ADC1115_I2C];
      #endif
    #if (USE_MQ135_GAS_ANA > OFF)
        //filterValue tholdGas(MQ135_ThresFilt,1);
        md_val<double> gasVal;
        //md_val<uint16_t> gasThres;
      #endif

    #if (USE_MQ3_ALK_ANA > OFF)
        md_val<int16_t> alkVal[USE_MQ3_ALK_ANA];
        md_scale<int16_t> alkScal[USE_MQ3_ALK_ANA];
        uint16_t alk[USE_MQ3_ALK_ANA];
      #endif

    #if (USE_PHOTO_SENS_ANA > OFF)
        md_val<int16_t>   photoVal[USE_PHOTO_SENS_ANA];
        md_scale<int16_t> photoScal[USE_PHOTO_SENS_ANA];
        int16_t           bright[USE_PHOTO_SENS_ANA];
      #endif

    #if (USE_POTI_ANA > OFF)
        md_val<int16_t>   potiVal[USE_POTI_ANA];
        md_scale<int16_t> potiScal[USE_POTI_ANA];
        uint16_t          poti[USE_POTI_ANA];
      #endif

    #if (USE_VCC_ANA > OFF)
        md_val<int16_t>   vccVal[USE_VCC_ANA];
        md_scale<int16_t> vccScal[USE_VCC_ANA];
        uint16_t          vcc[USE_VCC_ANA];
      #endif

    #if (USE_ACS712_ANA > OFF)
        md_val<int16_t>   i712Val[USE_ACS712_ANA];
        md_scale<int16_t> i712Scal[USE_ACS712_ANA];
        //float i712[USE_ACS712_ANA];
        uint16_t          i712[USE_ACS712_ANA];
      #endif

    #if (USE_TYPE_K_SPI > 0)
        //SPIClass* tkSPI = new SPIClass();
        //md_31855_ktype TypeK1(TYPEK1_CS_PIN, tkSPI);
        md_31855_ktype TypeK1(TYPEK1_CS_PIN, TYPEK_CLK_PIN, TYPEK_DATA_PIN);
        filterValue valTK1(TYPEK_FILT, TYPEK_DROP_PEEK, TYPEK1_OFFSET, TYPEK1_GAIN);
        filterValue valTK1ref(TYPEK_FILT, TYPEK_DROP_PEEK, TYPEK1_OFFSET, TYPEK1_GAIN);
        int16_t     tk1Val;
        int16_t     tk1ValRef;
        #if ( USE_TYPE_K_SPI > 1)
            //md_31855_ktype TypeK2(TYPEK2_CS_PIN, tkSPI);
            md_31855_ktype TypeK2(TYPEK2_CS_PIN, TYPEK_CLK_PIN, TYPEK_DATA_PIN);
            filterValue valTK2(TYPEK_FILT, TYPEK_DROP_PEEK, TYPEK2_OFFSET, TYPEK2_GAIN);
            filterValue valTK2ref(TYPEK_FILT, TYPEK_DROP_PEEK, TYPEK2_OFFSET, TYPEK2_GAIN);
            int16_t     tk2Val;
            int16_t     tk2ValRef;
          #endif
      #endif

  // ------ memories
    #if (USE_FLASH_MEM > OFF)
        #include <SPIFFS.h>
      #endif

    #if (USE_FRAM_I2C > OFF)
        md_FRAM fram = md_FRAM();
      #endif

    #if (USE_SD_SPI > OFF)
        SPIClass psdSPI(VSPI);
        File sdFile;                       // file object that is used to read and write data
      #endif
// ----------------------------------------------------------------
// ---  -----------------------------------
// ----------------------------------------------------------------

// ----------------------------------------------------------------
// --- system setup -----------------------------------
// ----------------------------------------------------------------
  void setup()
    {
      //      uint32_t i32tmp = 0;
      // --- system
        // disable watchdog
          disableCore0WDT();
          #if (BOARD ^ XIAO_ESP32C3)
              disableCore1WDT();
            #endif
          disableLoopWDT();
        // start system
          Serial.begin(SER_BAUDRATE);
          usleep(3000); // power-up safety delay
          SOUTLN(); SOUT(millis()); SOUTLN(" setup start ...");
          #if (SCAN_I2C > OFF)
              scanI2C(&i2c1, PIN_I2C1_SDA, PIN_I2C1_SCL);
              #if (USE_I2C > 1)
                  scanI2C(&i2c2, 0, SCAN_I2C, PIN_I2C2_SDA, PIN_I2C2_SCL);
                #endif
            #endif
          #if (TEST_NUM_CONVERT > OFF)
              int32_t src32   = 200 * 2<<16;
              int32_t src32m  = -src32;
              int16_t src16   = 0x0100;
              int16_t src16m  = -src16;
              uint8_t bits    = 16;
              int16_t dest16cpp  = (int16_t) (src32/(2 << bits));
              int16_t dest16mcpp = (int16_t) (src32m/(pow(2, bits)));
              SOUTLN();
              SOUTLN(" test convertion NUM format");
              SOUT(" (cpp) int32 -> int16: ");
              SOUT(src32); SOUT(" ~ 0x") ; SOUTHEX(src32); SOUT(" -> ");
              SOUT(dest16cpp); SOUT(" ~ 0x") ; SOUTHEXLN(dest16cpp);
              SOUT(src32m); SOUT(" ~ 0x") ; SOUTHEX(src32m); SOUT(" -> ");
              SOUT(dest16mcpp); SOUT(" ~ 0x") ; SOUTHEXLN(dest16mcpp);
              SOUTLN();
            #endif
          #if (USE_LED_BLINK_OUT > 0)
              pinMode(PIN_BOARD_LED, OUTPUT);
              digitalWrite(PIN_BOARD_LED, SYS_LED_ON);
            #endif

      // --- user output
        // start display - output to user
          #if (USE_TRAFFIC_COL16_OUT > 0)
              pinMode(PIN_TL_GREEN, OUTPUT);
              pinMode(PIN_TL_YELLOW, OUTPUT);
              pinMode(PIN_TL_RED, OUTPUT);
              digitalWrite(PIN_TL_GREEN, ON);
              digitalWrite(PIN_TL_RED, ON);
              digitalWrite(PIN_TL_YELLOW, ON);
              usleep(500000);
              digitalWrite(PIN_TL_GREEN, OFF);
              digitalWrite(PIN_TL_RED, OFF);
              digitalWrite(PIN_TL_YELLOW, OFF);
            #endif
          #if (USE_RGBLED_PWM > 0)
              // RGB red
                pinMode(PIN_RGB_RED, OUTPUT);
                ledcSetup(PWM_RGB_RED,    PWM_LEDS_FREQ, PWM_LEDS_RES);
                ledcAttachPin(PIN_RGB_RED,   PWM_RGB_RED);
                ledcWrite(PWM_RGB_RED, 255);
                SOUTLN("LED rot");
                usleep(300000);
                ledcWrite(PWM_RGB_RED, 0);

              // RGB green
                pinMode(PIN_RGB_GREEN, OUTPUT);
                ledcSetup(PWM_RGB_GREEN,  PWM_LEDS_FREQ, PWM_LEDS_RES);
                ledcAttachPin(PIN_RGB_GREEN, PWM_RGB_GREEN);
                ledcWrite(PWM_RGB_GREEN, 255);
                SOUTLN("LED gruen");
                usleep(500000);
                ledcWrite(PWM_RGB_GREEN, 0);

              // RGB blue
                pinMode(PIN_RGB_BLUE, OUTPUT);
                ledcSetup(PWM_RGB_BLUE,   PWM_LEDS_FREQ, PWM_LEDS_RES);
                ledcAttachPin(PIN_RGB_BLUE,  PWM_RGB_BLUE);
                ledcWrite(PWM_RGB_BLUE, 255);
                SOUTLN("LED blau");
                usleep(500000);
                ledcWrite(PWM_RGB_BLUE, 0);

            #endif
          startDisp();
          dispStatus("setup start ...", true);

        // WS2812 LEDs
          #if (USE_WS2812_MATRIX_OUT > OFF)
              SOUTLN("start WS2812 matrix ...");
              dispStatus("start WS2812 Matrix");
              initWS2812Matrix();
              ws2812T1.startT();
              SOUTLN(" ok");
            #endif

          #if (USE_WS2812_LINE_OUT > OFF)
              SOUT("start NEOPIXEL LED strip ... ");
              initWS2812Line();
              sleep(1);
            #endif

        // start buzzer (task)
          #if (USE_BUZZER_PWM > OFF)
              pinMode(PIN_BUZZ, OUTPUT);                                                                               // Setting pin 11 as output
              #ifdef PLAY_MUSIC
                buzz.initMusic(PIN_BUZZ, PWM_BUZZ);
                #if defined(PLAY_START_MUSIC)
                    playSong();
                  #endif
                #if defined(PLAY_START_DINGDONG)
                    buzz.playDingDong();
                  #endif
              #endif
            #endif
        // start key device
          #if defined(KEYS)
              startKeys();
            #endif
        // start fans
          #if (USE_FAN_PWM > OFF)
              // Fan 1
                pinMode(PIN_PWM_FAN_1, OUTPUT);
                ledcSetup(PWM_FAN_1, PWM_FAN_FREQ, PWM_FAN_RES);
                ledcAttachPin(PIN_PWM_FAN_1, PWM_FAN_1);
                ledcWrite(PWM_FAN_1, 255);
                SOUTLN("Test Fan 1");
                sleep(1);
                ledcWrite(PWM_FAN_1, 0);
              // Fan 2
                #if (USE_FAN_PWM > 1)
                    pinMode(PIN_PWM_FAN_2, OUTPUT);
                    ledcSetup(PWM_FAN_2, PWM_FAN_FREQ, PWM_FAN_RES);
                    ledcAttachPin(PIN_PWM_FAN_2, PWM_FAN_2);
                    ledcWrite(PWM_FAN_2, 255);
                    SOUTLN("Test Fan 2");
                    sleep(1);
                    ledcWrite(PWM_FAN_2, 0);
                  #endif

            #endif

        // start freq generator
          #if (USE_BUZZER_PWM > OFF)

            #endif
      // --- user input
        // start digital inputs
          #if (USE_DIG_INP > OFF)
              SOUT("config digSW Pins " );
              #if (USE_WS2812_PWR_IN_SW > OFF)
                  pinMode(PIN_WS2812_PWR_IN_SW, INPUT_PULLUP);
                  SOUT(PIN_WS2812_PWR_IN_SW); SOUT(" ");
                #endif
              #if (USE_CTRL_SW_INP > OFF)
                  pinInpDig[INP_SW_CTRL] = PIN_INP_SW_1;
                  polmodInpDig[INP_SW_CTRL] = POL_SW_CTRL >> 4 + MOD_SW_CTRL;
                #endif
              #if (USE_GEN_SW_INP > OFF)
                  pinInpDig[INP_REED_1] = PIN_INP_REED_1;
                  polInpDig[INP_REED_1] = POL_REED_1;
                  modInpDig[INP_REED_1] = MOD_REED_1;
                  SOUT(" polInpDig "); SOUT(polInpDig[INP_REED_1]); SOUT(" ");
                  SOUT(" modInpDig "); SOUT(modInpDig[INP_REED_1]); SOUT(" ");
                #endif
              for (uint8_t i = 0 ; i < USE_DIG_INP ; i++ )
                {
                  pinMode(pinInpDig[i], modInpDig[i]);
                  SOUT(pinInpDig[i]); SOUT("-"); SOUTHEX(modInpDig[i] & 0x0F);
                }
              SOUTLN();
            #endif
        // start dutycycle (pwm) inputs
          #if (USE_PWM_INP > OFF)
              //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_OUT);
              mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, PIN_PWM_INP_1); // MAGIC LINE to define WHICH GPIO
              // gpio_pulldown_en(GPIO_CAP0_IN); //Enable pull down on CAP0 signal
              mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, 1);
            #endif
      // --- network
        // start WIFI
          #if (USE_WIFI > OFF)
              uint8_t rep = WIFI_ANZ_LOGIN;
              while(rep > 0)
                {
                  startWIFI(true);
                  if ((md_error & ERRBIT_WIFI) == OK)
                      {
                        dispStatus("WIFI connected",true);
                        break;
                      }
                    else
                      {
                        #if (WIFI_IS_DUTY > OFF)
                            dispStatus("WIFI error -> halted", true);
                        #else
                            rep--;
                            if (rep > 0)
                              { dispStatus("WIFI error ..."); }
                            else
                              { dispStatus("WIFI not connected"); }
                          #endif
                      }
                  usleep(50000);
                }
              #if (USE_NTP_SERVER > OFF)   // get time from NTP server
                  if ((md_error & ERRBIT_WIFI) == OK)
                    {
                      dispStatus("init NTP time", true);
                      initNTPTime();
                      ntpGet = true;
                    }
                #endif

            #endif // USE_WIFI
        // start Webserer
          #if (USE_WEBSERVER > OFF)
              {
                servT.startT();
                #if (TEST_SOCKET_SERVER > OFF)
                    //socket.onEvent(onEvent);
                    //serv.addHandler(&socket);

                    //serv.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                    //  {
                    //    request->send_P(200, "text/html", index_html, NULL);
                    //  });

                    // serv.begin();
                #else
                    startWebServer();
                  #endif
                    //md_error = setBit(md_error, ERRBIT_SERVER, webMD.md_handleClient());
              }
            #endif
        // start MQTT
          #if (USE_MQTT > OFF)
              SOUTLN("Connecting to MQTT...");
              //mqttClient.onConnect(onMqttConnect);
              //mqttClient.onDisconnect(onMqttDisconnect);
              //mqttClient.onSubscribe(onMqttSubscribe);
              //mqttClient.onUnsubscribe(onMqttUnsubscribe);
              //mqttClient.onMessage(onMqttMessage);
              //mqttClient.onPublish(onMqttPublish);
              //mqttClient.setServer(MQTT_HOST, MQTT_PORT);
              //mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, mqttID,
                                                //reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
              //mqttClient.connect();
            #endif
      // --- sensors
        // ADC ADS1115
          #if (USE_ADC1115_I2C > OFF)
              #if (ADC1115_1_I2C == I2C1)
                  ads[0].begin(ADC1115_1_ADDR, &i2c1);
                  ads[0].setDataRate(ADS0_DATARATE);
                #else
                  ads[0].begin(ADC1115_1_ADDR, &i2c1);
                #endif
            #endif
        // temp. sensor DS18D20
          #if (USE_DS18B20_1W_IO > OFF)
                  SOUT(millis()); SOUT(" DS18D20 ... " );
              dispStatus("init DS18D20");
              dsSensors.begin();
              String DS18Str = getDS18D20Str();
              dispStatus(DS18Str);
                  SOUTLN(DS18Str);
            #endif
        // BME280 temperature, pessure, humidity
          #if (USE_BME280_I2C > OFF)
                    SOUT(millis()); SOUT(" BME280 ... " );
              dispStatus("init BME2801");
              bool bmeda = false;
              bmeda = bme1.begin(I2C_BME280, pbme1i2c);
              if (bmeda)
                  {
                    bme1.setSampling(bme1.MODE_SLEEP);
                          SOUTLN(" gefunden");
                    bme1T.begin(BME2801T_FILT, BME2801T_Drop, FILT_NU);
                    bme1P.begin(BME2801P_FILT, BME2801P_Drop, FILT_NU);
                    bme1H.begin(BME2801H_FILT, BME2801H_Drop, FILT_NU);
                  }
                else
                  {
                    SOUT(" nicht gefunden");
                  }
              #if (USE_BME280_I2C > 1)
                        SOUT(millis()); SOUT(" BME280 ... " );
                  dispStatus("init BME280");
                  bmeda = false;
                  bmeda = bme2.begin(I2C_BME280, pbme2i2c);
                  if (bmeda)
                      {
                        bme2.setSampling(bme2.MODE_SLEEP);
                              SOUTLN(" gefunden");
                        bme2T.begin(BME2802T_FILT, BME2802T_Drop, FILT_NU);
                        bmeP.begin(BME2802P_FILT, BME2802P_Drop, FILT_NU);
                        bmeH.begin(BME2802H_FILT, BME2802H_Drop, FILT_NU);
                      }
                    else
                      {
                        SOUT(" nicht gefunden");
                      }
                #endif
            #endif
        // photo sensor
          #if (USE_PHOTO_SENS_ANA > OFF)
              SOUT("init poto sensors ... ");
              photoVal[0].begin(PHOTO1_FILT, PHOTO1_DROP, FILT_FL_MEAN);
              photoScal[0].setScale(PHOTO1_SCAL_OFFRAW, PHOTO1_SCAL_GAIN, PHOTO1_SCAL_OFFREAL);
              #if (PHOTO1_ADC > OFF)
                  pinMode(PIN_PHOTO1_SENS, INPUT);
                  adc1_config_channel_atten((adc1_channel_t) ADC_PHOTO1_SENS,
                                            (adc_atten_t)    PHOTO1_ADC_ATT);
                #endif
              #if (PHOTO1_1115 > OFF)

                #endif
              SOUTLN(" ready");
            #endif
        // alcohol sensor
          #if (USE_MQ3_ALK_ANA > OFF)
              SOUT("init alc sensors ... ");
              alkVal[0].begin(MQ3_FILT, MQ3_DROP, FILT_FL_MEAN);
              alkScal[0].setScale(MQ3_OFFRAW, PHOTO1_SCAL_GAIN, PHOTO1_SCAL_OFFREAL);
              SOUTLN(" ready");
            #endif
        // vcc measure
          #if (USE_VCC_ANA > OFF)
              SOUT("init vcc measure ... ");
              vccVal[0].begin(VCC_FILT, VCC_DROP, FILT_FL_MEAN);
              vccScal[0].setScale(VCC_OFFRAW, VCC_GAIN, VCC_OFFREAL);
              SOUTLN(" ready");
            #endif
        // poti measure
          #if (USE_POTI_ANA > OFF)
              SOUT("init poti ... ");
              potiVal[0].begin(POTI1_FILT, POTI1_DROP, FILT_FL_MEAN);
              potiScal[0].setScale(POTI1_OFFRAW, POTI1_GAIN, POTI1_OFFREAL);
              SOUTLN(" ready");
            #endif

        // ACS712 current measurement
          #if (USE_ACS712_ANA > OFF)
              SOUT("init alc sensors ... ");
              i712Val[0].begin(I712_1_FILT, I712_1_DROP, FILT_FL_MEAN);
              i712Scal[0].setScale(I712_1_SCAL_OFFRAW, I712_1_SCAL_GAIN, I712_1_SCAL_OFFREAL);
              SOUTLN(" ready");
            #endif

        // K-type thermoelementation
          #if ( USE_TYPE_K_SPI > 0)
                    SOUT(millis()); SOUT(" Tcouple1 ... " );
                dispStatus("init TypeK");
                uint8_t tkerr = TypeK1.begin();
                if (!tkerr)
                    {
                            SOUT(" gefunden TK1 ");
                      int16_t itmp = TypeK1.actT();
                            SOUT(itmp); SOUT(" TK1cold ");
                      itmp = TypeK1.refT();
                            SOUTLN(itmp);
                    }
                  else
                    {
                      SOUTLN(" nicht gefunden");
                    }
                #if ( USE_TYPE_K_SPI > 1)
                          SOUT(millis()); SOUT(" Tcouple2 ... " );
                      int16_t itmp = 0;
                      tkerr = TypeK2.begin();
                      if (!tkerr)
                          {
                                  SOUT(" gefunden TK2 ");
                            itmp = TypeK2.actT();
                                  SOUT(itmp); SOUT(" TK2cold ");
                            itmp = TypeK2.refT();
                                  SOUTLN(itmp);
                          }
                        else
                          {
                            SOUTLN(" nicht gefunden");
                          }
                  #endif
            #endif
      // --- memories
        // FLASH memory
          #if (USE_FLASH_MEM > OFF)
              testFlash();
            #endif

        // SD card
          #if (USE_SD_SPI > OFF)
              //File sdFile;
              pinMode(SD_CS, OUTPUT); // chip select pin must be set to OUTPUT mode
                    SOUT(" init SD ... ");
              //psdSPI.begin(SD_SCL, SD_MISO, SD_MOSI, SD_CS);
              //psdSPI.end();
              if (SD.begin(SD_CS, psdSPI))
                {
                  if (SD.exists("/test.txt"))
                    { // if "file.txt" exists, fill will be deleted
                      if (SD.remove("/test.txt") == true)
                        { SOUT(" file removed "); }
                    }
                  sdFile = SD.open("/test.txt", FILE_WRITE); // open "file.txt" to write data
                  if (sdFile)
                    {
                      sdFile.println("test ok"); // write test text
                      sdFile.close();
                      SOUT(" wrote text ");
                      sdFile = SD.open("/test.txt", FILE_READ);
                      if (sdFile)
                        {
                          char c = 32;
                          SOUT(" read: ");
                          while (c >= ' ')
                            {
                              c = sdFile.read();
                              SOUT(c);
                            }
                          sdFile.close();
                          SOUTLN(" ready ");
                        }
                      else
                        { SOUTLN(" ERR could not open file (read)"); }
                    }
                  else
                    { SOUTLN(" ERR could not open file (write)"); }
                }
              else
                { SOUTLN(" ERROR SD could not be initialised "); }
            #endif
        // FRAM
          #if (USE_FRAM_I2C > OFF)
            // Read the first byte
            SOUT("FRAM addr "); SOUTHEX(FRAM1_I2C_ADDR);
            dispStatus("init FRAM");
            bool ret = !fram.begin(FRAM1_I2C_SDA, FRAM1_I2C_SCL, FRAM1_I2C_ADDR);
            if (ret == ISOK)
              {
                SOUT(" ok ProdID= ");
                uint16_t prodID, manuID;
                fram.getDeviceID(&manuID, &prodID);
                SOUT(" product "); SOUT(prodID); SOUT(" producer "); SOUTLN(manuID);

                SOUTLN(" FRAM selftest "); SOUTLN(fram.selftest());
              }
            #endif
      // --- services using interrupt
        // start counter
          #if (USE_CNT_INP > OFF)
              SOUT("config counter Pins " );
              for (uint8_t i = 0 ; i < USE_CNT_INP ; i++ )
                {
                  switch (i)
                    {
                      case 0:
                        //pinMode(PCNT0_SIO, INPUT_PULLUP);     SOUT(PCNT0_SIO); SOUT(" - ");
                        pinMode(PCNT0_SIO, INPUT);     SOUT(PCNT0_SIO); SOUT(" - ");
                        //pinMode(PCNT0_CIO, OUTPUT);           SOUT(PCNT0_CIO); SOUT(" ");
                        break;
                      #if (USE_CNT_INP > 1)
                          case 1:
                          pinMode(PCNT1_SIO, INPUT_PULLUP);   SOUT(PCNT1_SIO); SOUT(" - ");
                          //pinMode(PCNT1_CIO, OUTPUT);         SOUT(PCNT1_CIO); SOUT(" ");
                          break;
                        #endif
                      #if (USE_CNT_INP > 2)
                          case 2:
                          pinMode(PCNT2_SIO, INPUT_PULLUP); SOUT(PCNT2_CIO); SOUT(" ");
                          //pinMode(PCNT2_CIO, OUTPUT); SOUT(PCNT2_CIO); SOUT(" ");
                          break;
                        #endif
                      #if (USE_CNT_INP > 3)
                          case 3:
                          pinMode(PCNT3_SIO, INPUT_PULLUP); SOUT(PCNT3_CIO); SOUT(" ");
                          //pinMode(PCNT3_CIO, OUTPUT); SOUT(PCNT3_CIO); SOUT(" ");
                          break;
                        #endif
                      default: break;
                    }
                }
              SOUTLN();
              initGenPCNT();
                      //attachInterrupt(digitalPinToInterrupt(PIN_PWM_FAN_1), &pcnt_intr_handler, FALLING);

              #ifdef USE_MCPWM
                  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, PIN_CNT_FAN_1); // MAGIC LINE to define WHICH GPIO
                  // gpio_pulldown_en(GPIO_CAP0_IN); //Enable pull down on CAP0 signal
                  mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, 0);
                #endif
            #endif
      // --- finish setup
          #if (DEBUG_MODE >= CFG_DEBUG_STARTUP)
              #if (USE_COL16_BLINK_OUT > 0)
                  digitalWrite(PIN_BOARD_LED, OFF);
                  SYS_LED_ON = OFF;
                #endif
              SOUTLN();
              dispStatus("... end setup");
              SOUT("... end setup -- error="); SOUTLN(md_error);
              SOUTLN();
              usleep(400000);
            #endif
    }

// ----------------------------------------------------------------
// --- system run = endless loop
// ----------------------------------------------------------------
  void loop()
    {
      anzUsCycles++;
      //SOUT(" "); SOUT(millis());
      #if ( USE_DISP > 0 )
          outStr   = "";
        #endif
      //tmpval32 = heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_32BIT);
      //heapFree("+loop");
      if (tmpval32 < freeHeap)
        {
          freeHeap = tmpval32;
          SOUT(millis()); SOUT(" loop "); SOUT(dispIdx); SOUT(" freeHeap "); SOUTLN(freeHeap);
        }
      if (firstrun == true)
        {
          String taskMessage = "loop task running on core ";
          taskMessage = taskMessage + xPortGetCoreID();
          SOUTLN(taskMessage);
          usLast = micros();
          firstrun = false;
        }
      //uint16_t t_x = 0, t_y = 0; // To store the touch coordinates
      // --- network ---
        #if (XXXUSE_WIFI > OFF)  // restart WIFI if offline
            if(wifiT.TOut())
              {
                //Serial.print("WiFi md_error = "); Serial.println(md_error);
                wifiT.startT();
                if((md_error & ERRBIT_WIFI) != OK)
                  {
                    SOUTLN("WiFi restartWIFI");
                    dispStatus("WiFi restartWIFI");
                    startWIFI(false);
                  }
              }
          #endif // USE_WIFI

        #if (USE_NTP_SERVER > OFF)   // get time from NTP server
            if (ntpT.TOut() == true)
              {
                setTime(++ntpTime);
                if ((md_error & ERRBIT_WIFI) == OK)
                  { // WiFi online
                    #ifdef UNUSED
                        if (((md_error & ERRBIT_NTPTIME) > 0) || (year() < 2000))   // time not initialized
                          {
                            initNTPTime();
                            ntpGet = true;
                          }
                      #endif

                    if (ntpGet == true)
                      {
                        ntpGet = wifi.getNTPTime(&ntpTime);
                        setTime(ntpTime);
                        ////////////////////////////////SOUT(" NTP time "); SOUT(ntpTime);
                      }
                  }
                ntpT.startT();
                      #if (DEBUG_MODE == CFG_DEBUG_DETAILS)
                        //SOUT("Datum "); SOUT(day()); SOUT("."); SOUT(month()); SOUT("."); SOUT(year()); SOUT(" ");
                        //SOUT("Zeit "); SOUT(hour()); SOUT("."); SOUT(minute()); SOUT(":"); SOUTLN(second());
                        #endif
              }
          #endif // USE_NTP_SERVER
        #if (USE_WEBSERVER > OFF)    // run webserver -> restart/run not allowed in loop task
            // read only 1 message / cycle for cycle time
            //heapFree("webserv +readmsg");
            readMessage();
            //heapFree("webserv -readmsg");
          #endif
      // --- direct input ---
        #if (USE_TOUCHSCREEN > OFF)
          //touch.runTouch(outBuf);
          #endif // USE_TOUCHSCREEN
        #if (USE_KEYPADSHIELD > OFF)
          key = getKey();
          if (key)
            {
              sprintf(outBuf,"key %d", key);
              dispStatus(outBuf);
            }
          #endif
        #if (USE_CNT_INP > OFF)
            uint64_t        lim  = 0ul;
            pcnt_evt_type_t ev;
            uint8_t         doIt = false;
            pcnt_unit_t     unit;
            sprintf(cmsg,"  loop/cnt_inp");
            for ( uint8_t i = 0; i < USE_CNT_INP ; i++ )
              {
                switch (i)
                  {
                    case 0:
                      lim  = PCNT0_UFLOW;
                      ev   = PCNT0_EVT_0;
                      unit = PCNT0_UNIT;
                      pcnt_res = xQueueReceive(pcnt_evt_queue[PCNT0_UNIT], &cntErg[i], 0);
                      break;
                    #if (USE_CNT_INP > 1)
                        case 1:
                          lim  = PCNT1_UFLOW;
                          ev   = PCNT1_EVT_0;
                          unit = PCNT1_UNIT;
                          pcnt_res = xQueueReceive(pcnt_evt_queue[PCNT1_UNIT], &cntErg[i], 0);
                          break;
                      #endif
                    #if (USE_CNT_INP > 2)
                        case 2:
                          lim  = PCNT2_UFLOW;
                          ev   = PCNT2_EVT_0;
                          unit = PCNT2_UNIT;
                          pcnt_res = xQueueReceive(pcnt_evt_queue[PCNT2_UNIT], &tmpErg, 0);
                          break;
                      #endif
                    #if (USE_CNT_INP > 3)
                        case 3:
                          lim  = PCNT3_UFLOW;
                          ev   = PCNT3_EVT_0;
                          unit = PCNT3_UNIT;
                          pcnt_res = xQueueReceive(pcnt_evt_queue[PCNT3_UNIT], &tmpErg, 0);
                          break;
                      #endif
                    default:
                      break;
                  }
                if (pcnt_res == pdTRUE)
                  {
                            //if (i == 0) { SOUT(cmsg); }
                            //SOUT("  "); SOUT(millis()); SOUT("  "); SOUT(i);
                            //SOUT(" "); SOUT("usCnt"); SOUT(" "); SOUT(cntErg[i].usCnt);
                            //SOUT(" T "); SOUT(cntThresh[i]); Serial.flush();

                    if ( (cntErg[i].usCnt > 0) )
                      {
                        cntErg[i].freq = (uint16_t) (1000000ul * cntThresh[i] * cntFakt[i] / cntErg[i].usCnt);
                          //cntErg[i].freq = (uint16_t) (cntErg[i].pulsCnt * 1000000ul / cntErg[i].usCnt);
                        //SOUT(" "); SOUT(cntErg[i].freq); Serial.flush();
                      }
                    else
                      {
                        cntErg[i].freq = 0;
                      }
                            //SOUT(" "); SOUT(i); SOUT(" "); SOUT((uint32_t) cntErg[i].freq);
                  }
                // autorange
                #if (USE_CNT_AUTORANGE > OFF)
                    // check for auto range switching

                    if (cntErg[i].usCnt > PNCT_AUTO_SWDN)
                      { // low freq
                        if (cntFilt[i] > -5)
                          {
                            SOUTLN(); SOUT("SWDN filt "); SOUTLN(cntFilt[i]);
                            cntFilt[i]--;
                            usleep(500000);
                          }
                        if ((cntThresh[i] > 1) && (cntFilt[i] > -5))
                          {
                            cntThresh[i] /= 2;
                            doIt = true;
                            SOUTLN(); SOUT("SWDN new "); SOUTLN(cntThresh[i]);
                            usleep(500000);
                          }
                      }
                    else if ( (cntErg[i].usCnt < PNCT_AUTO_SWUP) && (cntErg[i].pulsCnt > 0) )
                      { // high freq
                        if (cntFilt[i] < 5)
                          {
                            SOUTLN(); SOUT("SWUP filt "); SOUTLN(cntFilt[i]);
                            cntFilt[i]++;
                            usleep(500000);
                          }
                        if ((cntThresh[i] < 16) && (cntFilt[i] > 5))
                          {
                            cntThresh[i] *= 2;
                            doIt = true;
                            SOUTLN(); SOUT("SWUP new "); SOUTLN(cntThresh[i]);
                            usleep(500000);
                          }
                      }
                    else
                      {
                        cntFilt[i] = 0;
                      }

                    if (doIt)
                      {
                        pcnt_counter_pause(unit);
                        logESP(pcnt_event_disable  (unit, ev),            cmsg, i);
                        logESP(pcnt_set_event_value(unit, ev, cntThresh[i]), cmsg, i);
                        pcnt_counter_clear(unit);
                        pcnt_counter_resume(unit);
                        logESP(pcnt_event_enable   (unit, ev),            cmsg, i);
                        doIt = false;
                        cntThresh[i] = 0;
                      }
                  #endif // USE_CNT_AUTORANGE
              }
                      //Serial.flush();
          #endif
        #if (USE_PWM_INP > OFF)
            mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, 1);
            pwmInVal->lowVal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);

            mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 1);
            pwmInVal->highVal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);
          #endif
      // --- standard input cycle ---
        #ifdef USE_INPUT_CYCLE
            if (inputT.TOut())
              {
                inpIdx++;
                    //heapFree("+meascyc");
                    //SOUT(" #"); SOUT(millis()); SOUTLN(" MEASCYCLE ");
                inputT.startT();
                switch(inpIdx)
                  {
                    case 1:
                      #if (USE_BME280_I2C > OFF)
                          bme1.init();
                          usleep(100);
                          bme1T.doVal((int16_t)  ( bme1.readTemperature() + 0.5));
                          bme1H.doVal((uint16_t) ( bme1.readHumidity() + 0.5));
                          bme1P.doVal((uint16_t) ((bme1.readPressure() / 100.0F) + 0.5));
                        #endif
                      break;
                    case 2:
                      #if (USE_PHOTO_SENS_ANA > OFF)
                          #if (PHOTO1_ADC > OFF)
                              photoVal[0].doVal(analogRead(PIN_PHOTO1_SENS));
                            #endif
                          #if (PHOTO1_1115 > OFF)
                            #endif
                        #endif
                      break;
                    case 3:
                      #if (USE_MQ135_GAS_ANA > OFF)
                          #if (MQ135_GAS_ADC > OFF)
                              gasVal.doVal(analogRead(PIN_MQ135));
                                    //SOUT(millis()); SOUT(" gas measurment val = "); SOUTLN(gasValue);
                              gasValue = (int16_t) valGas.value((double) gasValue);
                                    //SOUT(millis()); SOUT("    gasValue = "); SOUTLN(gasValue);
                            #endif
                          #if (MQ135_GAS_1115 > OFF)
                            #endif
                        #endif
                      break;
                    case 4:
                      #if (USE_MQ3_ALK_ANA > OFF)
                          #if (MQ3_ALK_ADC > OFF)
                            #endif
                          #if (MQ3_ALK_1115 > OFF)
                              ads[0].setGain(MQ3_1115_ATT);
                              //ads[0].setDataRate(RATE_ADS1115_860SPS);
                              ads[0].startADCReading(MUX_BY_CHANNEL[MQ3_1115_CHAN], /*continuous=*/false);
                              usleep(1200); // Wait for the conversion to complete
                              while (!ads[0].conversionComplete());
                              alk[0] = ads[0].getLastConversionResults();   // Read the conversion results
                              alkVal[0].doVal(alk[0]);   // Read the conversion results
                              //alk[0] = (uint16_t) (1000 * ads[0].computeVolts(alkVal[0].doVal(ads->readADC_SingleEnded(MQ3_1115_CHAN))));
                            #endif
                        #endif
                      break;
                    case 5:
                      #if (USE_VCC_ANA > OFF)
                          #if (VCC_ADC > OFF)
                            #endif
                          #if (VCC_1115 > OFF)
                              ads[VCC50_IDX].setGain(VCC_1115_ATT);
                              ads[VCC50_IDX].startADCReading(MUX_BY_CHANNEL[VCC_1115_CHAN], /*continuous=*/false);
                              usleep(1200); // Wait for the conversion to complete
                              while (!ads[VCC50_IDX].conversionComplete());
                              vcc[VCC50_IDX] = ads[VCC50_IDX].getLastConversionResults();   // Read the conversion results
                              vccVal[VCC50_IDX].doVal(vcc[VCC50_IDX]);
                              //vcc[VCC50_IDX] = (uint16_t) (1000 * ads[VCC50_IDX].computeVolts(vccVal[VCC50_IDX].doVal(ads[VCC50_IDX].readADC_SingleEnded(VCC_1115_CHAN))));
                              #if (VCC_1115 > 1)
                                  ads[VCC33_IDX].setGain(VCC_1115_ATT);
                                  ads[VCC33_IDX].startADCReading(MUX_BY_CHANNEL[VCC_1115_CHAN], /*continuous=*/false);
                                  usleep(1200); // Wait for the conversion to complete
                                  while (!ads[VCC33_IDX].conversionComplete());
                                  vcc[VCC33_IDX] = ads[VCC33_IDX].getLastConversionResults();   // Read the conversion results
                                  vccVal[VCC33_IDX].doVal(vcc[VCC33_IDX]);
                                  //vcc[VCC33_IDX] = (uint16_t) (1000 * ads[VCC33_IDX].computeVolts(vccVal[VCC33_IDX].doVal(ads[VCC33_IDX].readADC_SingleEnded(VCC_1115_CHAN))));
                                #endif
                            #endif
                        #endif
                      break;
                    case 6:
                      #if (USE_POTI_ANA > OFF)
                          #if (POTI1_ADC > OFF)
                            #endif
                          #if (POTI1_1115 > OFF)
                              ads[0].setGain(POTI1_1115_ATT);
                              ads[0].startADCReading(MUX_BY_CHANNEL[POTI1_1115_CHAN], /*continuous=*/false);
                              usleep(1200); // Wait for the conversion to complete
                              while (!ads[0].conversionComplete());
                              poti[0] = ads[0].getLastConversionResults();   // Read the conversion results
                              potiVal[0].doVal(poti[0]);
                              //poti[0] = (uint16_t) (1000 * ads[0].computeVolts(potiVal[0].doVal(ads->readADC_SingleEnded(POTI1_1115_CHAN))));
                            #endif
                        #endif
                      break;
                    case 7:
                      #if (USE_ACS712_ANA > OFF)
                          #if (I712_1_ADC > OFF)
                            #endif
                          #if (I712_1_1115 > OFF)
                              ads[0].setGain(I712_1_1115_ATT);
                              ads[0].startADCReading(MUX_BY_CHANNEL[I712_1_1115_CHAN], /*continuous=*/false);
                              usleep(1200); // Wait for the conversion to complete
                              while (!ads[0].conversionComplete());
                              i712[0] = ads[0].getLastConversionResults();   // Read the conversion results
                              i712Val[0].doVal(i712[0]);
                              // = (uint16_t) (1000 * ads[0].computeVolts(i712Val[0].doVal(ads->readADC_SingleEnded(I712_1_1115_CHAN))));
                            #endif
                        #endif
                      break;
                    case 8:
                      #if (USE_CNT_INP > OFF)
                          #ifdef USE_PW
                              getCNTIn();
                            #endif
                        #endif

                      break;
                    case 9:
                      #if (USE_TYPE_K_SPI > OFF)
                          int8_t  tkerr = (int8_t) ISOK;
                          int16_t ival = TypeK1.actT();
                          tkerr = TypeK1.readErr();
                                //SOUT(" typeK1 err "); SOUT(tkerr);
                                //SOUT(" val "); SOUT( ival);
                          if (!tkerr)
                            {
                              tk1Val    = valTK1.value((double) ival);
                                //SOUT(" / "); SOUT(tk1Val);
                              ival      = TypeK1.refT();
                                    //SOUT(millis()); SOUT(" ref raw = "); SOUT((int) ival);
                              tk1ValRef = valTK1ref.value((double) ival);
                                //SOUT(millis()); SOUT(" ival = "); SOUT((int) tk1ValRef);
                            }
                                //SOUTLN();
                          #if (USE_TYPE_K_SPI > 1)
                              ival    = TypeK2.actT();
                              tkerr     = TypeK2.readErr() % 8;
                                    //SOUT(" typeK2 err "); SOUT(tkerr);
                                    //SOUT(" val "); SOUT(ival);
                              if (!tkerr)
                                {
                                  tk2Val    = valTK2.value((double) ival);
                                    //SOUT(" / "); SOUT((int) tk2Val);
                                  ival      = TypeK2.refT();
                                    //SOUT(millis()); SOUT(" ref raw = "); SOUT(ival);
                                  tk2ValRef = valTK2ref.value((double) ival);
                                    //SOUT(millis()); SOUT(" ival = "); SOUT(tk2ValRef);
                                }
                                    //SOUTLN();
                          #else
                                    SOUTLN();
                            #endif
                        #endif

                      break;
                    case 10:
                      #if (USE_DIG_INP > OFF)
                          getDIGIn();
                        #endif
                      break;
                    case 11:
                      #if (USE_ESPHALL > OFF)
                          valHall = hallRead();
                        #endif
                      break;
                    case 12:
                      #if (USE_MCPWM > OFF)
                          getCNTIn();
                        #endif
                      break;
                    default:
                      inpIdx = 0;
                      break;
                  }
                    //heapFree("-meascyc");
              }
          #endif
      // --- direct output ---
        #if (USE_WS2812_MATRIX_OUT > OFF)
            //if (ws2812T1.TOut())
              //{
                //ws2812T1.startT();
                  if (!(outM2812[0].bmpB->pix24 == outM2812[1].bmpB->pix24)) //{}
                    //else
                    {
                      outM2812[0].bmpB->pix24 = outM2812[1].bmpB->pix24;
                      outM2812[0].bmpE->pix24 = outM2812[1].bmpE->pix24;
                    }

                    //if (strcmp(outM2812[0].text->text, outM2812[1].text->text) > 0)
                  if (strncmp((char*) outM2812[0].text, (char*) outM2812[1].text, sizeof(outM2812[1].text)) > 0)
                    {
                      memcpy((char*) outM2812[0].text, (char*) outM2812[1].text, sizeof(outM2812[1].text));
                        //memcpy(outM2812[0].text->text, outM2812[1].text->text, sizeof((char*) outM2812[1].text->text));
                    }

                      //SOUT(" matrix 0/1  "); SOUTHEX((uint32_t)  outM2812[0].text->pix24); SOUT("/"); SOUTHEXLN((uint32_t)  outM2812[0].text->pix24);
                      //SOUT(" matrix 0/1 *"); SOUTHEX((uint32_t) *outM2812[0].text->pix24); SOUT("/"); SOUTHEXLN((uint32_t) *outM2812[0].text->pix24);
                  //md_LEDPix24* ppix = *outM2812[0].text->pix24
                  if (!(outM2812[0].text->pix24 == outM2812[1].text->pix24)) //{}
                    //else
                    {
                      //SOUTLN(" changed matrix bright&color ");
                        //SOUT((uint32_t) *outM2812[0].text->pix24); SOUT(" / "); SOUTHEXLN((uint32_t) *outM2812[1].text->pix24);
                        //SOUTHEX(outM2812[0].text->pix24->bright()); SOUT(" "); SOUTHEX(  outM2812[0].text->pix24->col24());
                        //SOUT(" / ");
                        //SOUTHEX(outM2812[1].text->pix24->bright()); SOUT(" "); SOUTHEXLN(outM2812[1].text->pix24->col24());
                      outM2812[0].text->pix24 = outM2812[1].text->pix24;
                        //SOUTLN(" neu ");
                        //SOUT((uint32_t) *outM2812[0].text->pix24); SOUT(" / "); SOUTHEXLN((uint32_t) *outM2812[1].text->pix24);
                        //SOUTHEX(outM2812[0].text->pix24->bright()); SOUT(" "); SOUTHEX(  outM2812[0].text->pix24->col24());
                        //SOUT(" / ");
                        //SOUTHEX(outM2812[1].text->pix24->bright()); SOUT(" "); SOUTHEXLN(outM2812[1].text->pix24->col24());
                    }

                      /*
                        if (outM2812[0] == outM2812[1]) {}
                        else
                          {
                            SOUT(" changed matrix bitmap "); SOUT(outM2812[0].text.pix24.bright());
                            *outM2812[0].bmpE.pix24 = *outM2812[1].bmpE.pix24;
                          }

                        if (doIt == true)
                          {
                            //SOUT(" do start-matrix ");
                            matrix_1.start_scroll_matrix((scroll2812_t*) &outM2812);
                          }
                      */
                      //SOUT(" "); SOUTLN(micros());
                matrix_1.scroll_matrix();
                    //SOUT(" matrix ready "); SOUTLN(micros());
                ws2812_Mcnt++;
              //}
          #endif
        #if (USE_WS2812_LINE_OUT > OFF)
            //if (ws2812LT.TOut())
              //{
                //ws2812LT.startT();
                //SOUT(" outCycle line 0/1 "); SOUTHEX(*line2812[0]); SOUT("/"); SOUTHEX(*line2812[1]);
                //SOUT(" bright 0/1 "); SOUTHEX(line2812[0]->bright()); SOUT("/"); SOUTHEX(line2812[1]->bright());
                //SOUT(" col24 0/1 "); SOUTHEX(line2812[0]->col24()); SOUT("/"); SOUTHEXLN(line2812[1]->col24());
                if (*(line2812[1]) == *(line2812[0])) {}
                  else
                  {
                    SOUTLN(" line changed ");
                    *line2812[0] = *line2812[1];
                    strip.setBrightness(line2812[0]->bright());
                    strip.fill(line2812[0]->col24());
                    strip.show();
                  }
              //}
          #endif
      // --- standard output cycle ---
        #ifdef USE_OUTPUT_CYCLE
            if (outpT.TOut())
              {
                outpIdx++;
                outpT.startT();
                switch(outpIdx)
                  {
                    case 1:
                      #if (USE_RGBLED_PWM > OFF)
                          if (rgbledT.TOut())
                            {
                                  //SOUT(" #"); SOUT(millis()); SOUTLN(" Out RGBLED");
                              rgbledT.startT();
                              #if (TEST_RGBLED_PWM > OFF)
                                /*
                                  switch (colRGBLED)
                                    {
                                      case 0:
                                        if (RGBLED_rt >= 254)
                                          {
                                            RGBLED_rt = 0;
                                            RGBLED_gr += incRGBLED;
                                            colRGBLED++;
                                          }
                                          else
                                          { RGBLED_rt += incRGBLED; }
                                        break;
                                      case 1:
                                        if (RGBLED_gr >= 254)
                                          {
                                            RGBLED_gr = 0;
                                            RGBLED_bl += incRGBLED;
                                            colRGBLED++;
                                          }
                                          else
                                          { RGBLED_gr += incRGBLED; }
                                        break;
                                      case 2:
                                        if (RGBLED_bl >= 254)
                                          {
                                            RGBLED_bl = 0;
                                            RGBLED_rt += incRGBLED;
                                            colRGBLED = 0;
                                          }
                                          else
                                          { RGBLED_bl += incRGBLED; }
                                        break;
                                      default:
                                        break;
                                    }
                                  */

                                  #if (USE_WEBCTRL_RGB > OFF)
                                      _tmp += 4;
                                      if (_tmp > 50)
                                        { _tmp = 0; }
                                      //SOUT(millis()); SOUT(" _tmp = "); SOUTLN(_tmp);
                                      ledcWrite(PWM_RGB_RED,   webMD.getDutyCycle(0));
                                      ledcWrite(PWM_RGB_GREEN, webMD.getDutyCycle(1));
                                      ledcWrite(PWM_RGB_BLUE,  webMD.getDutyCycle(2));
                                    #endif


                                  if(*RGBLED[0] == *RGBLED[1]) {}
                                    else
                                    {
                                      SOUT(" RGBLED changed 0/1 "); SOUTHEX((uint32_t) *RGBLED[0]);
                                      SOUT(" / "); SOUTHEXLN((uint32_t) *RGBLED[1]);
                                      *RGBLED[0] = *RGBLED[1];
                                      //ledcWrite(PWM_RGB_RED,   BrightCol(RGBLED[0][LED_RED],RGBLED[0][LED_BRIGHT]));
                                      //ledcWrite(PWM_RGB_GREEN, BrightCol(RGBLED[0][LED_GREEN],RGBLED[0][LED_BRIGHT]));
                                      //ledcWrite(PWM_RGB_BLUE, BrightCol()  BrightCol(RGBLED[0][LED_BLUE],RGBLED[0][LED_BRIGHT]));
                                      ledcWrite(PWM_RGB_RED,   Bright_x_Col(Red(RGBLED[0]->col24()),   RGBLED[0]->bright()));
                                      ledcWrite(PWM_RGB_GREEN, Bright_x_Col(Green(RGBLED[0]->col24()), RGBLED[0]->bright()));
                                      ledcWrite(PWM_RGB_BLUE,  Bright_x_Col(Blue(RGBLED[0]->col24()),  RGBLED[0]->bright()));
                                    }
                                #endif
                              // update changes from webserver
                              if (LEDout)
                                {
                                  LEDout = (uint8_t) map(RGBLED[1]->bright(), 0, 255,
                                                         0, Green(RGBLED[1]->col24()));
                                  ledcWrite(PWM_RGB_GREEN, LEDout);
                                  LEDout = (uint8_t) map(RGBLED[1]->bright(), 0, 255,
                                                         0, Red(RGBLED[1]->col24()));
                                  ledcWrite(PWM_RGB_RED, LEDout);
                                  LEDout = (uint8_t) map(RGBLED[1]->bright(), 0, 255,
                                                         0, Blue(RGBLED[1]->col24()));
                                  ledcWrite(PWM_RGB_BLUE, LEDout);
                                  LEDout = FALSE;
                                }
                            }
                        #endif
                      break;
                    case 2:
                      #if (USE_FAN_PWM > OFF)
                          if (fanT.TOut())
                            {
                                  //SOUT(" #"); SOUT(millis()); SOUTLN(" Out FAN");
                              fanT.startT();
                              if (fanIdx++ > 1000)
                                {
                                  fanIdx = 0;
                                  for (uint8_t i=0 ; i < USE_FAN_PWM ; i++)
                                    {
                                      valFanPWM[i] += 1;
                                      if (valFanPWM[i] >= 255) { valFanPWM[i] = 0; } // -50%
                                    }

                                  #if (USE_POTICTRL_FAN > 0)
                                      valFan[INP_CNT_FAN_1] = map((long) -inpValADC[INP_POTI_CTRL], -4095, 0, 0, 255);
                                      //SOUT(inpValADC[INP_POTI_CTRL]); SOUT(" "); SOUTLN(valFan[INP_CNT_FAN_1]);
                                      valFanPWM[0] = valFan[INP_CNT_FAN_1];
                                      #if (USE_POTICTRL_FAN > 1)
                                          valFan[INP_CNT_FAN_2] = map((long) -inpValADC[INP_POTI_CTRL], -4095, 0, 0, 255);
                                          valFanPWM[1] = valFan[INP_CNT_FAN_2];
                                        #endif
                                    #endif

                                  ledcWrite(PWM_FAN_1, valFanPWM[0]);
                                  #if (USE_FAN_PWM > 1)
                                      ledcWrite(PWM_FAN_2, valFanPWM[1]);
                                    #endif
                                }
                            }
                        #endif
                      break;
                    case 3:
                      #if (USE_WEBSERVER > OFF)
                          if (newClient)
                            {
                              char ctmp[8] = "";
                              // EL_TSLIDER
                              #if (USE_RGBLED_PWM > OFF)
                                  outStr = "SVB1";
                                  outStr.concat(RGBLED[0]->bright());    // RGB-LED col24
                                  pmdServ->updateAll(outStr);
                                  SOUTLN(outStr);
                                #endif
                              #if (USE_WS2812_LINE_OUT > OFF)
                                  outStr = "SVB2";
                                  outStr.concat(line2812[0]->bright());    // RGB-LED col24
                                  pmdServ->updateAll(outStr);
                                  SOUTLN(outStr);
                                #endif
                              #if (USE_WS2812_MATRIX_OUT > OFF)
                                  outStr = "SVB3";
                                  md_LEDPix24* ppix = outM2812[0].text->pix24;
                                  outStr.concat(ppix->bright());           // RGB-LED col24
                                  pmdServ->updateAll(outStr);
                                  SOUTLN(outStr);                              outStr = "SVB3";
                                #endif
                                  //tmpStr = "SVB4";
                                  //tmpStr.concat(line2812[0]->bright());    // RGB-LED col24
                                  //pmdServ->updateAll(tmpStr);

                              // EL_TCOLOR
                              #if (USE_RGBLED_PWM > OFF)
                                  outStr = "SVC1";
                                  colToHexStr(ctmp, RGBLED[0]->col24());
                                  outStr.concat(ctmp);    // RGB-LED col24
                                  pmdServ->updateAll(outStr);
                                  SOUTLN(outStr);
                                #endif
                              #if (USE_WS2812_LINE_OUT > OFF)
                                  outStr = "SVC2";
                                  colToHexStr(ctmp, line2812[0]->col24());
                                  outStr.concat(ctmp);    // RGB-LED col24
                                  pmdServ->updateAll(outStr);
                                  SOUTLN(outStr);
                                #endif
                              #if (USE_WS2812_MATRIX_OUT > OFF)
                                  outStr = "SVC3";
                                  ppix = outM2812[0].text->pix24;
                                  colToHexStr(ctmp, ppix->col24());
                                  outStr.concat(ctmp);    // RGB-LED col24
                                  pmdServ->updateAll(outStr);
                                  SOUTLN(outStr);
                                  outStr = "SVC4";
                                  ppix = outM2812[0].bmpB->pix24;
                                  colToHexStr(ctmp, ppix->col24());
                                  outStr.concat(ctmp);    // RGB-LED col24
                                  pmdServ->updateAll(outStr);
                                  SOUTLN(outStr);
                                #endif

                              newClient = false;
                            }
                        #endif
                      break;
                    default:
                      outpIdx = 0;
                      break;
                  }
              }
          #endif
      // --- Display -------------------
        #if (USE_DISP > 0)
          if (dispT.TOut())    // handle touch output
            {
              dispIdx++;
                    SOUT(" #"); SOUT(millis()); SOUT(" Display dispIdx ... "); SOUT(dispIdx); SOUT(" ");
                    heapFree("+disp");
              #ifdef RUN_OLED_TEST
                  oled.clearBuffer();
                  switch (dispIdx)
                    {
                      case 0:
                        oled.prepare();
                        oled.box_frame();
                        break;
                      case 1:
                        oled.disc_circle();
                        oled.sendBuffer();
                        break;
                      case 2:
                        oled.r_frame_box();
                        break;
                      case 3:
                        oled.prepare();
                        oled.string_orientation();
                        dispIdx--;
                        break;
                      case 4:
                        oled.line();
                        break;
                      case 5:
                        oled.triangle();
                        break;
                      case 6:
                        oled.bitmap();
                        break;
                      default:
                        break;
                    }
                  if (++dispIdx > 6) { dispIdx = 0; }
                  oled.sendBuffer();
                #endif // RUN_OLED_TEST
                    //heapFree("+dispIdx");
              switch (dispIdx)
                {
                case 1:  // system output
                    usTmp      = micros();
                    usPerCycle = (usTmp - usLast) / anzUsCycles;
                    usLast      = usTmp;
                      //SOUT(usLast); SOUT(" "); SOUT(micros()); SOUT(" "); SOUTLN(usPerCycle);
                      //outStr = "          ";
                      //dispText(outStr ,  22, 4, outStr.length());
                      //outStr = "";
                      //outStr.concat((unsigned long) usPerCycle);
                      //outStr.concat("us    ");
                      //dispText(outStr ,  22, 4, outStr.length());
                      //SOUTLN(); SOUT(usLast); SOUT(" ms/cyc "); SOUT((uint32_t) usPerCycle); SOUT(" ");
                    anzUsCycles = 0ul;
                  break;
                case 2:  // webserver nu
                    #if (USE_WIFI > OFF)
                        outStr = WiFi.localIP().toString();
                    #else
                        outStr = "IP Offline";
                      #endif
                    dispText(outStr ,  0, 4, outStr.length());
                  break;
                case 3:  // k-type sensor
                  #if (USE_TYPE_K_SPI > OFF)
                    outStr = "";
                    outStr = "TK1 ";
                    outStr.concat(tk1Val);
                    outStr.concat("°");
                    //dispText(outStr ,  0, 1, outStr.length());
                    #if (USE_TYPE_K_SPI > 1)
                        //outStr = "";
                        outStr.concat(" TK2 ");
                        outStr.concat(tk2Val);
                        outStr.concat("° ");
                      #endif
                    dispText(outStr ,  0, 2, outStr.length());
                    outStr.concat(" (");
                    outStr.concat(tk1ValRef);
                    #if (USE_TYPE_K > 1)
                        outStr.concat("° / ");
                        outStr.concat(tk2ValRef);
                      #endif
                    outStr.concat("°)");
                            SOUTLN(outStr); SOUT(" ");
                    #endif
                  break;
                case 4:  // gas sensor
                  #if (USE_MQ135_GAS_ANA > OFF)
                      //_tmp = showTrafficLight(gasValue, gasThres); // -> rel to defined break point
                      outStr = "CO2 ";
                      outStr.concat(gasValue);
                      outStr.concat("  ");
                      dispText(outStr, 17, 3, outStr.length());
                            //SOUTLN(outStr);
                    #endif
                  break;
                case 5:  // gas sensor
                  #if (USE_MQ3_ALK_ANA > OFF)
                      tmpval16 = alk[0];
                      #if (USE_MQTT > OFF)
                          sprintf(tmpMQTT, "%s%s", MQTT_DEVICE, MQ3_MQTT);
                        #endif
                      outStr = "  a ";
                      outStr.concat(alk[0]);
                      outStr.concat("  ");
                      dispText(outStr, 1, 1, outStr.length());
                            //SOUT(outStr);
                            //SOUTLN(outStr);
                            //SOUT(outStr); SOUT(" ");
                      #if (USE_MQTT > OFF)
                          sprintf(tmpOut, "%d", tmpval16);
                      //    mqttClient.publish(tmpMQTT, 0, true, tmpOut, 6);
                                //SOUT(tmpOut); SOUT(" ");
                        #endif
                    #endif
                  break;
                case 6:  // light sensor
                  #if (USE_PHOTO_SENS_ANA > OFF)
                      outStr = "          ";
                      dispText(outStr, 12, 4, outStr.length());
                      outStr = "";
                      outStr.concat(photoVal[0].getVal());
                      #if (USE_WEBSERVER > OFF)
                          tmpval16 = photoVal[0].getVal();
                          #if (USE_MQTT > OFF)
                              sprintf(tmpMQTT, "%s%s", MQTT_DEVICE, PHOTO1_MQTT);
                            #endif
                          tmpStr = "SVA";
                          tmpStr.concat("3");
                          tmpStr.concat(tmpval16);
                          pmdServ->updateAll(tmpStr);
                        #endif
                      #if (USE_MQTT > OFF)
                          sprintf(tmpOut, "%d", tmpval16);
                      //    mqttClient.publish(tmpMQTT, 0, true, tmpOut, 6);
                          //SOUT(tmpOut); SOUT(" ");
                        #endif
                    #endif
                  outStr.concat("  ");
                  dispText(outStr, 12, 4, outStr.length());
                        SOUTLN(outStr); //SOUT(" ");
                  break;
                case 7:  // temp sensor
                  #if (USE_DS18B20_1W_IO > OFF)
                    outStr = "";
                    outStr = getDS18D20Str();
                    dispText(outStr ,  0, 4, outStr.length());
                    #endif
                  break;
                case 8:  // BME 280 temp, humidity, pressure
                  #if ( USE_BME280_I2C > OFF )
                    outStr = "";
                    for (uint8_t i = 0; i < 3 ; i++)
                      {
                        tmpStr = "SVA";
                        tmpStr.concat(i);
                        switch (i)
                          {
                            case 0:
                              tmpval16 = bme1T.getVal();
                              #if (USE_MQTT > OFF)
                                  sprintf(tmpMQTT, "%s%s", MQTT_DEVICE, BME2801T_MQTT);
                                #endif
                              tmpStr.concat(tmpval16);
                              outStr.concat(tmpval16);
                              outStr.concat("°  ");
                              break;
                            case 1:
                              tmpval16 = bme1P.getVal();
                              #if (USE_MQTT > OFF)
                                  sprintf(tmpMQTT, "%s%s", MQTT_DEVICE, BME2801P_MQTT);
                                #endif
                              tmpStr.concat(tmpval16);
                              outStr.concat(tmpval16);
                              outStr.concat("%  ");
                              break;
                            case 2:
                              tmpval16 = bme1H.getVal();
                              #if (USE_MQTT > OFF)
                                  sprintf(tmpMQTT, "%s%s", MQTT_DEVICE, BME2801H_MQTT);
                                #endif
                              tmpStr.concat(tmpval16);
                              outStr.concat(tmpval16);
                              outStr.concat("mb");
                              break;
                            default:
                              break;
                          }
                        // send to websocket
                        #if (USE_WEBSERVER > OFF)
                            pmdServ->updateAll(tmpStr);
                          #endif
                        #if (USE_MQTT > OFF)
                            sprintf(tmpOut, "%d", tmpval16);
                            heapFree("+publish");
                            //
                            //mqttClient.publish(tmpMQTT, 0, true, tmpOut, 6);
                                  //SOUT(tmpOut); SOUTLN(" ");
                            heapFree("+publish");
                            //sleep(2);
                          #endif
                      }
                    dispText(outStr , 0, 3, outStr.length());
                            //SOUT(outStr); SOUT(" ");
                    #endif
                	break;
                case 9:  // voltage, current
                    #if (USE_VCC_ANA > OFF)
                        tmpval16 = vcc[VCC50_IDX];
                        #if (USE_MQTT > OFF)
                            sprintf(tmpMQTT, "%s%s", MQTT_DEVICE, VCC50_MQTT);
                          #endif
                        outStr = "  V ";
                        outStr.concat(tmpval16);
                        outStr.concat("  ");
                        dispText(outStr, 1, 2, outStr.length());
                              //SOUT(outStr);
                              //SOUTLN(outStr);
                              //SOUT(outStr); SOUT(" ");
                        #if (USE_MQTT > OFF)
                            sprintf(tmpOut, "%d", tmpval16);
                        //    mqttClient.publish(tmpMQTT, 0, true, tmpOut, 6);
                                  //SOUT(tmpOut); SOUT(" ");
                          #endif
                      #endif
                    #if (USE_ACS712_ANA > OFF)
                        tmpval16 = i712[0];
                        #if (USE_MQTT > OFF)
                            sprintf(tmpMQTT, "%s%s", MQTT_DEVICE, I712_1_MQTT);
                          #endif
                        outStr = "  I ";
                        outStr.concat(i712[0]);
                        //outStr.concat(tmpval16);
                        outStr.concat("  ");
                        dispText(outStr, 15, 2, outStr.length());
                              //SOUT(outStr);
                              //SOUTLN(outStr);
                              //SOUT(outStr); SOUT(" ");
                        #if (USE_MQTT > OFF)
                            sprintf(tmpOut, "%d", tmpval16);
                        //    mqttClient.publish(tmpMQTT, 0, true, tmpOut, 6);
                                  //SOUT(tmpOut); SOUT(" ");
                          #endif
                      #endif
                   	break;
                case 10:  // poti,
                    #if (USE_POTI_ANA > OFF)
                        tmpval16 = potiScal[0].scale((float) poti[0]);
                        #if (USE_MQTT > OFF)
                            sprintf(tmpMQTT, "%s%s", MQTT_DEVICE, POTI1_MQTT);
                          #endif
                        outStr = "  P ";
                        outStr.concat(tmpval16);
                        outStr.concat("  ");
                        dispText(outStr, 15, 1, outStr.length());
                              //SOUTLN(outStr);
                              //SOUT(outStr); SOUT(" ");
                        #if (USE_MQTT > OFF)
                            sprintf(tmpOut, "%d", tmpval16);
                            //mqttClient.publish(tmpMQTT, 0, true, tmpOut, 6);
                          #endif
                      #endif
                    break;
                case 11:  // digital inputs
                    #if (USE_ESPHALL > OFF)
                        int32_t valHall = 0;
                      #endif
                    #if (USE_DIG_INP > OFF)
                      //SOUT(" SWD ");
                      outStr = "";
                      for (uint8_t i = 0 ; i < USE_GEN_SW_INP; i++)
                        {
                          //SOUT(i); SOUT("="); SOUT(valInpDig[i]); SOUT(" ");
                          tmpStr = "SWD";
                          tmpStr.concat(i);
                          tmpStr.concat(valInpDig[i]);
                          outStr.concat(valInpDig[i]);
                          outStr.concat(" ");
                          // send to websocket
                          #if (USE_WEBSERVER > OFF)
                              pmdServ->updateAll(tmpStr);
                            #endif
                          #if (USE_MQTT > OFF)
                            #endif
                        }
                      dispText(outStr , 17, 3, outStr.length());
                            //SOUTLN();
                    #endif
                  #if (USE_CTRL_POTI)
                      //SOUT("POT "); SOUT(inpValADC[INP_POTI_CTRL]); SOUT(" ");
                    #endif
                  break;
                case 12:  // WS2812 lines
                  #if ((USE_WS2812_MATRIX_OUT > OFF) || (USE_WS2812_LINE_OUT > OFF))
                      outStr = "              ";
                      dispText(outStr ,  0, 0, outStr.length());
                      //outStr = "LED ";
                      outStr = "";
                          //outStr += (String) ws2812_Mcnt; outStr += " ";
                      ws2812_Mv = millis() - ws2812_Malt; // dispT.getTout();
                      ws2812_Malt = millis();
                      if (ws2812_Mcnt > 0)
                        {
                          ws2812_Mv = ws2812_Mv / ws2812_Mcnt;
                          ws2812_Mcnt = 0;
                        }
                      outStr += (String) ws2812_Mv;
                      outStr += ("ms");
                            //SOUT((uint32_t) millis()); SOUT(" ");
                                //SOUT(outStr); SOUT(" ");
                      dispText(outStr ,  0, 0, outStr.length());
                    #endif
                  break;
                case 13:  // counter values
                  #if (USE_CNT_INP > OFF)
                        outStr = "              ";
                        dispText(outStr ,  0, 2, outStr.length());
                        outStr = "f";
                        SOUT(millis());
                        for (uint8_t i = 0; i < USE_CNT_INP ; i++ )
                          {
                            outStr = "   f";
                            outStr += (String) i;
                            outStr += " = ";
                            outStr += (String) cntErg[i].freq;
                            outStr += "     ";
                            SOUT(outStr); SOUT(" Hz"); SOUT("/"); SOUT(" usCnt "); SOUT(cntErg[i].usCnt); SOUT(" ");
                            dispText(outStr ,  0, i+1, outStr.length());
                          }
                        SOUTLN();
                        //SOUT(outStr); SOUT(" "); //SOUT(valFanPWM[0]); SOUT(" ");
                      #ifdef USE_MCPWM
                          outStr = "              ";
                          dispText(outStr ,  0, 0, outStr.length());
                          outStr = "LH ";
                          outStr += (String) cntLowPulse[0]; outStr += (" ");
                          outStr += (String) cntHighPulse[0];
                                    SOUT(outStr); SOUT(" ");
                          dispText(outStr ,  0, 2, outStr.length());
                        #endif
                    #endif
                  break;
                case 14: // pwm counter values
                  #if (USE_PWM_INP > OFF)
                        outStr = "              ";
                        dispText(outStr ,  0, 1, outStr.length());
                        outStr = "pwm ";
                        for (uint8_t i = 0; i < USE_PWM_INP ; i++ )
                          {
                            outStr += " ";
                            outStr += (String) pwmInVal[i].lowVal;
                            outStr += " ";
                            outStr += (String) pwmInVal[i].highVal;
                                  //SOUT("/"); SOUT(cntErg[i].pulsCnt); SOUT(" "); SOUT("/"); SOUT(cntErg[i].usCnt); SOUT(" ");
                          }
                        //SOUT(outStr); SOUT(" ");
                        //dispText(outStr ,  0, 1, outStr.length());
                    #endif
                  break;

                default:
                  dispIdx = 0;
                  if (SYS_LED_ON == ON) { SYS_LED_ON = OFF; }
                  else                  { SYS_LED_ON = ON ; }
                  digitalWrite(PIN_BOARD_LED, SYS_LED_ON);
                  break;
                }
              dispT.startT();
                      //heapFree("-dispIdx");
                      //SOUT(" "); SOUT(millis()); SOUTLN("  disp end ");

              #ifdef USE_STATUS
                  dispStatus("");
                #endif
                        //SOUTLN();
            }
          #endif // defined(DISP)
      // --- system control --------------------------------
        #if (USE_COL16_BLINK_OUT > 0)
          if (ledT.TOut())    // handle touch output
            {
              ledT.startT();
              if (SYS_LED_ON == TRUE)
                  {
                    digitalWrite(PIN_BOARD_LED, OFF);
                    SYS_LED_ON = OFF;
                  }
                else
                  {
                    digitalWrite(PIN_BOARD_LED, ON);
                    SYS_LED_ON = ON;
                  }
            }
          #endif
       if (firstrun == true)
          {
            String taskMessage = "loop task running on core ";
            taskMessage = taskMessage + xPortGetCoreID();
            SOUTLN(taskMessage);
            usLast = micros();
            firstrun = false;
          }
        anzUsCycles++;
          //usleep(20);
          //delay(1);
    }
// ----------------------------------------------------------------
// --- subroutine and drivers ----------------
// ----------------------------------------------------------------
  // --- system --------------------------
    // --- heap output
      void heapFree(const char* text)
        {
          uint32_t tmp32 = ESP.getFreeHeap();
          //uint32_t tmp32 = heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_32BIT);
          SOUT(" #"); SOUT(millis()); SOUT(" "); SOUT(text); SOUT(" "); SOUTLN(tmp32);
        }
  // --- user output ---------------------
    // --- display
      void clearDisp()
        {
          #if (USE_DISP > 0)
              #if (USE_OLED_I2C > OFF)
                  oled1.clear();
                  #if (USE_OLED_I2C > 1)
                      oled2.clear();
                    #endif
                #endif
            #endif
        }

      void dispStatus(String msg, bool direct)
        {
          #ifdef USE_STATUS
              size_t statLen = msg.length();
              bool   doIt    = false;
              bool   info    = false;

              if (statLen)
                {
                  if ( statLen > OLED1_MAXCOLS)
                    {
                      msg.remove(OLED1_MAXCOLS);
                    }
                  statOn = true;
                  statT.startT();
                  doIt = true;    // output statOut
                  statT.startT();
                }
              else // empty input
                {
                  if (statOn && statT.TOut())
                    statOn = false;
                }
              if (!statOn) // disp def val and actual time
                {
                  if (statN.TOut())
                    {
                      statN.startT();
                      #if (USE_NTP_SERVER > OFF)
                          sprintf(statOut,"%02d.%02d. %02d:%02d:%02d ", day(), month(), hour(), minute(), second());
                          msg = statOut;
                          msg.concat(" ");
                          msg.concat((unsigned long) usPerCycle);
                          msg.concat("us");
                          info = true;
                          doIt = true;
                        #endif
                    }
                }
              if (doIt)
                {
                  #if (USE_TOUCHSCREEN > OFF)
                      touch.wrStatus(msg);
                    #endif
                  #if (USE_OLED_I2C > OFF)
                      #if ( OLED1_STATUS > OFF )
                          oled1.wrStatus(msg);
                        #endif
                      #if ( USE_STATUS2 > OFF)
                          oled2.wrStatus(msg);
                        #endif
                           //SOUT("  md_error="); SOUTLN(md_error);
                    #endif
                  #if (USE_TFT > 0)
                      #if !(DISP_TFT ^ MC_UO_TFT1602_GPIO_RO)
                          mlcd.wrStatus((char*) statOut);
                        #endif
                              //#if !(DISP_TFT ^ MC_UO_TOUCHXPT2046_AZ)
                                   //if (info)
                                   //  {
                                   //    #if ( USE_BME280_I2C > OFF )
                                   //        outStr[0] = 0;
                                   //        outStr.concat(bmeT.getVal());
                                   //        outStr.concat("° ");
                                   //        outStr.concat(bmeH.getVal());
                                   //        outStr.concat("% ");
                                   //        outStr.concat(bmeP.getVal());
                                   //        outStr.concat("mb  ");
                                   //      #endif
                                   //  }
                                   // outStr.concat((char*) statOut);
                                   // if (info)
                                   // {
                                   //   #if (USE_WEBSERVER > OFF)
                                   //       outStr.concat(" ");
                                   //       outStr.concat(WiFi.localIP().toString());
                                   //     #endif
                                   // }
                                  // #endif
                          #if (DEBUG_MODE >= CFG_DEBUG_DETAILS)
                              SOUT("  md_error="); SOUTLN(md_error);
                            #endif
                    #endif // USE_DISP
                  info = false;
                }
            #endif // USE_STATUS
        }

      void dispStatus(const char* msg, bool direct)
        {
          dispStatus((String) msg);
        }

      void dispText(String msg, uint8_t col, uint8_t row, uint8_t len)
        {
          #if (USE_DISP > 0)
              #if (USE_OLED_I2C > OFF)
                  oled1.wrText(msg, col, row, len);
                            //SOUT((uint32_t) millis); SOUT(" dispText oled1 '"); SOUT(msg); SOUTLN("'");
                  #if (USE_OLED_I2C > 1)
                      oled2.wrText(msg, col, row, len);
                            //SOUT((uint32_t) millis); SOUT(" dispText oled2 '"); SOUT(msg); SOUTLN("'");
                    #endif
                #endif
              #if (USE_TFT > 0)
                  #if !(DISP_TFT ^ MC_UO_TFT1602_GPIO_RO)
                      mlcd.wrText(msg, row, col);
                    #endif
                  #if (USE_TOUCHSCREEN > OFF)
                      touch.wrText(msg, col, row, len);
                    #endif
                #endif
            #endif
        }

      void dispText(char* msg, uint8_t col, uint8_t row, uint8_t len)
        {
          dispText((String) msg, col, row, len);
            /*
                #if (USE_DISP > 0)
                    #if (USE_OLED1_I2C > OFF)
                        oled1.wrText(msg, col, row, len);
                            #if (DEBUG_MODE >= CFG_DEBUG_DETAILS)
                                SOUT("  md_error="); SOUTLN(md_error);
                              #endif
                      #endif
                    #if defined(OLED2)
                        oled2.wrText(msg, col, row, len);
                            #if (DEBUG_MODE >= CFG_DEBUG_DETAILS)
                              SOUT("  md_error="); SOUTLN(md_error);
                            #endif
                      #endif
                    #if (USE_TFT > 0)
                        #if !(DISP_TFT ^ MC_UO_TFT1602_GPIO_RO)
                            mlcd.wrText(msg, row, col);
                          #endif
                        #if !(DISP_TFT ^ MC_UO_TOUCHXPT2046_AZ)
                            touch.wrText(msg, col, row, len);
                          #endif
                      #endif
                    #if (USE_TOUCHSCREEN > OFF)
                      #endif
                  #endif
              */
        }

      void startDisp()
        {
          #if (USE_DISP > 0)
              #ifdef USE_STATUS
                statOut[OLED1_MAXCOLS] = 0;  // limit strlen
                #endif

              #if (USE_TFT > 0)
                  #if !(DISP_TFT ^ MC_UO_TFT1602_GPIO_RO)
                      mlcd.start(plcd);
                    #endif
                  #if (USE_TOUCHSCREEN_SPI > OFF)
                      touch.start(DISP_ORIENT, DISP_BCOL);
                          #if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                            SOUT(millis()); SOUTLN(" startTouch ");
                          #endif
                    #endif
                #endif

              #if (USE_OLED_I2C > OFF)
                  oled1.begin((uint8_t) OLED1_MAXCOLS, (uint8_t) OLED1_MAXROWS);
                  SOUT(millis()); SOUTLN(" oled1 gestartet");
                  #if (USE_OLED_I2C > 1)
                      oled2.begin((uint8_t) OLED2_MAXCOLS, (uint8_t) OLED2_MAXROWS);
                    #endif
                #endif
            #endif
        }
    // --- WS2812 LED
      #if (USE_WS2812_LINE_OUT > OFF)
          void initWS2812Line()
            {
              uint32_t i32tmp = 0;
              strip.begin();
              strip.setBrightness(255);
              strip.fill(strip.Color(200,0,0));
              strip.show();
              usleep(400000);
              i32tmp = COL24_2812_L1 + (BRI_2812_L1 << 24);
              SOUT(" strip BriCol org "); SOUTHEX(i32tmp);

              line2812[0] = new md_LEDPix24();
              line2812[1] = new md_LEDPix24(i32tmp);
                  //(BRI_2812_L1,
                  // (uint8_t) ((COL24_2812_L1 & 0x00ff0000) >> 16),
                  // (uint8_t) ((COL24_2812_L1 & 0x0000ff00) >> 8),
                  // (uint8_t) ( COL24_2812_L1 & 0x000000ff)
                  //);
                  //SOUT(" line[0]/[1] vor "); SOUTHEX((uint32_t)*line2812[0]); SOUT(" "); SOUTHEX((uint32_t)*line2812[1]);
              *line2812[0] = *line2812[1];
                  //SOUT(" line[0]/[1] nach "); SOUTHEX((uint32_t)*line2812[0]); SOUT(" "); SOUTHEXLN((uint32_t)*line2812[1]);
                  //SOUT(" line[0].bright() "); SOUTHEX(line2812[0]->bright());
                  //SOUT(" line[0].col24() "); SOUTHEXLN(line2812[0]->col24());
                  //strip.setBrightness(line2812[0]->bright());
              strip.setBrightness(5);
              strip.fill(line2812[0]->col24());
                  //strip.fill(strip.Color((uint8_t) ((line2812[0]->col24() & 0x00FF0000u) >> 16),
                  //                       (uint8_t) ((line2812[0]->col24() & 0x0000FF00u) >> 8 ),
                  //                       (uint8_t)  (line2812[0]->col24() & 0x000000FFu) ) );
              strip.show();
              SOUTLN("ok");
            }
        #endif

      #if (USE_WS2812_MATRIX_OUT > OFF)
          void initWS2812Matrix()
            {
              md_LEDPix24* ppix = outM2812[1].bmpB->pix24;
                //SOUT(" p_outM 0/1 "); SOUTHEX((uint32_t) &outM2812[0]);            SOUT(" / "); SOUTHEXLN ((uint32_t) &outM2812[1]);
                //SOUT(" p_text 0/1 "); SOUTHEX((uint32_t) &outM2812[0].text);       SOUT(" / "); SOUTHEXLN ((uint32_t) outM2812[1].text);
                //SOUT(" p_pix24 0/1 "); SOUTHEX((uint32_t) outM2812[0].text->pix24); SOUT(" / "); SOUTHEXLN ((uint32_t) outM2812[1].bmpB->pix24);
              ppix->col16(COL24_2812_BM1);
              ppix->bright(BRI_2812_BM1);
              outM2812[1].bmpB->bmp_num = MD_BITMAP_SMILY;
              outM2812[0].bmpB = outM2812[1].bmpB;

              ppix = outM2812[1].text->pix24;
              ppix->col16(COL24_2812_M1);
              ppix->bright(BRI_2812_M1);
              outM2812[1].text->text = (char*) text2812;
              outM2812[0].text = outM2812[1].text;

              ppix = outM2812[1].bmpE->pix24;
              ppix->col16(COL24_2812_BM1);
              ppix->bright(BRI_2812_BM1);
              outM2812[1].bmpE->bmp_num = MD_BITMAP_SMILY;
              outM2812[0].bmpE = outM2812[1].bmpE;
                //memcpy((char*) &outM2812[0], (char*) &outM2812[1], sizeof(outM2812[1]));
                //SOUT(" outM bmpB 0/1 ");
                //SOUTHEX(outM2812[0].bmpB->bmp_num); SOUT(" / "); SOUTHEX(outM2812[1].bmpB->bmp_num); SOUT(" - ");
                //SOUTHEX((uint32_t) outM2812[0].bmpB->pix24);   SOUT(" / "); SOUTHEXLN ((uint32_t) outM2812[1].bmpB->pix24);
                //SOUTHEX(*outM2812[0].bmpB->pix24);  SOUT(" / "); SOUTHEXLN(*outM2812[1].bmpB->pix24);
                //SOUT(" outM text 0/1 ");
                //SOUT(outM2812[0].text->text);       SOUT(" / "); SOUT(outM2812[1].text->text); SOUT(" - ");
                //SOUTHEX((uint32_t) outM2812[0].text->pix24);   SOUT(" / "); SOUTHEXLN((uint32_t) outM2812[1].text->pix24);
                //SOUTHEX(*outM2812[0].text->pix24);  SOUT(" / "); SOUTHEXLN(*outM2812[1].text->pix24);
                //SOUT(" outM bmpE 0/1 ");
                //SOUTHEX(outM2812[0].bmpE->bmp_num); SOUT(" / "); SOUTHEX(outM2812[1].bmpE->bmp_num); SOUT(" - ");
                //SOUTHEX(*outM2812[0].bmpE->pix24);  SOUT(" / "); SOUTHEXLN(*outM2812[1].bmpE->pix24);

                //SOUT(" p_outM 0/1 "); SOUTHEX((uint32_t) &outM2812[0]);            SOUT(" / "); SOUTHEXLN ((uint32_t) &outM2812[1]);
                //SOUT(" p_text 0/1 "); SOUTHEX((uint32_t) &outM2812[0].text);       SOUT(" / "); SOUTHEXLN ((uint32_t) outM2812[1].text);
                //SOUT(" p_pix24 0/1 "); SOUTHEX((uint32_t) outM2812[0].text->pix24); SOUT(" / "); SOUTHEXLN ((uint32_t) outM2812[1].bmpB->pix24);

              usleep(10000);
                //outM2812[0].text = outM2812[1].text;
                //outM2812[0].bmpE = outM2812[1].bmpE;
              matrix_1.begin();
              usleep(50000);
              matrix_1.display_boxes();
              usleep(1500000);
                //matrix_1.start_scroll_task((scroll2812_t*) &outM2812, &posM2812);

              SOUT(" start = "); SOUT(posM2812);
              int16_t tmp = (strlen(text2812) * (uint8_t) COLCHAR_2812 + (uint8_t) OFFEND_2812_M1);
              matrix_1.start_scroll_matrix(  (scroll2812_t*) outM2812, &posM2812, - tmp);
            }
        #endif
    // --- passive buzzer
      #ifdef PLAY_MUSIC
          void playSong(int8_t songIdx)
            {
              if (buzz.setSong(SONG0_LEN,(void*) SONG0_NOTES) == ISOK)
                {
                  #ifdef USE_SONGTASK
                    buzz.playSong();
                  #endif
                }
            }

          void playSong()
            { playSong(0); }

        #endif

    // --- frequency generator
      #if (_USE_OUT_FREQ_PWM > OFF)
          void init_oscillator ()                                              // Inicializa gerador de pulsos
            {
              resolucao = (log (80000000 / oscilador)  / log(2)) / 2 ;                // Calculo da resolucao para o oscilador
              if (resolucao < 1) resolucao = 1;                                       // Resolu�ao m�nima
              // Serial.println(resolucao);                                           // Print
              mDuty = (pow(2, resolucao)) / 2;                                        // Calculo do ciclo de carga 50% do pulso
              // Serial.println(mDuty);                                               // Print

              ledc_timer_config_t ledc_timer = {};                                    // Instancia a configuracao do timer do LEDC

              ledc_timer.duty_resolution =  ledc_timer_bit_t(resolucao);              // Configura resolucao
              ledc_timer.freq_hz    = oscilador;                                      // Configura a frequencia do oscilador
              ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;                           // Modo de operacao em alta velocidade
              ledc_timer.timer_num = LEDC_TIMER_0;                                    // Usar timer0 do LEDC
              ledc_timer_config(&ledc_timer);                                         // Configura o timer do LEDC

              ledc_channel_config_t ledc_channel = {};                                // Instancia a configuracao canal do LEDC

              ledc_channel.channel    = LEDC_CHANNEL_0;                               // Configura canal 0
              ledc_channel.duty       = mDuty;                                        // Configura o ciclo de carga
              ledc_channel.gpio_num   = LEDC_HS_CH0_GPIO;                             // Configura GPIO da saida do LEDC - oscilador
              ledc_channel.intr_type  = LEDC_INTR_DISABLE;                            // Desabilita interrup��o do LEDC
              ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;                         // Modo de operacao do canal em alta velocidade
              ledc_channel.timer_sel  = LEDC_TIMER_0;                                 // Seleciona timer 0 do LEDC
              ledc_channel_config(&ledc_channel);                                     // Configura o canal do LEDC
            }
        #endif
  // --- user input ----------------------
    // --- keypad
      #if defined(KEYS)
          void startKeys()
            {
              #if (USE_KEYPADSHIELD > OFF)
                  kpad.init(KEYS_ADC);
                #endif // USE_KEYPADSHIELD
            }

          uint8_t getKey()
            {
              #if (USE_KEYPADSHIELD > OFF)
                  return kpad.getKey();
                #else
                  return NOKEY;
                #endif // USE_KEYPADSHIELD
            }
        #endif

    // --- counter
      #if (USE_CNT_INP > OFF)
          static void initGenPCNT()
            {
              /* Initialize PCNT event queue and PCNT functions */
              pcnt_unit_t unit;
              esp_err_t   retESP;
              SOUT("init pcnt ");
              for (uint8_t i = 0 ; i < USE_CNT_INP ; i++)
                {
                  sprintf(cmsg, "pcnt_unit_config[%1i]",i);
                  // Initialize PCNT unit and  queue
                  cntFilt[i] = 0;
                  //retESP = pcnt_unit_config(&config_cnt[i]);
                  logESP(pcnt_unit_config(&config_cnt[i]), cmsg, i);
                  pcnt_evt_queue[i] = xQueueCreate(1, sizeof(pcnt_evt_t));
                  switch (i)
                    {
                      case 0:
                        unit       = (pcnt_unit_t) PCNT0_UNIT;
                        cntThresh[i]  = (uint16_t)    PCNT0_THRESH0_VAL;
                        cntFakt[i] = (float)    PCNT0_CNT_FAKT;
                        // Configure and enable the input filter
                          pcnt_set_filter_value(unit, PCNT0_INP_FILT);
                          pcnt_filter_enable(unit);
                        // Set threshold 0 and 1 values and enable events to watch //
                          pcnt_set_event_value(unit, PCNT0_EVT_0, PCNT0_THRESH0_VAL);
                          pcnt_event_enable(unit, PCNT0_EVT_0);
                              //pcnt_set_event_value(unit, PCNT_EVT_THRES_1, PCNT1_THRESH0_VAL);
                              //pcnt_event_enable(unit, PCNT_EVT_THRES_1);
                            // Enable events on zero, maximum and minimum limit values //
                              //pcnt_event_enable(unit, PCNT1_EVT_ZERO);
                              //pcnt_event_enable(unit, PCNT1_EVT_H_LIM);
                              //pcnt_event_enable(unit, PCNT1_EVT_L_LIM);
                        break;
                      #if (USE_CNT_INP > 1)
                          case 1:
                            unit = (pcnt_unit_t) PCNT1_UNIT;
                            cntThresh[i] = (uint16_t) PCNT1_THRESH0_VAL;
                            // Configure and enable the input filter
                              pcnt_set_filter_value(unit, PCNT1_INP_FILT);
                              pcnt_filter_enable(unit);
                            // Set threshold 0 and 1 values and enable events to watch //
                              pcnt_set_event_value(unit, PCNT1_EVT_0, PCNT1_THRESH0_VAL);
                              pcnt_event_enable(unit, PCNT1_EVT_0);
                            break;
                        #endif
                      #if (USE_CNT_INP > 2)
                          case 2:
                            unit = (pcnt_unit_t) PCNT2_UNIT;
                            // Configure and enable the input filter
                              pcnt_set_filter_value(unit, PCNT2_INP_FILT);
                              pcnt_filter_enable(unit);
                            // Set threshold 0 and 1 values and enable events to watch //
                              pcnt_set_event_value(unit, PCNT_EVT_0, PCNT2_THRESH0_VAL);
                              pcnt_event_enable(unit, PCNT_EVT_0);
                            break;
                        #endif
                      #if (USE_CNT_INP > 3)
                          case 1:
                            unit = (pcnt_unit_t) PCNT3_UNIT;
                            // Configure and enable the input filter
                              pcnt_set_filter_value(unit, PCNT3_INP_FILT);
                              pcnt_filter_enable(unit);
                            // Set threshold 0 and 1 values and enable events to watch //
                              pcnt_set_event_value(unit, PCNT3_EVT_0, PCNT3_THRESH0_VAL);
                              pcnt_event_enable(unit, PCNT3_EVT_0);
                            break;
                        #endif
                      default:
                        break;
                    }
                  // Initialize PCNT's counter //
                  pcnt_counter_pause(unit);
                  pcnt_counter_clear(unit);

                  // Install interrupt service and add isr callback handler //
                  sprintf(cmsg, "_unit %i pcnt_isr_service_install_ ", i);
                  logESP(pcnt_isr_service_install(unit), cmsg, i);
                  switch (i)
                    {
                      case 0:
                        SOUTLN("  pcnt_isr_handler unit 0");
                        sprintf(cmsg, "_unit %i pcnt_isr_handler_add_ ", i);
                        logESP(pcnt_isr_handler_add(unit, pcnt0_intr_hdl, &unit), cmsg, i);
                        break;
                      #if (USE_CNT_INP > 1)
                          case 1:
                            SOUTLN("  pcnt_isr_handler unit 1");
                            sprintf(cmsg, "_unit %i pcnt_isr_handler_add_ ", i);
                            logESP(pcnt_isr_handler_add(unit, pcnt1_intr_hdl, &unit), cmsg, i);
                            break;
                        #endif
                      #if (USE_CNT_INP > 2)
                          case 2:
                            pcnt_isr_handler_add(unit, pcnt2_intr_hdl, &unit);
                            break;
                        #endif
                      #if (USE_CNT_INP > 3)
                          case 3:
                            pcnt_isr_handler_add(unit, pcnt3_intr_hdl, &unit);
                            break;
                        #endif
                      default:
                        break;
                    }
                  // Everything is set up, now go to counting //
                  pcnt_counter_resume(unit);
                }
            }
          #ifdef USE_MCPWM
              static void getCNTIn()
                {
                      // count LOW width
                      mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, 0);
                      cntLowPulse[0] = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0) / 1000;
                      // count HIGH width
                      mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);
                      cntHighPulse[0] = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0) /1000 ;
                }
            #endif
        #endif
    // --- digital input
      #if (USE_DIG_INP > OFF)
          void getDIGIn()
            {
              #if USE_WS2812_PWR_IN_SW
                  ws2812_pwr = digitalRead(PIN_WS2812_PWR_IN_SW);
                #endif
              #if (USE_GEN_SW_INP > OFF)
                  uint8_t tmp;
                  for (uint8_t i = 0; i < USE_GEN_SW_INP ; i++)
                    {
                      tmp = digitalRead(pinInpDig[i]);
                      valInpDig[i] = tmp;
                      if (!polInpDig[i])
                        if (tmp == valInpDig[i])
                          if (tmp == digitalRead(pinInpDig[i]))
                            valInpDig[i] = !tmp;
                      else
                        if (tmp != valInpDig[i])
                          if (tmp == digitalRead(pinInpDig[i]))
                            valInpDig[i] = tmp;
                    }
                #endif
            }
        #endif
    // --- analog input
      #if (USE_CTRL_POTI > OFF)
          void getADCIn()
            {
              for (uint8_t i = 0 ; i < USE_CTRL_SW_INP ; i++ )
                {
                  inpValADC[i] = analogRead(PIN_ADC_CONF[i].pin);
                }
            }
        #endif

      #if (USE_ADC1115_I2C > OFF)
          static void init1115_chan(uint8_t unit, uint8_t chan, uint8_t mode, uint8_t att)
            {

            }
        #endif

  // --- sensors -------------------------
    // --- DS18B20
      String getDS18D20Str()
        {
          String outS = "";
          #if (USE_DS18B20_1W_IO > OFF)
              dsSensors.requestTemperatures(); // Send the command to get temperatures
              for (uint8_t i = 0 ; i < DS18B20_ANZ ; i++ )
                {
                  dsTemp[i] = dsSensors.getTempCByIndex(i);
                        //SOUTLN(dsTemp[i]);
                  if (i < 1) { outS  = "T1 "; }
                  else       { outS += "    T2  ";}
                  outS += (String) ((int) (dsTemp[i] + 0.5)) + "°";
                }
            #endif
          return outS;
        }
    // --- BME280
    // --- T-element type K
    // --- photo sensor
  // --- memory --------------------------
    void testFlash()
      {
        SOUTLN("mounting SPIFFS ... ");
        if(!SPIFFS.begin(true))
          {
           SOUTLN("ERROR");
            return;
          }

        uint32_t bFree = SPIFFS.totalBytes();
        SOUT("found "); SOUT(bFree); SOUTLN(" bytes free");
        //*
        SOUT("dir: test_example.txt ");
        File file = SPIFFS.open("/test_example.txt");
        if(!file)
          {
            SOUTLN("Failed to open file for reading");
            return;
          }

        SOUT("File Content: ");
        int8_t n = 0;
        while(n >= 0)
          {
            n = file.read();
            if (n > 0) SOUT((char) n);
          }
        SOUTLN();
        file.close();
        //*/
      }

  // --- network -------------------------
    // --- WIFI
      bool startWIFI(bool startup)
        {
          #if (USE_WIFI > OFF)
              bool ret = ISERR;
              dispStatus("  start WIFI");
              if (startup)
                {
                  ip_list ipList = ip_list(); // temporary object
                            #if (DEBUG_MODE > CFG_DEBUG_STARTUP)
                                SOUT(millis()); SOUT(" setup startWIFI created ipList "); SOUTHEXLN((int) &ipList);
                                SOUT(millis()); SOUTLN(" setup startWIFI add WIFI 0");
                              #endif
                  ipList.append(WIFI_FIXIP0, WIFI_GATEWAY0, WIFI_SUBNET, WIFI_SSID0, WIFI_SSID0_PW);
                  #if (WIFI_ANZ_LOGIN > 1)
                            #if (DEBUG_MODE > CFG_DEBUG_STARTUP)
                                SOUT(millis()); SOUTLN(" setup startWIFI add WIFI 1");
                              #endif
                      ipList.append(WIFI_FIXIP1, WIFI_GATEWAY1, WIFI_SUBNET, WIFI_SSID1, WIFI_SSID1_PW);
                    #endif
                  #if (WIFI_ANZ_LOGIN > 2)
                            #if (DEBUG_MODE > CFG_DEBUG_STARTUP)
                                SOUT(millis()); SOUTLN(" setup startWIFI add WIFI 2");
                              #endif
                      ipList.append(WIFI_FIXIP2, WIFI_GATEWAY2, WIFI_SUBNET, WIFI_SSID2, WIFI_SSID2_PW);
                    #endif
                  #if (WIFI_ANZ_LOGIN > 3)
                            #if (DEBUG_MODE > CFG_DEBUG_STARTUP)
                                SOUT(millis()); SOUTLN(" setup add WIFI 3");
                              #endif
                      ipList.append(WIFI_FIXIP3, WIFI_GATEWAY3, WIFI_SUBNET, WIFI_SSID3, WIFI_SSID3_PW);
                    #endif
                  #if (WIFI_ANZ_LOGIN > 4)
                            #if (DEBUG_MODE > CFG_DEBUG_STARTUP)
                                SOUT(millis()); SOUTLN(" setup add WIFI 4");
                              #endif
                      ipList.append(WIFI_FIXIP4, WIFI_GATEWAY4, WIFI_SUBNET, WIFI_SSID4, WIFI_SSID4_PW);
                    #endif
                  #if (WIFI_ANZ_LOGIN > 5)
                            #if (DEBUG_MODE > CFG_DEBUG_STARTUP)
                                SOUT(millis()); SOUTLN(" setup add WIFI 5");
                              #endif
                      ipList.append(WIFI_FIXIP5, WIFI_GATEWAY5, WIFI_SUBNET, WIFI_SSID5, WIFI_SSID5_PW);
                    #endif
                  #if (WIFI_ANZ_LOGIN > 6)
                            #if (DEBUG_MODE > CFG_DEBUG_STARTUP)
                                SOUT(millis()); SOUTLN(" setup add WIFI 6");
                              #endif
                      ipList.append(WIFI_FIXIP6, WIFI_GATEWAY6, WIFI_SUBNET, WIFI_SSID6, WIFI_SSID6_PW);
                    #endif
                  #if (WIFI_ANZ_LOGIN > 7)
                            #if (DEBUG_MODE > CFG_DEBUG_STARTUP)
                                SOUT(millis()); SOUTLN(" setup add WIFI 7");
                              #endif
                      ipList.append(WIFI_FIXIP7, WIFI_GATEWAY7, WIFI_SUBNET, WIFI_SSID7, WIFI_SSID7_PW);
                    #endif
                  #if (WIFI_ANZ_LOGIN > 8)
                            #if (DEBUG_MODE > CFG_DEBUG_STARTUP)
                                SOUT(millis()); SOUTLN(" setup add WIFI 8");
                              #endif
                      ipList.append(WIFI_FIXIP8, WIFI_GATEWAY8, WIFI_SUBNET, WIFI_SSID8, WIFI_SSID8_PW);
                    #endif
                            SOUT(millis()); SOUTLN(" setup startWIFI locWIFI fertig");

                            //ip_cell* pip = (ip_cell*) ipList.pFirst();
                            //char stmp[NET_MAX_SSID_LEN] = "";
                                    /*
                                      SOUT(" setup ip_list addr "); SOUT((u_long) &ipList);
                                      SOUT(" count "); SOUTLN(ipList.count());
                                      SOUT(" ip1: addr "); SOUTHEX((u_long) pip);
                                      SOUT(" locIP "); SOUTHEX(pip->locIP());
                                      SOUT(" gwIP ");  SOUTHEX(pip->gwIP());
                                      SOUT(" snIP ");  SOUTHEX(pip->snIP());
                                      pip->getSSID(stmp); SOUT(" ssid "); SOUT(stmp);
                                      pip->getPW(stmp); SOUT(" pw "); SOUTLN(stmp);
                                      pip = (ip_cell*) pip->pNext();
                                      SOUT(" ip2: addr "); SOUTHEX((u_long) pip);
                                      SOUT(" locIP "); SOUTHEX(pip->locIP());
                                      SOUT(" gwIP ");  SOUTHEX(pip->gwIP());
                                      SOUT(" snIP ");  SOUTHEX(pip->snIP());
                                      pip->getSSID(stmp); SOUT(" ssid "); SOUT(stmp);
                                      pip->getPW(stmp); SOUT(" pw "); SOUTLN(stmp);
                                      pip = (ip_cell*) pip->pNext();
                                      SOUT(" ip3: addr "); SOUTHEX((u_long) pip);
                                      SOUT(" locIP "); SOUTHEX(pip->locIP());
                                      SOUT(" gwIP ");  SOUTHEX(pip->gwIP());
                                      SOUT(" snIP ");  SOUTHEX(pip->snIP());
                                      pip->getSSID(stmp); SOUT(" ssid "); SOUT(stmp);
                                      pip->getPW(stmp); SOUT(" pw "); SOUTLN(stmp);
                                    */

                  ret = wifi.scanWIFI(&ipList);
                            SOUT(millis()); SOUT(" scanWIFI ret="); SOUTLN(ret);
                  if (&ipList != NULL)
                    {
                      ipList.~ip_list();
                    }
                }
              ret = wifi.startWIFI();
                          SOUT(" startWIFI ret="); SOUT(ret);
              //md_error = setBit(md_error, ERRBIT_WIFI, ret);
                    //#if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                      //SOUT("  md_error="); SOUTLN(md_error);
                      //#endif

                    //if ((md_error & ERRBIT_WIFI) == 0)
              //if (ret == ISOK)
                  //dispStatus("WIFI connected");
                //else
                  //dispStatus("WIFI error");

              #if (USE_NTP_SERVER > OFF)
                  if((md_error & ERRBIT_WIFI) == 0) // WiFi ok
                    if((md_error & ERRBIT_NTPTIME) != 0) // WiFi ok
                      wifi.initNTP();
                #endif
            #endif // USE_WIFI
            return ret;
        }

    // --- NTP server
      void initNTPTime()
        {
          #if (USE_NTP_SERVER > OFF)
              bool ret = wifi.initNTP();
                    #if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                      Serial.print("initNTPTime ret="); Serial.print(ret);
                    #endif
              md_error = setBit(md_error, ERRBIT_NTPTIME, ret);
                    #if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                      Serial.print("  md_error="); Serial.println(md_error);
                    #endif
              if ((md_error & ERRBIT_WIFI) == OK)
                {
                  dispStatus("NTPTime ok");
                }
                else
                {
                  dispStatus("NTPTime error");
                }
            #endif // USE_NTP_SERVER
        }

    // --- webserver
      #if (USE_WEBSERVER > OFF)
        void startWebServer()
          {
            bool ret = ISERR;
            //if ((md_error & ERRBIT_SERVER) != 0)
            if (getBit(md_error, ERRBIT_SERVER))
              {
                dispStatus("start webserver");
                SOUT("startServer ... ");
                //if ((md_error & ERRBIT_WIFI) == 0)
                if (!getBit(md_error, ERRBIT_WIFI))
                  {
                    ret = pmdServ->md_startServer();
                        #if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                            SOUT("ret="); SOUTLN(ret);
                          #endif
                  }
                md_error = setBit(md_error, ERRBIT_SERVER, ret);
                      #if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                        // SOUT("  md_error="); SOUTLN(md_error);
                      #endif

                //if ((md_error & ERRBIT_SERVER) == 0)
                if (!getBit(md_error, ERRBIT_SERVER))
                  {
                    dispStatus("Webserver online");
                    SOUTLN("Webserver online");
                  }
                  else
                  {
                    dispStatus("Webserver ERROR");
                    SOUTLN("Webserver ERROR");
                  }
              }
          }

        void readMessage()
          {
            md_message *pM   = NULL;
            int         itmp = 0;
            char*       ctmp = NULL;
            #if (USE_WS2812_MATRIX_OUT > OFF)
                md_LEDPix24* ppix = NULL;
              #endif
            uint8_t     idx;
            char        tval;
            char        tdata;

            if (inMsgs->count() > 0)
              {
                pM   = (md_message*) inMsgs->pFirst();
                switch (pM->msgType())
                  {
                    case ME_TSOCKET:
                      SOUT(" Socket ");
                      break;

                    case ME_TREQ:
                      pmdServ->isRequest = true;
                      break;

                    case ME_TCONN:
                      newClient = true;
                      break;

                    default:
                      ctmp  = pM->payload();
                      tdata = pM->dataType();
                      tval  = ctmp[0];
                      idx   = ctmp[1] - '0';
                      ctmp += 2;
                          SOUT(" msg client "); SOUT(pM->client());
                          SOUT(" tMsg ");       SOUT(pM->msgType());
                          SOUT(" tData ");      SOUT((char) pM->dataType());
                          SOUT(" tval ");       SOUT(tval);
                          SOUT(" payload ");    SOUT(pM->payload());
                          SOUT(" idx ");        SOUTLN(idx);
                          SOUT(" ctmp '");      SOUT(ctmp); SOUTLN("' ");
                      if (tdata == MD_SINGLE)
                        {
                          SOUTLN("---- switch(tval) ----");
                          switch (tval)
                            {
                              case EL_TANALOG:
                                  SOUT(" Analog nu ");
                                break;
                              case EL_TSLIDER:
                                  SOUTLN(" ---- Slider ----");
                                  itmp  = (int)strtol(ctmp, NULL, 10);
                                  switch (idx) // index of serverelement
                                    {
                                      case 1: // RGB-LED col24
                                        #if (USE_RGBLED_PWM > OFF)
                                            SOUTLN("  ---- RGBLED bright ----"); SOUT(RGBLED[1]->bright()); SOUT(" neu ");
                                            RGBLED[1]->bright(itmp);
                                            LEDout = TRUE;
                                            SOUT(RGBLED[1]->bright()); SOUT(" ");
                                          #endif
                                        break;
                                      case 2: // 2821 line col24
                                        #if (USE_WS2812_LINE_OUT >OFF)
                                            SOUTLN("  ---- line bright ----"); SOUT(line2812[1]->bright()); SOUT(" neu ");
                                            line2812[1]->bright((uint8_t) itmp);
                                            SOUT(line2812[1]->bright()); SOUT(" ");
                                          #endif
                                        break;
                                      case 3:
                                        #if (USE_WS2812_MATRIX_OUT > OFF)
                                            ppix = outM2812[1].text->pix24;
                                            SOUTLN("  ---- matrix bright ----"); SOUT(ppix->bright()); SOUT(" neu ");
                                            ppix->bright((uint8_t) itmp);
                                            SOUTLN(ppix->bright());
                                          #endif
                                        break;
                                      case 4:
                                        #if (USE_WS2812_MATRIX_OUT > OFF)
                                            ppix = outM2812[1].bmpB->pix24;
                                            SOUTLN("  ---- smilyB bright ----"); SOUT(ppix->bright()); SOUT(" neu ");
                                            ppix->bright((uint8_t) itmp);
                                            SOUTLN(ppix->bright());
                                            ppix = outM2812[1].bmpE->pix24;
                                            SOUTLN("  ---- smilyE bright ----"); SOUT(ppix->bright()); SOUT(" neu ");
                                            ppix->bright((uint8_t) itmp);
                                          #endif
                                        break;
                                    }
                                break;
                              case EL_TCOLOR:
                                  SOUTLN(" ---- Color ----");
                                  itmp  = (int)strtol(ctmp, NULL, 16);
                                  switch (idx) // index of serverelement
                                    {
                                      case 1: // RGB-LED col24
                                        #if (USE_RGBLED_PWM > OFF)
                                            SOUTLN("  ---- RGBLED color ----"); SOUTHEX(RGBLED[1]->col24()); SOUT(" neu ");
                                            RGBLED[1]->col24(itmp);
                                            LEDout = TRUE;
                                            SOUTHEXLN(RGBLED[1]->col24());
                                          #endif
                                        break;
                                      case 2: // 2821 line col24
                                        #if (USE_WS2812_LINE_OUT >OFF)
                                            SOUTLN(" ---- line2812 color24 ----"); SOUTHEX(line2812[1]->col24());
                                            line2812[1]->col24(itmp);
                                            SOUT(" neu 24/16"); SOUTHEX(line2812[1]->col24()); SOUT(" / "); SOUTLN(Col16(line2812[1]->col24()));
                                          #endif
                                        break;
                                      case 3: // 2821 matrix col16
                                        #if (USE_WS2812_MATRIX_OUT > OFF)
                                            ppix = outM2812[1].text->pix24;
                                            SOUTLN(" ---- matrix color24 ----"); SOUTHEX(ppix->col24());
                                            ppix->col24(itmp);
                                            SOUT(" neu 24/16 "); SOUTHEX(ppix->col24()); SOUT(" / "); SOUTHEX(Col16(ppix->col24())); SOUT(" ");
                                          #endif
                                        break;
                                      case 4: // 2821 matrix col16 bmp
                                        #if (USE_WS2812_MATRIX_OUT > OFF)
                                            ppix = outM2812[1].bmpB->pix24;
                                            SOUTLN(" ---- matrix color24 ----"); SOUTHEX(ppix->col24());
                                            ppix->col24(itmp);
                                            SOUT(" neu 24/16 "); SOUTHEX(ppix->col24()); SOUT(" / "); SOUTHEX(Col16(ppix->col24())); SOUT(" ");
                                            ppix = outM2812[1].bmpE->pix24;
                                            SOUTLN(" ---- matrix color24 ----"); SOUTHEX(ppix->col24());
                                            ppix->col24(itmp);
                                            SOUT(" neu 24/16 "); SOUTHEX(ppix->col24()); SOUT(" / "); SOUTHEX(Col16(ppix->col24())); SOUT(" ");
                                          #endif
                                        break;
                                    }
                                break;
                              case EL_TSWITCH:
                                  SOUT(" Switch ");
                                break;
                              case EL_TTEXT:
                                  SOUT(" Text ");
                                break;
                              case EL_TOFFSET:
                                  SOUT(" Offset ");
                                break;
                              case EL_TGRAPH:
                                  SOUT(" Graph ");
                                break;
                              case EL_TINDEX:
                                  SOUT(" Index ");
                                break;
                              default:
                                  SOUT(" ERROR ");
                                break;
                            }
                        }
                      break;
                  }
                SOUT("'"); SOUT(pM->payload()); SOUT("'");
                inMsgs->rem();
                SOUT(" inMsgs.count "); SOUTLN(inMsgs->count());
              }
          }
          /*
            void    handleClient(AsyncWebSocketClient *client, void *arg, uint8_t *data, size_t len)
              {
                AwsFrameInfo *info = (AwsFrameInfo*)arg;
                char* txt = (char*) data;
                if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
                  { //  SOUT(" handleWebSocketMessage info->index "); SOUT(info->index); SOUT(" info->final "); SOUT(info->final); SOUT(" info->len "); SOUTLN(info->len);
                    data[len] = 0;
                    uint8_t type  = txt[0];  // extract obj type
                    uint8_t index = txt[1] - WS_IDX_OFFSET;  // extract index
                    int16_t value = atoi(&txt[2]);
                              //SOUT(" Payload type "); SOUT(type);
                              //SOUT(" index "); SOUT(index); SOUT(" len "); SOUT(len);
                              //SOUT(" data '"); SOUT(&txt[2]); SOUT(" = "); SOUT(value);
                              //SOUT(" ledList cnt "); SOUTLN(psliderList->count());

                    if (type == EL_TSLIDER)
                      {
                        md_slider* psl = (md_slider*) psliderList->pIndex(index);
                              //SOUT(" psl "); SOUTHEX((uint32_t) psl);
                        if (psl != NULL)
                          {
                            psl->destVal = value;
                            SOUT(" slider "); SOUT((index+1)); SOUT("="); SOUTLN(value);
                          }
                      }

                    else if (type == EL_TSWITCH)
                      {
                        md_switch* psw = (md_switch*) pswitchList->pIndex(index);
                        while (psw != NULL)
                          {
                            psw->destVal = value; SOUT(" switch "); SOUTLN(value);
                          }
                      }

                    else if (type == EL_TANALOG)
                      {
                        md_analog* pana = (md_analog*) panalogList->pIndex(index);
                        while (pana != NULL)
                          {
                            pana->destVal = value; SOUT(" analog "); SOUTLN(value);
                          }
                      }

                    else { }
                  }
              }
            */
          /*
            void handlingIncomingData()
              {


                AwsFrameInfo *info = (AwsFrameInfo*)arg;

                if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
                  {
                    String hexColor = "";
                    for (int i=0; i < len; i++)
                      hexColor += ((char) data[i]);

                    Serial.println("Hex Color: " + hexColor);

                    long n = strtol(&hexColor[0], NULL, 16);
                    Serial.println(n);
                    strip.fill(n);
                    strip.show();
                  }
              }
            */
            // Callback for incoming event
          /*
            void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type,
                           void * arg, uint8_t *data, size_t len)
              {
                switch(type)
                  {
                    case WS_EVT_CONNECT:
                      Serial.printf("Client connected: \n\tClient id:%u\n\tClient IP:%s\n",
                           client->id(), client->remoteIP().toString().c_str());
                      break;
                    case WS_EVT_DISCONNECT:
                      Serial.printf("Client disconnected:\n\tClient id:%u\n", client->id());
                      break;
                    case WS_EVT_DATA:
                      handlingIncomingData(client, arg, data, len);
                      break;
                    case WS_EVT_PONG:
                      Serial.printf("Pong:\n\tClient id:%u\n", client->id());
                      break;
                    case WS_EVT_ERROR:
                      Serial.printf("Error:\n\tClient id:%u\n", client->id());
                      break;
                  }
              }
            void configWebsite()
              {
                webMD.createElement(EL_TSLIDER, "LED red", "%");
                webMD.createElement(EL_TSLIDER, "LED green", "%");
                webMD.createElement(EL_TSLIDER, "LED blue", "%");

                webMD.createElement(EL_TANALOG, "DS18B20 Temp", "°C");
                webMD.createElement(EL_TANALOG, "Type-K Temp", "°C");
                webMD.createElement(EL_TANALOG, "BME_Temp", "°C");
                webMD.createElement(EL_TANALOG, "BME_Humidity", "%");
                webMD.createElement(EL_TANALOG, "BME_Pressure", "mb");
                webMD.createElement(EL_TANALOG, "Gaswert", "");
              }
            */

        #endif // USE_WEBSERVER
    // --- MQTT
      #if (USE_MQTT > OFF)
#ifdef UNUSED
        void connectToMqtt()
          {
                  SOUT("Connecting to MQTT ...");
              //AsyncMqttClient& setKeepAlive(uint16_t `keepAlive`);   // Set keep alive. Defaults to 15 seconds
              //AsyncMqttClient& setClientId(const char\* `clientId`); // Defaults to `esp8266<chip ID on 6 hex caracters>`
              //AsyncMqttClient& setCleanSession(bool `cleanSession`); // Defaults to `true`
              //AsyncMqttClient& setMaxTopicLength(uint16_t `maxLen`); // Defaults to `128`
              //AsyncMqttClient& setCredentials(const char\* `username`, const char\* `password` = nullptr);
              //AsyncMqttClient& setWill(const char\* `topic`, uint8_t `qos`, bool `retain`, const char\* `payload` = nullptr, size_t `length` = 0); //Defaults to none
                  SOUT("  connect ...");
            mqttClient.connect();
                  SOUT("  MQTT connected");
          }

        void onMqttConnect(bool sessionPresent)
          {
            char temp[32] = "";
            uint16_t packetIdSub;
            SOUT("Connected to MQTT ");
            SOUT("Session present: ");
            SOUTLN(sessionPresent);
            /*
              for (uint8_t i=0; i<6; i++)
                {
                  switch(i)
                    {
                      case 0:
                        sprintf(temp, "%s%s", MQTT_DEVICE, BME2801T_MQTT);
                        break;
                      case 1:
                        sprintf(temp, "%s%s", MQTT_DEVICE, BME2801P_MQTT);
                        break;
                      case 2:
                        sprintf(temp, "%s%s", MQTT_DEVICE, BME2801H_MQTT);
                        break;
                      case 3:
                        sprintf(temp, "%s%s", MQTT_DEVICE, PHOTO1_MQTT);
                        break;
                      case 4:
                        sprintf(temp, "%s%s", MQTT_DEVICE, POTI1_MQTT);
                        break;
                      case 5:
                        sprintf(temp, "%s%s", MQTT_DEVICE, VCC50_MQTT);
                        break;
                    }
                  packetIdSub = mqttClient.subscribe(temp, 0);
                        SOUT("Subscribing "); SOUT(temp); SOUT(" Id: ");
                        SOUTLN(packetIdSub);
                    //mqttClient.publish("temp", 0, true);
                }
              */
          }

        void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
          {
            SOUTLN("Disconnected from MQTT.");
            if (WiFi.isConnected())
              {
                xTimerStart(mqttReconnectTimer, 0);
              }
          }

        void onMqttSubscribe(uint16_t packetId, uint8_t qos)
          {
            SOUT("Subscribe ack Id/qos "); SOUT(packetId); SOUT("/");
            SOUTLN(qos);
          }

        void onMqttUnsubscribe(uint16_t packetId)
          {
            SOUTLN("Unsubscribe acknowledged.");
            SOUT("  packetId: ");
            SOUTLN(packetId);
          }

        void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
          {
            SOUT("Publish rec topic/payload/len: ");
            SOUT(topic); SOUT("/"); SOUT(payload); SOUT("/"); SOUTLN(len);
                  //SOUT("  qos: ");   SOUTLN(properties.qos);
                  //SOUT("  dup: ");   SOUTLN(properties.dup);
                  //SOUT("  retain: ");SOUTLN(properties.retain);
                  //SOUT("  len: ");   SOUTLN(len);
                  //SOUT("  index: "); SOUTLN(index);
                  //SOUT("  total: "); SOUTLN(total);
          }

        void onMqttPublish(uint16_t packetId)
          {
            SOUT("Publish ack Id"); SOUTLN(packetId);
          }
#endif
      #endif
  // --- error ESP -------------------------
    void logESP(const esp_err_t _err, const char *_msg, uint8_t nr, bool _stop)
      {
        if (_err == ESP_OK) { return; }  // no error
        // error
        Serial.printf("\n  %i  ", nr);
        SOUT(_msg);
        Serial.printf(" %i - 0x%H\n", _err, _err);
        usleep(500000);

        if (_stop)
          {
            SOUTLN(); SOUTLN(" !!! STOP for ever ... goodby !!!");
            while (1)
              {
                sleep(1000);
              }
          }
      }
// --- end of code -----------------------------------------------