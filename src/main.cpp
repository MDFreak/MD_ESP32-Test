#include <main.h>
#include <prj_config.h>

// ---------------------------------------
// --- declarations
  // ------ system -----------------------
    int16_t _tmp = 0;
    bool firstrun = true;
    uint16_t     md_error  = 0    // Error-Status bitkodiert -> 0: alles ok
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
    TwoWire i2c1 = TwoWire(0);
    // cycletime measurement
    uint64_t anzUsCycles = 0;
    uint64_t usLast = 0;
    uint64_t usPerCycle = 0;
    uint64_t anzMsCycles = 0;
    uint64_t msLast = 0;
    uint64_t msPerCycle = 0;

    #if ( USE_I2C > 1 )
        TwoWire i2c2 = TwoWire(1);
      #endif

    #if ( USE_LED_BLINK_OUT > 0 )
        msTimer ledT = msTimer(BLINKTIME_MS);
        bool LED_ON = FALSE;
      #endif

    #if ( USE_DISP > 0 )
        msTimer       dispT  = msTimer(DISP_CYCLE);
        uint32_t      ze     = 1;      // aktuelle Schreibzeile
        char          outBuf[DISP1_MAXCOLS + 1] = "";
        String        outStr;
      #endif

        //
    #ifdef USE_STATUS
        msTimer     statT  = msTimer(STAT_TIMEDEF);
        char        statOut[DISP1_MAXCOLS + 1] = "";
        bool        statOn = false;
        bool        statDate = false;
        //char        timeOut[STAT_LINELEN + 1] = "";
      #endif

  // ------ user input ---------------
    #if (USE_CNT_INP > OFF)
      #endif

    #if (USE_TOUCHSCREEN > OFF)
        md_touch touch = md_touch();
      #endif

    #if (USE_KEYPADSHIELD > OFF)
        md_kpad kpad(KEYS_ADC);
        uint8_t key;
      #endif // USE_KEYPADSHIELD

    #if (USE_WS2812_LINE_OUT >OFF)
        CRGBPalette16 currentPalette;
        TBlendType    currentBlending;
        extern CRGBPalette16 myRedWhiteBluePalette;
        extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;
        CRGB leds[LEDS_2812_1];
        msTimer ws2812T   = msTimer(10);
        static uint8_t startIndex = 0;
        unsigned long ws2812_alt = 0;
        uint32_t ws2812_cnt = 0;
        uint32_t ws2812_v = 0;
      #endif

    #if (USE_CTRL_POTI_ADC > OFF)
        uint16_t inpValADC[USE_CTRL_POTI_ADC];
      #endif

    #if (USE_DIG_INP > OFF)
        uint8_t  inpValDig[USE_DIG_INP];
      #endif

  // ------ user output ---------------
    #if (USE_RGBLED_PWM > OFF)
        outRGBVal_t outValRGB[USE_RGBLED_PWM];
        #if (TEST_RGBLED_PWM > OFF)
            uint8_t  colRGBLED = 0;
            uint16_t incRGBLED = 10;
            uint32_t RGBLED_gr = 0;
            uint32_t RGBLED_bl = 0;
            uint32_t RGBLED_rt = 0;
          #endif
      #endif

    #if (USE_BUZZER_PWM > OFF)
        md_buzzer     buzz       = md_buzzer();
        #ifdef PLAY_MUSIC
            tone_t test = {0,0,0};
          #endif
      #endif // USE_BUZZER_PWM

    #if (USE_FAN_PWM > OFF)
        #if (USE_POTICTRL_FAN > OFF)
            uint8_t valFan[USE_FAN_PWM] = { 0, 0 };
          #endif
      #endif

    #if (USE_OLED_I2C > OFF)
        #ifdef OLED1
            #if !(OLED1_DRV ^ OLED_DRV_1106)
                md_oled_1106 oled1 = md_oled_1106((uint8_t) I2C_ADDR_OLED1, (uint8_t) I2C_SDA_OLED1,
                                        (uint8_t) I2C_SCL_OLED1, (OLEDDISPLAY_GEOMETRY) OLED1_GEO);
              #else
                md_oled_1306 oled1 = md_oled_1306((uint8_t) I2C_ADDR_OLED1, (uint8_t) I2C_SDA_OLED1,
                                        (uint8_t) I2C_SCL_OLED1, (OLEDDISPLAY_GEOMETRY) OLED1_GEO);
              #endif
          #endif
        #ifdef OLED2
            #if !(OLED2_DRV ^ OLED_DRV_1106)
                md_oled_1106 oled2 = md_oled_1106((uint8_t) I2C_ADDR_OLED2, (uint8_t) I2C_SDA_OLED2,
                                        (uint8_t) I2C_SCL_OLED2, (OLEDDISPLAY_GEOMETRY) OLED2_GEO);
              #else
                md_oled_1306 oled2 = md_oled_1306((uint8_t) I2C_ADDR_OLED2, (uint8_t) I2C_SDA_OLED2,
                                        (uint8_t) I2C_SCL_OLED2, (OLEDDISPLAY_GEOMETRY) OLED2_GEO);
              #endif
          #endif
        msTimer oledT   = msTimer(DISP_CYCLE);
        uint8_t oledIdx = 0;
      #endif //USE_OLED_I2C

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
        md_server webMD = md_server();
        msTimer   servT = msTimer(WEBSERVER_CYCLE);
      #endif // USE_WEBSERVER

  // ------ sensors ----------------------
    #ifdef USE_MEASURE_CYCLE
        msTimer measT   = msTimer(MEASURE_CYCLE_MS);
      #endif
    #if (USE_DS18B20_1W_IO > OFF)
        OneWire dsOneWire(DS_ONEWIRE_PIN);
        DallasTemperature dsSensors(&dsOneWire);
        DeviceAddress     dsAddr[DS18B20_ANZ];
        float dsTemp[DS18B20_ANZ];
      #endif

    #if (USE_BME280_I2C > OFF)
        Adafruit_BME280 bme;
        Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
        Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
        Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
      #endif

    #if (USE_MQ135_GAS_ADC > OFF)
        filterValue valGas(MQ135_FILT, 1);
        //filterValue tholdGas(MQ135_ThresFilt,1);
        int16_t gasValue;
        int16_t gasThres;
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
    #if (USE_FRAM_I2C > OFF)
        md_FRAM fram = md_FRAM();
      #endif
// --- system setup -----------------------------------
  void setup()
    {
      // --- system
        // start system
          Serial.begin(SER_BAUDRATE);
          delay( 3000 ); // power-up safety delay
          SOUTLN(); SOUTLN("setup start ...");
          #ifdef SCAN_I2C
              scanI2C(I2C1, 0, SCAN_I2C, PIN_I2C1_SDA, PIN_I2C1_SCL);
              #if (USE_I2C > 1)
                  scanI2C(I2C2, 0, SCAN_I2C, PIN_I2C2_SDA, PIN_I2C2_SCL);
                #endif
            #endif

          #if (USE_LED_BLINK_OUT > 0)
              pinMode(PIN_BOARD_LED, OUTPUT);
              digitalWrite(PIN_BOARD_LED, ON);
              LED_ON = ON;
            #endif

      // --- user output
        // start display - output to user
          #if (USE_TRAFFIC_LED_OUT > 0)
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
          dispStatus("setup start ...");

        // WS2812 LEDs
          #if (USE_WS2812_LINE_OUT > OFF)
              FastLED.addLeds<TYPE_2812_1, PIN_WS2812_D1, COLORD_2812_1>(leds, LEDS_2812_1).setCorrection(TypicalLEDStrip);
              FastLED.setBrightness(BRIGHT_2812_1);
              currentPalette = RainbowColors_p;
              currentBlending = LINEARBLEND;
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
                pinMode(PIN_PWM_FAN_2, OUTPUT);
                ledcSetup(PWM_FAN_2, PWM_FAN_FREQ, PWM_FAN_RES);
                ledcAttachPin(PIN_PWM_FAN_2, PWM_FAN_2);
                ledcWrite(PWM_FAN_2, 255);
                SOUTLN("Test Fan 2");
                sleep(1);
                ledcWrite(PWM_FAN_2, 0);

            #endif

        // start freq generator
          #if (USE_BUZZER_PWM > OFF)

            #endif
      // --- user input
        // start digital inputs
          #if (USE_DIG_INP > OFF)
              SOUT("config digSW Pins " );
              for (uint8_t i = 0 ; i < USE_DIG_INP ; i++ )
                {
                  pinMode(PIN_DIG_INP[i], INPUT_PULLUP);
                  SOUT(PIN_DIG_INP[i]); SOUT(" ");
                }
              SOUTLN();
            #endif
        // start poti inputs
          #if (USE_ADC1 > OFF)
              SOUT("config ADCs Pins " );
              for (uint8_t i = 0 ; i < USE_CTRL_POTI_ADC ; i++ )
                {
                  pinMode(PIN_ADC_CONF[i].pin, INPUT);
                  adc1_config_channel_atten((adc1_channel_t) PIN_ADC_CONF[i].channel,
                                            (adc_atten_t)   PIN_ADC_CONF[i].atten);
                  SOUT(PIN_ADC_CONF[i].pin); SOUT(" (");
                  SOUT(PIN_ADC_CONF[i].channel); SOUT(", ");
                  SOUT(PIN_ADC_CONF[i].atten) ; SOUT(") ");
                }
              SOUTLN();
            #endif
        // start counter
          #if (USE_CNT_INP > OFF)
              SOUT("config counter Pins " );
              for (uint8_t i = 0 ; i < USE_CTRL_POTI_ADC ; i++ )
                {
                  pinMode(PIN_INP_CNT[i], INPUT_PULLUP);
                  SOUT(PIN_INP_CNT[i]); SOUT(" ");
                }
            #endif
      // --- network
        // start WIFI
          #if (USE_WIFI > OFF)
            startWIFI(true);
            if ((md_error & ERRBIT_WIFI) == 0)
                dispStatus("WIFI connected");
              else
                dispStatus("WIFI error");
              if ((md_error & ERRBIT_WIFI) == 0)
                {
                  dispStatus("WIFI connected");
                }
                else
                {
                  dispStatus("WIFI error");
                }
              #endif // USE_WIFI
        // start Webserer
          #if (USE_WEBSERVER > OFF)
              {
                servT.startT();
                startWebServer();
                //md_error = setBit(md_error, ERRBIT_SERVER, webMD.md_handleClient());

              }
            #endif

      // --- sensors
        // temp. sensor DS18D20
          #if (USE_DS18B20_1W_IO > OFF)
                  SOUT(millis()); SOUT(" DS18D20 ... " );
              dsSensors.begin();
              String DS18Str = getDS18D20Str();
              dispStatus(DS18Str);
                  SOUTLN(DS18Str);
            #endif
        // BME280 temperature, pessure, humidity
          #if ( USE_BME280_I2C > OFF )
                    SOUT(millis()); SOUT(" BME280 ... " );
                bool bmeda = false;
                #if defined( I2C_BME2801_USE_I2C1 )
                    bmeda = bme.begin(I2C_ADDR_BME2801, &i2c1);
                  #endif
                #if defined( I2C_BME2801_USE_I2C2 )
                    bmeda = bme280.begin(I2C_ADDR_BME2801, &i2c2);
                  #endif
                if (bmeda)
                    {
                            SOUTLN(" gefunden");
                      bme.setSampling(bme.MODE_SLEEP);
                      String stmp = getBME280Str();
                            SOUTLN(stmp);
                      //bme_temp->printSensorDetails();
                      //bme_pressure->printSensorDetails();
                      //bme_humidity->printSensorDetails();

                    }
                  else
                    {
                      SOUT(" nicht gefunden");
                    }
            #endif

        // K-type thermoelementation
          #if ( USE_TYPE_K_SPI > 0)
                    SOUT(millis()); SOUT(" Tcouple1 ... " );
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
        // FRAM
          #if (USE_FRAM_I2C > OFF) // NIO funktioniert nicht
            // Read the first byte
            SOUT("FRAM addr "); SOUTHEX(I2C_ADDR_FRAM1);
            bool ret = !fram.begin(I2C_SDA_FRAM1, I2C_SCL_FRAM1, I2C_ADDR_FRAM1);
            if (ret == ISOK)
              {
                SOUT(" ok ProdID= ");
                uint16_t prodID, manuID;
                fram.getDeviceID(&manuID, &prodID);
                SOUT(" product "); SOUT(prodID); SOUT(" producer "); SOUTLN(manuID);

                SOUT(" FRAM selftest "); SOUT(fram.selftest());
              }
            #endif

      // --- finish setup
          #if (DEBUG_MODE >= CFG_DEBUG_STARTUP)
              #if (USE_LED_BLINK_OUT > 0)
                  digitalWrite(PIN_BOARD_LED, OFF);
                  LED_ON = OFF;
                #endif
              SOUTLN();
              SOUT("... end setup -- error="); SOUTLN(md_error);
              SOUTLN();
            #endif
    }

// ---------------------------------------
// --- system run = endless loop
  void loop()
    {
      //uint16_t t_x = 0, t_y = 0; // To store the touch coordinates
      #if (USE_WIFI > OFF)  // restart WIFI if offline
          if(wifiT.TOut())
            {
              //Serial.print("WiFi md_error = "); Serial.println(md_error);
              wifiT.startT();
              if((md_error & ERRBIT_WIFI) > 0)
                {
                  SOUTLN("WiFi startWIFI");
                  dispStatus("WiFi startWIFI");
                  startWIFI(false);
                }
            }
        #endif // USE_WIFI

      // ----------------------
      #if (USE_NTP_SERVER > OFF)   // get time from NTP server
        if (ntpT.TOut() == true)
          {
            setTime(++ntpTime);
            if ((md_error & ERRBIT_WIFI) == 0)
              { // WiFi online
                if (((md_error & ERRBIT_NTPTIME) > 0) || (year() < 2000))   // time not initialized
                  {
                    initNTPTime();
                    ntpGet = true;
                  }
                if (ntpGet == true)
                  {
                    ntpGet = wifi.getNTPTime(&ntpTime);
                    setTime(ntpTime);
                  }
              }
            ntpT.startT();
                  #if (DEBUG_MODE == CFG_DEBUG_DETAILS)
                    //SOUT("Datum "); SOUT(day()); SOUT("."); SOUT(month()); SOUT("."); SOUT(year()); SOUT(" ");
                    //SOUT("Zeit "); SOUT(hour()); SOUT("."); SOUT(minute()); SOUT(":"); SOUTLN(second());
                  #endif
          }
        #endif // USE_NTP_SERVER


      // ----------------------
      #if (USE_WEBSERVER > OFF)    // run webserver -> restart/run not allowed in loop task
          if (servT.TOut())
            {
              servT.startT();
              /*
              if ((md_error & ERRBIT_SERVER) != 0)
                {;}//startWebServer();
              else
                //bool ret = webMD.md_handleClient();
                md_error = setBit(md_error, ERRBIT_SERVER, webMD.md_handleClient());
              */
            }

        #endif
      // ----------------------
      #if (USE_WS2812_LINE_OUT > OFF)
          if (ws2812T.TOut())
            {
              ws2812T.startT();
              ChangePalettePeriodically();
              startIndex = startIndex + 1; /* motion speed */
              ws2812_cnt++;
                    //SOUT(millis()); SOUT(" "); SOUTLN(ws2812_cnt);
              FillLEDsFromPaletteColors( startIndex);
              FastLED.show();
            }
        #endif

      // ----------------------
      #if (USE_TOUCHSCREEN > OFF)
        touch.runTouch(outBuf);
        #endif // USE_TOUCHSCREEN

      // ----------------------
      #if (USE_KEYPADSHIELD > OFF)
        key = getKey();
        if (key)
          {
            sprintf(outBuf,"key %d", key);
            dispStatus(outBuf);
          }
        #endif
      // ----------------------
      #ifdef USE_MEASURE_CYCLE
          if (measT.TOut())
            {
              measT.startT();
              #if (USE_MQ135_GAS_ADC > OFF)
                  gasValue = analogRead(PIN_MQ135);
                        //SOUT(millis()); SOUT(" gas measurment val = "); SOUTLN(gasValue);
                  gasValue = (int16_t) valGas.value((double) gasValue);
                        //SOUT(millis()); SOUT("    gasValue = "); SOUTLN(gasValue);
                  //gasThres = analogRead(PIN_CO2_THOLD);
                        //SOUT(millis()); SOUT(" gas threshold val = "); SOUTLN(gasThres);
                  //gasThres = (int16_t) tholdGas.value((double) gasThres);
                        //SOUT(millis()); SOUT("    gasThres = "); SOUTLN(gasThres);
                #endif
              #if (USE_RGBLED_PWM > OFF)
                  {
                    #if (TEST_RGBLED_PWM > OFF)
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

                        #if (USE_WEBCTRL_RGB > OFF)
                            _tmp += 4;
                            if (_tmp > 50)
                              { _tmp = 0; }
                            //SOUT(millis()); SOUT(" _tmp = "); SOUTLN(_tmp);
                            ledcWrite(PWM_RGB_RED,   webMD.getDutyCycle(0));
                            ledcWrite(PWM_RGB_GREEN, webMD.getDutyCycle(1));
                            ledcWrite(PWM_RGB_BLUE,  webMD.getDutyCycle(2));
                          #endif

                        ledcWrite(PWM_RGB_RED,   RGBLED_rt);
                        ledcWrite(PWM_RGB_GREEN, RGBLED_gr);
                        ledcWrite(PWM_RGB_BLUE,  RGBLED_bl);

                      #endif

                    #if (USE_ADC1 > OFF)
                        getADCIn();
                      #endif

                    #if (USE_DIG_INP > OFF)
                        getDIGIn();
                      #endif

                    #if (USE_CNT_INP > OFF)
                        getCNTIn();
                      #endif

                    #if (USE_POTICTRL_FAN > 0)
                        valFan[INP_CNT_FAN_1] = (uint8_t) map((long) -inpValADC[INP_POTI_CTRL], -4095, 0, 0, 100);
                        valFan[INP_CNT_FAN_2] = map((long) -inpValADC[INP_POTI_CTRL], -4095, 0, 0, 255);
                        //SOUT(inpValADC[INP_POTI_CTRL]); SOUT(" "); SOUTLN(valFan[INP_CNT_FAN_1]);
                        ledcWrite(PWM_FAN_1, valFan[INP_CNT_FAN_1]);
                        ledcWrite(PWM_FAN_2, valFan[INP_CNT_FAN_2]);
                      #endif
                  }
                #endif
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
            }
        #endif
      // ----------------------
      #if (USE_DISP > 0)
        if (dispT.TOut())    // handle touch output
          {
            dispT.startT();
            #ifdef RUN_OLED_TEST
                oled.clearBuffer();
                switch (oledIdx)
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
                      oledIdx--;
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
                if (++oledIdx > 6) { oledIdx = 0; }
                oled.sendBuffer();
              #endif // RUN_OLED_TEST
            oledIdx++;
            switch (oledIdx)
              {
              case 1: // system output
                SOUTLN(); SOUT(millis()); SOUT(" us/cyc "); SOUT((uint32_t) usPerCycle); SOUT(" ");
                break;

              case 2: // webserver nu
                #if (USE_WEBSERVER > OFF)
                  #endif
                break;

              case 3: // k-type sensor
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

              case 4: // gas sensor
                #if (USE_MQ135_GAS_ADC > OFF)
                  outStr = "";
                  //_tmp = showTrafficLight(gasValue, gasThres); // -> rel to defined break point
                  outStr = "CO2 ";
                  outStr.concat(gasValue);
                  //outStr.concat(" (");
                  //outStr.concat(_tmp);
                  //outStr.concat(")");
                  dispText(outStr ,  0, 1, outStr.length());
                          SOUT(outStr); SOUT("  ");
                  #endif
                break;

              case 5: // temp sesor
                #if (USE_DS18B20_1W_IO > OFF)
                  outStr = "";
                  outStr = getDS18D20Str();
                  dispText(outStr ,  0, 4, outStr.length());
                  #endif
                break;

              case 6: // BME 280 temp, humidity, pressure
                #if ( USE_BME280_I2C > OFF )
                  outStr = getBME280Str();
                          SOUT(outStr); SOUT(" ");
                  dispText(outStr , 0, 3, outStr.length());
                  #endif
                break;

              case 7: // test values
                SOUT("SW1 "); SOUT(inpValDig[INP_SW_CTRL]); SOUT(" ");
                SOUT("POT "); SOUT(inpValADC[INP_POTI_CTRL]); SOUT(" ");
                break;

              case 8: // WS2812 lines
                #if (USE_WS2812_LINE_OUT > OFF)
                  outStr = "              ";
                  dispText(outStr ,  0, 0, outStr.length());
                  //outStr = "LED ";
                  outStr = "";
                      //outStr += (String) ws2812_cnt; outStr += " ";
                  ws2812_v = millis() - ws2812_alt; // dispT.getTout();
                  ws2812_alt = millis();
                  if (ws2812_cnt > 0)
                    {
                      ws2812_v = ws2812_v / ws2812_cnt;
                      ws2812_cnt = 0;
                    }
                  outStr += (String) ws2812_v;
                  outStr += ("ms");
                        //SOUT((uint32_t) millis()); SOUT(" ");
                            SOUT(outStr); SOUT(" ");
                  dispText(outStr ,  0, 0, outStr.length());
                  #endif
                break;

              default:
                SOUTLN();
                oledIdx = 0;
                break;
              }
            #ifdef USE_STATUS
                dispStatus("");
              #endif
          }
        #endif // defined(DISP)

      // --- system control --------------------------------
        #if (USE_LED_BLINK_OUT > 0)
          if (ledT.TOut())    // handle touch output
            {
              ledT.startT();
              if (LED_ON == TRUE)
                  {
                    digitalWrite(PIN_BOARD_LED, OFF);
                    LED_ON = OFF;
                  }
                else
                  {
                    digitalWrite(PIN_BOARD_LED, ON);
                    LED_ON = ON;
                  }
            }
          #endif

        if (!firstrun)
          {
            anzUsCycles++;
            if (anzUsCycles > 10000)
              {
                usPerCycle = (micros() - usLast) / anzUsCycles;
                anzUsCycles = 0;
                usLast = micros();
              }
            anzMsCycles++;

          }


        else
          {
            String taskMessage = "loop task running on core ";
            taskMessage = taskMessage + xPortGetCoreID();
            SOUTLN(taskMessage);
            usLast = micros();
            firstrun = false;
          }
      usleep(14);
    }
//
// --- subroutine and drivers ----------------
  // --- user output
    // --- display
      void clearDisp()
        {
          #if (USE_DISP > 0)
              #if defined(OLED1)
                oled1.clear();
                #endif
              #if defined(OLED2)
                oled2.clear();
                #endif
            #endif
        }

      void dispStatus(String msg)
        {
          #ifdef USE_STATUS
            size_t statLen = msg.length();
            bool   doIt    = false;

            if (statLen)
              {
                if ( statLen > DISP1_MAXCOLS)
                  {
                    msg.remove(DISP1_MAXCOLS);
                  }
                statOn = true;
                statT.startT();
                doIt = true;    // output statOut
                statT.startT();
              }
            else // empty input
              if (statOn && statT.TOut())
                statOn = false;

            if (!statOn) // disp actual time
              {
                sprintf(statOut,"%02d.%02d. %02d:%02d:%02d ", day(), month(), hour(), minute(), second());
                msg = statOut;
                doIt = true;
              }
            if (doIt)
              {
                #if (USE_TOUCHSCREEN > OFF)
                  touch.wrStatus(msg);
                  #endif
                #if (USE_OLED_I2C > OFF)
                    #if defined( USE_STATUS1 )
                        oled1.wrStatus(msg);
                      #endif
                    #if defined( USE_STATUS2 )
                        oled2.wrStatus(msg);
                      #endif
                         //SOUT("  md_error="); SOUTLN(md_error);
                  #endif
                #if (USE_TFT > 0)
                    mlcd.wrStatus((char*) statOut);
                        #if (DEBUG_MODE >= CFG_DEBUG_DETAILS)
                            SOUT("  md_error="); SOUTLN(md_error);
                          #endif
                  #endif // USE_DISP
              }
            #endif // USE_STATUS
        }
      void dispStatus(const char* msg)
        {
          dispStatus((String) msg);
        }

      void dispText(char* msg, uint8_t col, uint8_t row, uint8_t len)
        {
          #if (USE_DISP > 0)
              #if (USE_TOUCHSCREEN > OFF)
                touch.wrTouch(msg, col, row);
                #endif
              #if defined(OLED1)
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
                mlcd.wrText(msg, row, col);
                #endif
            #endif
        }
      void dispText(String msg, uint8_t col, uint8_t row, uint8_t len)
        {
          #if (USE_DISP > 0)
              #if (USE_TOUCHSCREEN > OFF)
                touch.wrTouch(msg, col, row);
                #endif
              #if defined(OLED1)
                oled1.wrText(msg, col, row, len);
                          //SOUT((uint32_t) millis); SOUT(" dispText oled1 '"); SOUT(msg); SOUTLN("'");
                #endif
              #if defined(OLED2)
                oled2.wrText(msg, col, row, len);
                          //SOUT((uint32_t) millis); SOUT(" dispText oled2 '"); SOUT(msg); SOUTLN("'");
                #endif
              #if (USE_TFT > 0)
                mlcd.wrText(msg, row, col);
                #endif
            #endif
        }

      void startDisp()
        {
          #if (USE_DISP > 0)
              #ifdef USE_STATUS
                statOut[DISP1_MAXCOLS] = 0;  // limit strlen
                #endif

              #if (USE_TFT > 0)
                mlcd.start(plcd);
                #endif

              #if (USE_TOUCHSCREEN > OFF)
                bool ret = touch.startTouch();
                      #if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                        SOUT("startTouch ret="); SOUT(ret);
                      #endif
                md_error = setBit(md_error, ERRBIT_TOUCH, ret);
                      #if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                        SOUT("  md_error="); SOUTLN(md_error);
                      #endif
                #endif

              #if defined (OLED1)
                  oled1.begin((uint8_t) DISP1_MAXCOLS, (uint8_t) DISP1_MAXROWS);
                #endif

              #if defined (OLED2)
                  oled2.begin((uint8_t) DISP2_MAXCOLS, (uint8_t) DISP2_MAXROWS);
                #endif
            #endif
        }
    // --- WS2812 lines
      #if (USE_WS2812_LINE_OUT > OFF)
          void FillLEDsFromPaletteColors( uint8_t colorIndex)
            {
              uint8_t brightness = 255;
              for( int i = 0; i < LEDS_2812_1; i++)
                {
                  leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
                  colorIndex += 3;
                }
          }
          /* There are several different palettes of colors demonstrated here.
            //
            // FastLED provides several 'preset' palettes: RainbowColors_p, RainbowStripeColors_p,
            // OceanColors_p, CloudColors_p, LavaColors_p, ForestColors_p, and PartyColors_p.
            //
            // Additionally, you can manually define your own color palettes, or you can write
            // code that creates color palettes on the fly.  All are shown here.
            */

          void ChangePalettePeriodically()
            {
              uint8_t secondHand = (millis() / 1000) % 60;
              static uint8_t lastSecond = 99;

              if( lastSecond != secondHand)
                {
                  lastSecond = secondHand;
                  if( secondHand ==  0)  { currentPalette = RainbowColors_p;         currentBlending = LINEARBLEND; }
                  if( secondHand == 10)  { currentPalette = RainbowStripeColors_p;   currentBlending = NOBLEND;  }
                  if( secondHand == 15)  { currentPalette = RainbowStripeColors_p;   currentBlending = LINEARBLEND; }
                  if( secondHand == 20)  { SetupPurpleAndGreenPalette();             currentBlending = LINEARBLEND; }
                  if( secondHand == 25)  { SetupTotallyRandomPalette();              currentBlending = LINEARBLEND; }
                  if( secondHand == 30)  { SetupBlackAndWhiteStripedPalette();       currentBlending = NOBLEND; }
                  if( secondHand == 35)  { SetupBlackAndWhiteStripedPalette();       currentBlending = LINEARBLEND; }
                  if( secondHand == 40)  { currentPalette = CloudColors_p;           currentBlending = LINEARBLEND; }
                  if( secondHand == 45)  { currentPalette = PartyColors_p;           currentBlending = LINEARBLEND; }
                  if( secondHand == 50)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = NOBLEND;  }
                  if( secondHand == 55)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = LINEARBLEND; }
                }
            }

          // This function fills the palette with totally random colors.
          void SetupTotallyRandomPalette()
            {
              for( int i = 0; i < 16; i++)
                {
                  currentPalette[i] = CHSV( random8(), 255, random8());
                }
            }

          /* This function sets up a palette of black and white stripes,
            // using code.  Since the palette is effectively an array of
            // sixteen CRGB colors, the various fill_* functions can be used
            // to set them up.
            */
          void SetupBlackAndWhiteStripedPalette()
            {
              // 'black out' all 16 palette entries...
              fill_solid( currentPalette, 16, CRGB::Black);
              // and set every fourth one to white.
              currentPalette[0] = CRGB::White;
              currentPalette[4] = CRGB::White;
              currentPalette[8] = CRGB::White;
              currentPalette[12] = CRGB::White;

            }

          // This function sets up a palette of purple and green stripes.
          void SetupPurpleAndGreenPalette()
            {
              CRGB purple = CHSV( HUE_PURPLE, 255, 255);
              CRGB green  = CHSV( HUE_GREEN, 255, 255);
              CRGB black  = CRGB::Black;

              currentPalette = CRGBPalette16(
                                             green,  green,  black,  black,
                                             purple, purple, black,  black,
                                             green,  green,  black,  black,
                                             purple, purple, black,  black );
            }

          /* This example shows how to set up a static color palette
            // which is stored in PROGMEM (flash), which is almost always more
            // plentiful than RAM.  A static PROGMEM palette like this
            // takes up 64 bytes of flash.
            */
          const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM =
            {
              CRGB::Red,
              CRGB::Gray, // 'white' is too bright compared to red and blue
              CRGB::Blue,
              CRGB::Black,

              CRGB::Red,
              CRGB::Gray,
              CRGB::Blue,
              CRGB::Black,

              CRGB::Red,
              CRGB::Red,
              CRGB::Gray,
              CRGB::Gray,
              CRGB::Blue,
              CRGB::Blue,
              CRGB::Black,
              CRGB::Black
            };

          /* Additionl notes on FastLED compact palettes:
            //
            // Normally, in computer graphics, the palette (or "color lookup table")
            // has 256 entries, each containing a specific 24-bit RGB color.  You can then
            // index into the color palette using a simple 8-bit (one byte) value.
            // A 256-entry color palette takes up 768 bytes of RAM, which on Arduino
            // is quite possibly "too many" bytes.
            //
            // FastLED does offer traditional 256-element palettes, for setups that
            // can afford the 768-byte cost in RAM.
            //
            // However, FastLED also offers a compact alternative.  FastLED offers
            // palettes that store 16 distinct entries, but can be accessed AS IF
            // they actually have 256 entries; this is accomplished by interpolating
            // between the 16 explicit entries to create fifteen intermediate palette
            // entries between each pair.
            //
            // So for example, if you set the first two explicit entries of a compact
            // palette to Green (0,255,0) and Blue (0,0,255), and then retrieved
            // the first sixteen entries from the virtual palette (of 256), you'd get
            // Green, followed by a smooth gradient from green-to-blue, and then Blue.
            */
        #endif

    // --- passive buzzer
      #ifdef PLAY_MUSIC
          void playSong(int8_t songIdx)
            {
              if (buzz.setSong(SONG0_LEN,(void*) SONG0_NOTES) == ISOK)
                {
                  #ifndef USE_SONGTASK
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
  // --- user input
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
          void getCNTIn()
            {

            }
        #endif

      #if (USE_DIG_INP > OFF)
          void getDIGIn()
            {
              for ( uint8_t i=0 ; i < USE_DIG_INP ; i++ )
                {
                  inpValDig[i] = digitalRead(PIN_DIG_INP[i]);
                }
            }
        #endif

      #if (USE_CTRL_POTI_ADC > OFF)
          void getADCIn()
            {
              for (uint8_t i = 0 ; i < USE_CTRL_SW_INP ; i++ )
                {
                  inpValADC[i] = analogRead(PIN_ADC_CONF[i].pin);
                }
            }
        #endif

  // --- sensors
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
      String getBME280Str()
        {
          String _outS = "";
          #if ( USE_BME280_I2C > OFF )
            bme.init();
            usleep(1000);
            _outS  = ((String) ((int) (bme.readTemperature()         + 0.5))) + "° ";
            usleep(1000);
            _outS += ((String) ((int) (bme.readHumidity()            + 0.5))) + "% ";
            usleep(1000);
            _outS += ((String) ((int) ((bme.readPressure() / 100.0F) + 0.5))) + "mb";
            //outS += "  Alt~ " + (String) bme280.readAltitude(1013.25);

                    //SOUT(" BME280 "); SOUTLN(_outS);
            #endif
          return _outS;
        }
    // --- T-element type K
  // ------ network -------------------------
    // --- WIFI
      void startWIFI(bool startup)
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
                    ipList.append(WIFI_FIXIP3, WIFI_GATEWAY4, WIFI_SUBNET, WIFI_SSID4, WIFI_SSID4_PW);
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
              }
            ret = wifi.startWIFI();
                        SOUT("startWIFI ret="); SOUT(ret);
            md_error = setBit(md_error, ERRBIT_WIFI, ret);
                  #if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                    SOUT("  md_error="); SOUTLN(md_error);
                    #endif

            if ((md_error & ERRBIT_WIFI) == 0)
                dispStatus("WIFI connected");
              else
                dispStatus("WIFI error");

            #if (USE_NTP_SERVER > OFF)
              if((md_error & ERRBIT_WIFI) == 0) // WiFi ok
                  if((md_error & ERRBIT_NTPTIME) != 0) // WiFi ok
                    wifi.initNTP();
              #endif
            #endif // USE_WIFI
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
            if ((md_error & ERRBIT_WIFI) == 0)
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
      void configWebsite()
        {
          #if (USE_WEBSERVER > OFF)
            webMD.createElement(EL_TYPE_SLIDER, "LED red", "%");
            webMD.createElement(EL_TYPE_SLIDER, "LED green", "%");
            webMD.createElement(EL_TYPE_SLIDER, "LED blue", "%");

            webMD.createElement(EL_TYPE_ANALOG, "DS18B20 Temp", "°C");
            webMD.createElement(EL_TYPE_ANALOG, "Type-K Temp", "°C");
            webMD.createElement(EL_TYPE_ANALOG, "BME_Temp", "°C");
            webMD.createElement(EL_TYPE_ANALOG, "BME_Humidity", "%");
            webMD.createElement(EL_TYPE_ANALOG, "BME_Pressure", "mb");
            webMD.createElement(EL_TYPE_ANALOG, "Gaswert", "");
          #endif // USE_WEBSERVER
        }

      void startWebServer()
        {
          #if (USE_WEBSERVER > OFF)
            bool ret = ISERR;
            if ((md_error & ERRBIT_SERVER) != 0)
              {
                dispStatus("start webserver");
                if ((md_error & ERRBIT_WIFI) == 0)
                  {
                    ret = webMD.md_startServer(0, 3, 0);
                        #if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                          // SOUT("startServer ret="); SOUT(ret);
                        #endif
                  }
                md_error = setBit(md_error, ERRBIT_SERVER, ret);
                      #if (DEBUG_MODE >= CFG_DEBUG_DETAIL)
                        // SOUT("  md_error="); SOUTLN(md_error);
                      #endif

                if ((md_error & ERRBIT_SERVER) == 0)
                  {
                    dispStatus("Webserver online");
                  }
                  else
                  {
                    dispStatus("Webserver error");
                  }
              }
          #endif // USE_WEBSERVER
        }

// --- end of code --------------------------