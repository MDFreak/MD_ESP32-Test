#include "main.h"


// --- declarations
  // ------ system -----------------------
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
    #if ( USE_I2C > 1 )
        TwoWire i2c2 = TwoWire(1);
      #endif

    #if ( USE_LED_BLINK > 0 )
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

  // ------ user interface ---------------
    #if (USE_TOUCHSCREEN > OFF)
        md_touch touch = md_touch();
      #endif

    #if (USE_KEYPADSHIELD > OFF)
        md_kpad kpad(KEYS_ADC);
        uint8_t key;
      #endif // USE_KEYPADSHIELD

    #ifdef USE_BUZZER
        md_buzzer     buzz       = md_buzzer();
      #endif // USE_BUZZER

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

    //
    #if (USE_WEBSERVER > OFF)
        md_server webMD = md_server();
        msTimer   servT = msTimer(WEBSERVER_CYCLE);
      #endif // USE_WEBSERVER

  // ------ sensors ----------------------
    #ifdef USE_MEASURE_CYCLE
        msTimer measT   = msTimer(MEASURE_CYCLE_MS);
      #endif
    #if (USE_DS18B20_1W > OFF)
        OneWire dsOneWire(DS_ONEWIRE_PIN);
        DallasTemperature dsSensors(&dsOneWire);
        DeviceAddress     dsAddr[DS18B20_ANZ];
        float dsTemp[DS18B20_ANZ];
      #endif

    #if ( USE_BME280_I2C > OFF )
        Adafruit_BME280 bme;
        Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
        Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
        Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
      #endif

    #if (USE_MQ135_GAS_ANA > OFF)
        filterValue valGas(MQ135_FILT, 1);
        //filterValue tholdGas(MQ135_ThresFilt,1);
        int16_t gasValue;
        int16_t gasThres;
      #endif

    #if ( USE_TYPE_K > 0)
        //SPIClass* tkSPI = new SPIClass();
        //md_31855_ktype TypeK1(TYPEK1_CS_PIN, tkSPI);
        md_31855_ktype TypeK1(TYPEK1_CS_PIN, TYPEK_CLK_PIN, TYPEK_DATA_PIN);
        filterValue valTK1(TYPEK_FILT, TYPEK_DROP_PEEK, TYPEK1_OFFSET, TYPEK1_GAIN);
        filterValue valTK1ref(TYPEK_FILT, TYPEK_DROP_PEEK, TYPEK1_OFFSET, TYPEK1_GAIN);
        int16_t     tk1Val;
        int16_t     tk1ValRef;
        #if ( USE_TYPE_K > 1)
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
// --- private prototypes
  // ------ user interface -----------------
    // --- website
      #if (USE_WEBSERVER > OFF)
          void configWebsite();
        #endif
    // --- user output
      // standard outputs
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

      // --- start display
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
    // --- user input
      // --- keypad
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
  // ------ WIFI -------------------------
    #if (USE_WIFI > OFF)
      void startWIFI(bool startup)
        {
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
          ret = wifi.startWIFI(USE_LOCAL_IP);
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
                  wifi.initNTP(0);
            #endif
        }
      #endif // USE_WIFI

    #if (USE_WEBSERVER > OFF)
      void startWebServer()
        {
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
        }
      #endif // USE_WEBSERVER

    #if (USE_NTP_SERVER > OFF)
      void initNTPTime()
        {
          bool ret = wifi.initNTP(UTC_SEASONTIME);
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
        }
      #endif // USE_NTP_SERVER

  // ------ passive buzzer --------------
    #ifdef PLAY_MUSIC
        tone_t test = {0,0,0};
        void playSong(int8_t songIdx)
          {
            if (buzz.setSong(SONG0_LEN,(void*) SONG0_NOTES) == ISOK)
              {
                #ifndef USE_SONGTASK
                  buzz.playSong();
                #endif
              }
          }
        void playSong() { playSong(0); }
      #endif

  // ------ traffic Light of gas sensor --------------
    #if (USE_MQ135_GAS_ANA > OFF)
        int16_t showTrafficLight(int16_t inval, int16_t inthres);
      #endif

// --- system startup
  void setup()
    {
      // --- system
        // start system
          Serial.begin(SER_BAUDRATE);
          SOUTLN(); SOUTLN("setup start ...");

          #ifdef SCAN_I2C
              scanI2C(I2C1, 0, SCAN_I2C, PIN_I2C1_SDA, PIN_I2C1_SCL);
              #if (USE_I2C > 1)
                  scanI2C(I2C2, 0, SCAN_I2C, PIN_I2C2_SDA, PIN_I2C2_SCL);
                #endif
            #endif

          #if (USE_LED_BLINK > 0)
              pinMode(PIN_BOARD_LED, OUTPUT);
            #endif

      // --- user interface
        // start display - output to user
          #if (USE_TRAFFIC_LIGHT > 0)
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
          #if (USE_RGBLED > 0)
                pinMode(PIN_RGB_RED, OUTPUT);
                pinMode(PIN_RGB_GREEN, OUTPUT);
                pinMode(PIN_RGB_BLUE, OUTPUT);
                ledcSetup(PWM_RGB_RED,    PWM_LEDS_FREQ, PWM_LEDS_RES);
                ledcSetup(PWM_RGB_GREEN,  PWM_LEDS_FREQ, PWM_LEDS_RES);
                ledcSetup(PWM_RGB_BLUE,   PWM_LEDS_FREQ, PWM_LEDS_RES);
                ledcAttachPin(PIN_RGB_RED,   PWM_RGB_RED);
                ledcAttachPin(PIN_RGB_GREEN, PWM_RGB_GREEN);
                ledcAttachPin(PIN_RGB_BLUE,  PWM_RGB_BLUE);
                ledcWrite(PWM_RGB_GREEN, 255);
                SOUTLN("LED gruen");
                usleep(200000);
                ledcWrite(PWM_RGB_GREEN, 0);
                ledcWrite(PWM_RGB_BLUE, 255);
                SOUTLN("LED blau");
                usleep(200000);
                ledcWrite(PWM_RGB_BLUE, 0);
                ledcWrite(PWM_RGB_RED, 255);
                SOUTLN("LED rot");
                usleep(200000);
                ledcWrite(PWM_RGB_RED, 0);
            #endif
          startDisp();
          dispStatus("setup start ...");
        // start input device
          startKeys();
        // start buzzer (task)
          #if (USE_BUZZER > OFF)
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
          #if (USE_DS18B20_1W > OFF)
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
          #if ( USE_TYPE_K > 0)
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
                #if ( USE_TYPE_K > 1)
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
              SOUTLN();
              SOUT("... end setup -- error="); SOUTLN(md_error);
              SOUTLN();
            #endif
    }

// --- system run = endless loop
  int16_t _tmp = 0;
  bool firstrun = true;
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
              #if (USE_MQ135_GAS_ANA > OFF)
                  gasValue = analogRead(PIN_MQ135);
                        //SOUT(millis()); SOUT(" gas measurment val = "); SOUTLN(gasValue);
                  gasValue = (int16_t) valGas.value((double) gasValue);
                        //SOUT(millis()); SOUT("    gasValue = "); SOUTLN(gasValue);
                  //gasThres = analogRead(PIN_CO2_THOLD);
                        //SOUT(millis()); SOUT(" gas threshold val = "); SOUTLN(gasThres);
                  //gasThres = (int16_t) tholdGas.value((double) gasThres);
                        //SOUT(millis()); SOUT("    gasThres = "); SOUTLN(gasThres);
                #endif
              #if (USE_RGBLED > OFF)
                  {
                    _tmp += 4;
                    if (_tmp > 50)
                      { _tmp = 0; }
                    //SOUT(millis()); SOUT(" _tmp = "); SOUTLN(_tmp);
                    ledcWrite(PWM_RGB_RED,   webMD.getDutyCycle(0));
                    ledcWrite(PWM_RGB_GREEN, webMD.getDutyCycle(1));
                    ledcWrite(PWM_RGB_BLUE,  webMD.getDutyCycle(2));
                  }
                #endif
              #if (USE_TYPE_K > OFF)
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
                  #if (USE_TYPE_K > 1)
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
              #endif
              //#ifdef OLED_NOTEST
                oledIdx++;
                switch (oledIdx)
                  {
                  case 1:
                      /*
                      outStr = "0-0-6";
                      dispText(outStr ,  0, 0, 6);
                      outStr = "";
                      dispText(outStr ,  7, 0, 6);
                      outStr = "15-0-6";
                      dispText(outStr , 14, 0, 6);
                      */
                          //SOUT((uint32_t) millis()); SOUT(" SW1 '"); SOUT(outBuf); SOUTLN("'");
                    break;
                  case 2:
                      /*
                      outStr = "";
                      dispText(outStr ,  0, 0, 6);
                      outStr = "7-1-6";
                      dispText(outStr ,  7, 0, 6);
                      outStr = "";
                      dispText(outStr , 14, 0, 6);
                      */
                    break;
                  case 3:
                    #if (USE_TYPE_K > OFF)
                        outStr = "";
                        outStr = "TK1 ";
                        outStr.concat(tk1Val);
                        outStr.concat("°");
                        //dispText(outStr ,  0, 1, outStr.length());
                        #if (USE_TYPE_K > 1)
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
                                SOUTLN(outStr);
                      #endif
                    break;
                  case 4:
                    #if (USE_MQ135_GAS_ANA > OFF)
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
                  case 5:
                    #if (USE_DS18B20_1W > OFF)
                        outStr = "";
                        outStr = getDS18D20Str();
                        dispText(outStr ,  0, 4, outStr.length());
                      #endif
                    break;
                  case 6:
                    #if ( USE_BME280_I2C > OFF )
                        outStr = getBME280Str();
                                SOUT("  "); SOUT(outStr); SOUT("  ");
                        dispText(outStr ,  0, 3, outStr.length());
                      #endif
                    break;
                   default:
                    oledIdx = 0;
                    break;
                  }
              //#endif
            #ifdef USE_STATUS
              dispStatus("");
              #endif
          }
        #endif // defined(DISP)
      // ----------------------
      #if (USE_LED_BLINK > 0)
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
      // ----------------------
      if (firstrun == true)
        {
          String taskMessage = "loop task running on core ";
          taskMessage = taskMessage + xPortGetCoreID();
          SOUTLN(taskMessage);
          firstrun = false;
        }
      sleep(1);
    }
// --- subroutine drivers
  #if (USE_WEBSERVER > OFF)
      void configWebsite()
        {
          webMD.createElement(EL_TYPE_SLIDER, "LED red", "%");
          webMD.createElement(EL_TYPE_SLIDER, "LED green", "%");
          webMD.createElement(EL_TYPE_SLIDER, "LED blue", "%");

          webMD.createElement(EL_TYPE_ANALOG, "DS18B20 Temp", "°C");
          webMD.createElement(EL_TYPE_ANALOG, "Type-K Temp", "°C");
          webMD.createElement(EL_TYPE_ANALOG, "BME_Temp", "°C");
          webMD.createElement(EL_TYPE_ANALOG, "BME_Humidity", "%");
          webMD.createElement(EL_TYPE_ANALOG, "BME_Pressure", "mb");
          webMD.createElement(EL_TYPE_ANALOG, "Gaswert", "");

        }
    #endif
  #if (USE_DS18B20_1W > OFF)
      String getDS18D20Str()
        {
          String outS = "";
          dsSensors.requestTemperatures(); // Send the command to get temperatures
          for (uint8_t i = 0 ; i < DS18B20_ANZ ; i++ )
            {
              dsTemp[i] = dsSensors.getTempCByIndex(i);
                    //SOUTLN(dsTemp[i]);
              if (i < 1) { outS  = "T1 "; }
              else       { outS += "    T2  ";}
              outS += (String) ((int) (dsTemp[i] + 0.5)) + "°";
            }
          return outS;
        }
    #endif

  #if ( USE_BME280_I2C > OFF )
      String getBME280Str()
        {
          String _outS = "";
          bme.init();
          usleep(1000);
          _outS  = ((String) ((int) (bme.readTemperature()         + 0.5))) + "° ";
          usleep(1000);
          _outS += ((String) ((int) (bme.readHumidity()            + 0.5))) + "% ";
          usleep(1000);
          _outS += ((String) ((int) ((bme.readPressure() / 100.0F) + 0.5))) + "mb";
          //outS += "  Alt~ " + (String) bme280.readAltitude(1013.25);

                  //SOUT(" BME280 "); SOUTLN(_outS);
          return _outS;
        }
    #endif

/*
  #if (USE_MQ135_GAS_ANA > OFF)
      int16_t showTrafficLight(int16_t inval, int16_t inthres)
        {
          // int16_t mytmp = inval - inthres;
          int16_t mytmp = inthres;
                  //SOUT("  mytmp "); SOUTLN(mytmp);

          if (mytmp <= -(int16_t) MQ135_EM_WIN)
            {
              digitalWrite(PIN_TL_GREEN, ON);
              digitalWrite(PIN_TL_YELLOW, OFF);
              digitalWrite(PIN_TL_RED,    OFF);
            }
          else if ( mytmp <= 0 )
            {
              digitalWrite(PIN_TL_GREEN, ON);
              digitalWrite(PIN_TL_YELLOW, ON);
              digitalWrite(PIN_TL_RED, OFF);
            }
          else if (mytmp < (int16_t) MQ135_EM_WIN )
            {
              digitalWrite(PIN_TL_GREEN, OFF);
              digitalWrite(PIN_TL_YELLOW, ON);
              digitalWrite(PIN_TL_RED,    ON);
              #if defined(PLAY_START_DINGDONG)
                  //buzz.playDingDong(2);
                #endif
            }
          else // ( mytmp >= MQ135_EM_WIN )
            {
              digitalWrite(PIN_TL_GREEN,  OFF);
              digitalWrite(PIN_TL_YELLOW, OFF);
              digitalWrite(PIN_TL_RED,    ON);
              #if defined(PLAY_START_DINGDONG)
                  //buzz.playDingDong(5);
                #endif
            }
          return mytmp;
        }
    #endif
*/
// --- end of implementation
//
// --- templates
        // template websever
          #if (USE_WEBSERVER > OFF)
            #ifdef DUMMY
            void drawGraph()
            {
              String out = "";
              char temp[100];
              out += "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"400\" height=\"150\">\n";
              out += "<rect width=\"400\" height=\"150\" fill=\"rgb(250, 230, 210)\" stroke-width=\"1\" stroke=\"rgb(0, 0, 0)\" />\n";
              out += "<g stroke=\"black\">\n";
              int y = rand() % 130;
              for (int x = 10; x < 390; x += 10)
              {
                int y2 = rand() % 130;
                sprintf(temp, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke-width=\"1\" />\n", x, 140 - y, x + 10, 140 - y2);
                out += temp;
                y = y2;
              }
              out += "</g>\n</svg>\n";

              server.send(200, "image/svg+xml", out);
            }

            void handleRoot()
            {
              digitalWrite(led, 1);
              char temp[400];
              int sec = millis() / 1000;
              int min = sec / 60;
              int hr = min / 60;

              snprintf(temp, 400,

                "<html>\
                  <head>\
                    <meta http-equiv='refresh' content='5'/>\
                    <title>ESP32 Demo</title>\
                    <style>\
                      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
                    </style>\
                  </head>\
                  <body>\
                    <h1>Hello from ESP32!</h1>\
                    <p>Uptime: %02d:%02d:%02d</p>\
                    <img src=\"/test.svg\" />\
                  </body>\
                </html>",

                         hr, min % 60, sec % 60
                        );
              server.send(200, "text/html", temp);
              digitalWrite(led, 0);
            }

            void handleNotFound()
            {
              digitalWrite(led, 1);
              String message = "File Not Found\n\n";
              message += "URI: ";
              message += server.uri();
              message += "\nMethod: ";
              message += (server.method() == HTTP_GET) ? "GET" : "POST";
              message += "\nArguments: ";
              message += server.args();
              message += "\n";

              for (uint8_t i = 0; i < server.args(); i++)
              {
                message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
              }

              server.send(404, "text/plain", message);
              digitalWrite(led, 0);
            }
            #endif
          #endif // USE_WEBSERVER

        //
        // template touchscreen
#if (USE_TOUCHSCREEN > OFF)
  #ifdef DUMMY

  #include "FS.h"

  #include <SPI.h>
  #include <TFT_eSPI.h>      // Hardware-specific library

  TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

  // This is the file name used to store the calibration data
  // You can change this to create new calibration files.
  // The SPIFFS file name must start with "/".
  #define CALIBRATION_FILE "/TouchCalData1"

  // Set REPEAT_CAL to true instead of false to run calibration
  // again, otherwise it will only be done once.
  // Repeat calibration if you change the screen rotation.
  #define REPEAT_CAL false

  // Keypad start position, key sizes and spacing
  #define KEY_X 40 // Centre of key
  #define KEY_Y 96
  #define KEY_W 62 // Width and height
  #define KEY_H 30
  #define KEY_SPACING_X 18 // X and Y gap
  #define KEY_SPACING_Y 20
  #define KEY_TEXTSIZE 1   // Font size multiplier

  // Using two fonts since numbers are nice when bold
  #define LABEL1_FONT &FreeSansOblique12pt7b // Key label font 1
  #define LABEL2_FONT &FreeSansBold12pt7b    // Key label font 2

  // Numeric display box size and location
  #define DISP_X 1
  #define DISP_Y 10
  #define DISP_W 238
  #define DISP_H 50
  #define DISP_TSIZE 3
  #define DISP_TCOLOR TFT_CYAN

  // Number length, buffer for storing it and character index
  #define NUM_LEN 12
  char numberBuffer[NUM_LEN + 1] = "";
  uint8_t numberIndex = 0;

  // We have a status line for messages
  #define STATUS_X 120 // Centred on this
  #define STATUS_Y 65

  // Create 15 keys for the keypad
  char keyLabel[15][5] = {"New", "Del", "Send", "1", "2", "3", "4", "5", "6", "7", "8", "9", ".", "0", "#" };
  uint16_t keyColor[15] = {TFT_RED, TFT_DARKGREY, TFT_DARKGREEN,
                           TFT_BLUE, TFT_BLUE, TFT_BLUE,
                           TFT_BLUE, TFT_BLUE, TFT_BLUE,
                           TFT_BLUE, TFT_BLUE, TFT_BLUE,
                           TFT_BLUE, TFT_BLUE, TFT_BLUE
                          };

  // Invoke the TFT_eSPI button class and create all the button objects
  TFT_eSPI_Button key[15];

  //------------------------------------------------------------------------------------------

  void setup() {
    // Use serial port
    Serial.begin(9600);

    // Initialise the TFT screen
    tft.init();

    // Set the rotation before we calibrate
    tft.setRotation(0);

    // Calibrate the touch screen and retrieve the scaling factors
    touch_calibrate();

    // Clear the screen
    tft.fillScreen(TFT_BLACK);

    // Draw keypad background
    tft.fillRect(0, 0, 240, 320, TFT_DARKGREY);

    // Draw number display area and frame
    tft.fillRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_BLACK);
    tft.drawRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_WHITE);

    // Draw keypad
    drawKeypad();
  }

  //------------------------------------------------------------------------------------------

  void loop(void) {
    uint16_t t_x = 0, t_y = 0; // To store the touch coordinates

    // Pressed will be set true is there is a valid touch on the screen
    boolean pressed = tft.getTouch(&t_x, &t_y);

    // / Check if any key coordinate boxes contain the touch coordinates
    for (uint8_t b = 0; b < 15; b++) {
      if (pressed && key[b].contains(t_x, t_y)) {
        key[b].press(true);  // tell the button it is pressed
      } else {
        key[b].press(false);  // tell the button it is NOT pressed
      }
    }

    // Check if any key has changed state
    for (uint8_t b = 0; b < 15; b++) {

      if (b < 3) tft.setFreeFont(LABEL1_FONT);
      else tft.setFreeFont(LABEL2_FONT);

      if (key[b].justReleased()) key[b].drawButton();     // draw normal

      if (key[b].justPressed()) {
        key[b].drawButton(true);  // draw invert

        // if a numberpad button, append the relevant # to the numberBuffer
        if (b >= 3) {
          if (numberIndex < NUM_LEN) {
            numberBuffer[numberIndex] = keyLabel[b][0];
            numberIndex++;
            numberBuffer[numberIndex] = 0; // zero terminate
          }
          status(""); // Clear the old status
        }

        // Del button, so delete last char
        if (b == 1) {
          numberBuffer[numberIndex] = 0;
          if (numberIndex > 0) {
            numberIndex--;
            numberBuffer[numberIndex] = 0;//' ';
          }
          status(""); // Clear the old status
        }

        if (b == 2) {
          status("Sent value to serial port");
          Serial.println(numberBuffer);
        }
        // we dont really check that the text field makes sense
        // just try to call
        if (b == 0) {
          status("Value cleared");
          numberIndex = 0; // Reset index to 0
          numberBuffer[numberIndex] = 0; // Place null in buffer
        }

        // Update the number display field
        tft.setTextDatum(TL_DATUM);        // Use top left corner as text coord datum
        tft.setFreeFont(&FreeSans18pt7b);  // Choose a nicefont that fits box
        tft.setTextColor(DISP_TCOLOR);     // Set the font colour

        // Draw the string, the value returned is the width in pixels
        int xwidth = tft.drawString(numberBuffer, DISP_X + 4, DISP_Y + 12);

        // Now cover up the rest of the line up by drawing a black rectangle.  No flicker this way
        // but it will not work with italic or oblique fonts due to character overlap.
        tft.fillRect(DISP_X + 4 + xwidth, DISP_Y + 1, DISP_W - xwidth - 5, DISP_H - 2, TFT_BLACK);

        delay(10); // UI debouncing
      }
    }
  }


  #endif
  //------------------------------------------------------------------------------------------



#endif // USE_TOUCHSCREEN

