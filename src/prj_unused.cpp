#ifdef UNUSED
#include "main.h"
//#include <md_defines.h>
//#include <project.h>
//#include <prj_config.h>

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
              statOut[OLED1_MAXCOLS] = 0;  // limit strlen
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
                oled1.begin((uint8_t) OLED1_MAXCOLS, (uint8_t) OLED1_MAXROWS);
              #endif

            #if defined (OLED2)
                oled2.begin((uint8_t) DISP2_MAXCOLS, (uint8_t) DISP2_MAXROWS);
              #endif
          #endif
      }

/*
  #if (USE_MQ135_GAS_ADC > OFF)
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
#endif // UNUSED
