#ifndef _PROJECT_H_
  #define _PROJECT_H_

  #include <Arduino.h>
  #include <md_defines.h>

  // ******************************************
  // --- project configuration
    // --- projects
      #define PRJ_TOUCHTEST_1        1
      #define PRJ_MEASFREQ_1         2
      #define PRJ_LIGHTSHOW_1        3
      #define PRJ_GEN_ESP32_NODE     4
      #define PRJ_GEN_ESP32_D1_MINI  5
      #define PRJ_GEN_ESP32_D1_R32   6

      //#define PROJECT PRJ_TOUCHTEST_1
      //#define PROJECT PRJ_MEASFREQ_1
      //#define PROJECT PRJ_LIGHTSHOW_1
      //#define PROJECT PRJ_GEN_ESP32_NODE
      //#define PROJECT PRJ_GEN_ESP32_D1_MINI
      //#define PROJECT PRJ_GEN_ESP32_D1_R32

      #if (PROJECT == PRJ_TOUCHTEST_1)
          #define PROJ_TITLE "Test Toucscreen"
          //#define BOARD   MC_ESP32_Node     // platform=espressiv32, env=env:esp32dev, az-delivery-devkit-v4
        #endif
      #if (PROJECT == PRJ_MEASFREQ_1)
          #define PROJ_TITLE "Measure Frequency "
          #define BOARD   MC_ESP32_Node     // platform=espressiv32, env=env:esp32dev, az-delivery-devkit-v4
        #endif
      #if (PROJECT == PRJ_LIGHTSHOW_1)
          #define PROJ_TITLE "Light Show"
          #define BOARD   MC_ESP32_Node     // platform=espressiv32, env=env:esp32dev, az-delivery-devkit-v4
        #endif
      #if (PROJECT == PRJ_GEN_ESP32_NODE)
          #define PROJ_TITLE "Generic ESP32-Node"
          #define BOARD   MC_ESP32_Node     // platform=espressiv32, env=env:esp32dev, az-delivery-devkit-v4
        #endif
      #if (PROJECT == GEN_XIAO_ESP32C3)
          #define PROJ_TITLE "Generic XIAO ESP32C3"
          #define BOARD   XIAO_ESP32C3     // platform=espressiv32, env=env:esp32dev, az-delivery-devkit-v4
        #endif
      #if (PROJECT == PRJ_GEN_ESP32_D1_MINI)
          #define PROJ_TITLE "Generic ESP32-D1-Mini"
          #define BOARD   MC_ESP32_Node     // platform=espressiv32, env=env:esp32dev, az-delivery-devkit-v4
        #endif
      #if (PROJECT == PRJ_GEN_ESP32_D1_R32)
          #define PROJ_TITLE "Generic ESP32-D1-R32"
          #define BOARD   MC_ESP32_D1_R32     // platform=espressiv32, env=env:esp32dev, az-delivery-devkit-v4
        #endif
#endif // _PRJ_CONFIG_H_