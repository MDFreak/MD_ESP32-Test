#ifndef _PRJ_CONFIG_H_
  #define _PRJ_CONFIG_H_

  #include <Arduino.h>
  #include <md_defines.h>
  //#include <project.h>

  // ******************************************
    // --- test features --------------------------------
      #define TEST_SOCKET_SERVER    OFF
      #define TEST_RGBLED_PWM       OFF
      #define USE_WEBCTRL_RGB       OFF
      #define USE_WEBCTRL_FAN       OFF
      #define USE_POTICTRL_RGB      OFF
      #define USE_POTICTRL_FAN      OFF
      #define USE_SWCTRL_RGB        OFF
      #define USE_SWCTRL_FAN        OFF
      #define USE_SWCTRL_1812       OFF
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

  // ******************************************

    #if (PROJECT == PRJ_TOUCHTEST_1)
        #include <prj_conf_touchtest_1.h>
      #endif
    #if (PROJECT == PRJ_MEASFREQ_1)
        #include <prj_conf_measfreq_1.h>
      #endif
    #if (PROJECT == PRJ_LIGHTSHOW_1)
        #include <prj_conf_lightshow_1.h>
      #endif
    #if (PROJECT == PRJ_GEN_ESP32_NODE)
        #define PROJ_TITLE "Generic ESP32-Node"
        #include <prj_conf_gen_esp32_node.h>
      #endif
            //#if (PROJECT == GEN_XIAO_ESP32C3)
            //    #include <prj_conf_gen_xiao_esp32c3.h>
            //  #endif
    #if (PROJECT == PRJ_GEN_ESP32_D1_MINI)
        #include <prj_conf_gen_esp32_d1_mini.h>
      #endif

#endif // _PRJ_CONFIG_H_
