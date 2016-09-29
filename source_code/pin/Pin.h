#ifndef PINS_H
#define PINS_H

#define X_MS1_PIN -1
#define X_MS2_PIN -1
#define Y_MS1_PIN -1
#define Y_MS2_PIN -1
#define Z_MS1_PIN -1
#define Z_MS2_PIN -1
#define E0_MS1_PIN -1
#define E0_MS2_PIN -1
#define E1_MS1_PIN -1
#define E1_MS2_PIN -1
#define DIGIPOTSS_PIN -1

#if MOTHERBOARD == 33
#define KNOWN_BOARD 1

	#if MOTHERBOARD == 33
    #define X_STEP_PIN         54
    #define X_DIR_PIN          55
    #define X_ENABLE_PIN       38
    #define X_MIN_PIN           3
    #define X_MAX_PIN           2

    #define Y_STEP_PIN         60
    #define Y_DIR_PIN          61
    #define Y_ENABLE_PIN       56
    #define Y_MIN_PIN          14
    #define Y_MAX_PIN          15

    #define Z_STEP_PIN         46
    #define Z_DIR_PIN          48
    #define Z_ENABLE_PIN       62
    #define Z_MIN_PIN          18
    #define Z_MAX_PIN          19

    #define Y2_STEP_PIN        //36
    #define Y2_DIR_PIN         //34
    #define Y2_ENABLE_PIN      //30

    #define Z2_STEP_PIN        //36
    #define Z2_DIR_PIN         //34
    #define Z2_ENABLE_PIN      //30

    #define E0_STEP_PIN        26
    #define E0_DIR_PIN         28
    #define E0_ENABLE_PIN      24

    #define E1_STEP_PIN        //36
    #define E1_DIR_PIN         //34
    #define E1_ENABLE_PIN      //30

    #define SDPOWER            //-1
    #define SDSS               //53
    #define LED_PIN            //13

	#define FAN_PIN            9 // (Sprinter config)
	#define HEATER_0_PIN       -1

	#define TEMP_0_PIN          0    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
	#define TEMP_1_PIN          -1
	#define TEMP_2_PIN          -1
	#define HEATER_BED_PIN      -1
	#define TEMP_BED_PIN        0//-1

// M240  Triggers a camera by emulating a Canon RC-1 Remote
// Data from: http://www.doc-diy.net/photo/rc-1_hacked/
	#define PHOTOGRAPH_PIN     29
	#define MOTOR_CURRENT_PWM_XY_PIN 44
	#define MOTOR_CURRENT_PWM_Z_PIN 45
	#define MOTOR_CURRENT_PWM_E_PIN 46
	#endif

#endif


#endif
