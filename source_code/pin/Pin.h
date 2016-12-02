#ifndef PINS_H
#define PINS_H

#include"stm32f10x.h"

#define X_MS1_PIN 40
#define X_MS2_PIN -1
#define Y_MS1_PIN -1
#define Y_MS2_PIN -1
#define Z_MS1_PIN -1
#define Z_MS2_PIN -1
#define E0_MS1_PIN -1
#define E0_MS2_PIN -1
#define E1_MS1_PIN -1
#define E1_MS2_PIN -1
#define DIGIPOTSS_PIN 38

#if MOTHERBOARD == 33
#define KNOWN_BOARD 1

	#if MOTHERBOARD == 33
    #define X_STEP_PIN         	0x0010//GPIO_Pin_4
    #define X_DIR_PIN          	0x0020//GPIO_Pin_5
    #define X_ENABLE_PIN      	0x0008//GPIO_Pin_3
    #define X_MIN_PIN           0x0400//GPIO_Pin_10
    #define X_MAX_PIN           2

    #define Y_STEP_PIN         0x0080//GPIO_Pin_7
    #define Y_DIR_PIN          0x0001//GPIO_Pin_0
    #define Y_ENABLE_PIN       0x0040//GPIO_Pin_6
    #define Y_MIN_PIN          0x0800//GPIO_Pin_11
    #define Y_MAX_PIN          15

    #define Z_STEP_PIN         0x0020//GPIO_Pin_5
    #define Z_DIR_PIN          0x0040//GPIO_Pin_6
    #define Z_ENABLE_PIN       0x0002//GPIO_Pin_1
    #define Z_MIN_PIN          0x1000//GPIO_Pin_12
    #define Z_MAX_PIN          19

    #define Y2_STEP_PIN        -1
    #define Y2_DIR_PIN         -1
    #define Y2_ENABLE_PIN      -1

    #define Z2_STEP_PIN        -1
    #define Z2_DIR_PIN         -1
    #define Z2_ENABLE_PIN      -1

    #define E0_STEP_PIN        0x0800//GPIO_Pin_11
    #define E0_DIR_PIN         0x1000//GPIO_Pin_12
    #define E0_ENABLE_PIN      0x0100//GPIO_Pin_8

    #define E1_STEP_PIN        -1
    #define E1_DIR_PIN         -1
    #define E1_ENABLE_PIN      -1

    #define SDPOWER            -1
    #define SDSS               -1
    #define LED_PIN            0x4000//GPIO_Pin_14

	#define FAN_PIN            0x2000//GPIO_Pin_13 // (Sprinter config)
	#define HEATER_0_PIN       0x0100//GPIO_Pin_8

	#define TEMP_0_PIN          0x0002//GPIO_Pin_1    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
	#define TEMP_1_PIN          -1
	#define TEMP_2_PIN          -1
	#define HEATER_BED_PIN      0x0200//GPIO_Pin_9
	#define TEMP_BED_PIN        0x0004//GPIO_Pin_2//-1

// M240  Triggers a camera by emulating a Canon RC-1 Remote
// Data from: http://www.doc-diy.net/photo/rc-1_hacked/
	#define PHOTOGRAPH_PIN     29
	#define MOTOR_CURRENT_PWM_XY_PIN 44
	#define MOTOR_CURRENT_PWM_Z_PIN 45
	#define MOTOR_CURRENT_PWM_E_PIN 46

	#define PS_ON_PIN          12
	#define SUICIDE_PIN        54  //PIN that has to be turned on right after start, to keep power flowing.
	#define KILL_PIN           -1

	#define DEFAULT_PWM_MOTOR_CURRENT  {1300, 1300, 1250}

	#endif

#endif

void PIN_Configuration(void);


#endif
