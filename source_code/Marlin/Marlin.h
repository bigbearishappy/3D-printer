#ifndef MARLIN_H
#define MARLIN_H

#include "stm32f10x.h"
#include "Configuration.h"
#include "Pin.h"
#include <stdio.h>
#include "string.h"					//add at 20160516
#include "stdlib.h"					//add at 20160516
#include <math.h>
#include "ADC.h"

#define true	1
#define false	0

#define lround(x)     			((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define round(x)     			((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define constrain(amt,low,high)	amt-low<0?low:amt;amt-high>0?high:amt;

#define READ(x)	GPIO_ReadInputDataBit(GPIOB,x)

#define  FORCE_INLINE __attribute__((always_inline))inline

//临界断面开始
#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  GPIO_SetBits(GPIOA,GPIO_Pin_2)//1//unsigned char _sreg = SREG; cli();//清零中断
  #define CRITICAL_SECTION_END    GPIO_SetBits(GPIOA,GPIO_Pin_2)//0//SREG = _sreg;
#endif //CRITICAL_SECTION_START

//DEFINATION OF SERIAL PRINT
#define SERIAL_PROTOCOL(x) printf("%d",x)//(MYSERIAL.print(x))
#define SERIAL_PROTOCOLF(x) printf("%.2f",x)//(MYSERIAL.print(x))
#define SERIAL_PROTOCOL_F(x,y) printf("%.2f",x)//(MYSERIAL.print(x,y))
#define SERIAL_PROTOCOLPGM(x) printf("%s",x)//(serialprintPGM(PSTR(x)))
#define SERIAL_PROTOCOLLN(x) printf("%d\r\n",x)//(MYSERIAL.print(x),MYSERIAL.write('\n'))
#define SERIAL_PROTOCOLLNPGM(x) printf("%s\r\n",x)//(serialprintPGM(PSTR(x)),MYSERIAL.write('\n'))

#define SERIAL_ERROR_START printf("\r\n")//(serialprintPGM(errormagic))
//#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) printf("%s\r\n",x)//SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHO_START printf("\r\n")//(serialprintPGM(echomagic))
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) printf("%s\r\n",x)//SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) printf("%s\r\n",x)//SERIAL_PROTOCOLLNPGM(x)




//打印开始和结束时间
extern unsigned long starttime;
extern unsigned long stoptime;

// Handling multiple extruders pins  处理多挤出机引脚
extern uint8_t active_extruder;

extern float add_homeing[3];

extern int extrudemultiply;
extern int fanSpeed;
extern uint8_t active_extruder;

enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};

void system_init(void);								//initialize the system

void setTargetHotend(const float celsius, uint8_t extruder);
void setTargetBed(const float celsius);	

void get_command(void);  							//get the GCODE from the USART
void FlushSerialRequestResend(void);				//
void ClearToSend(void);								//

void process_command(void);							//deal with the g_code
uint8_t code_seen(char code);
float code_value(void);
long code_value_long(void);
void get_coordinates(void);
void get_arc_coordinates(void);
void prepare_move(void);
void clamp_to_software_endstops(float target[3]);

int8_t next_block_index(int8_t block_index);

int setTargetedHotend(int code);
void manage_inactivity(void);
void checkHitEndstops(void);
void kill(void);
void suicide(void);

int fputc(int ch, FILE *f);							//it's related with the usart data transmission
int GetKey (void);

#if defined(X_ENABLE_PIN)// && (X_ENABLE_PIN > -1)
	#define  enable_x() GPIO_SetBits(GPIOA,GPIO_Pin_2)//1//WRITE(X_ENABLE_PIN, X_ENABLE_ON)
	#define disable_x() GPIO_ResetBits(GPIOA,GPIO_Pin_2)//0//{WRITE(X_ENABLE_PIN,!X_ENABLE_ON); axis_known_position[X_AXIS] = false; }
#endif
#if defined(Y_ENABLE_PIN)//&&Y_ENABLE_PIN > -1
	#define  enable_y() GPIO_SetBits(GPIOA,GPIO_Pin_2)//1//WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
	#define disable_y() GPIO_ResetBits(GPIOA,GPIO_Pin_2)//0//{ WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }
#endif
#if defined(Z_ENABLE_PIN)
	#define  enable_z() GPIO_SetBits(GPIOA,GPIO_Pin_2)//1//WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
	#define disable_z() GPIO_ResetBits(GPIOA,GPIO_Pin_2)//0//{ WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); axis_known_position[Z_AXIS] = false; }
#endif

#if defined(E0_ENABLE_PIN)// && (E0_ENABLE_PIN > -1)
  #define enable_e0()  GPIO_SetBits(GPIOA,GPIO_Pin_2)//1//WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e0() GPIO_ResetBits(GPIOA,GPIO_Pin_2)//0//WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)
#endif

#endif
