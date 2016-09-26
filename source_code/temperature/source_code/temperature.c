#include "Marlin.h"
#include "temperature.h"

//private value
#ifdef PIDTEMP
static float temp_iState_min[EXTRUDERS];
static float temp_iState_max[EXTRUDERS];
#endif

static int maxttemp[EXTRUDERS] = {16383};
static int minttemp[EXTRUDERS] = {0};

#ifdef PIDTEMP
  	float Kp=DEFAULT_Kp;
  	float Ki=(DEFAULT_Ki*PID_dT);
  	float Kd=(DEFAULT_Kd/PID_dT);
#endif

#ifdef PIDTEMP
	static float temp_iState_max[EXTRUDERS];
#endif

float current_temperature[EXTRUDERS] = { 0.0 };

/****************************************************************************
name:		tp_init
function:	
			initialize the configuration of the temperature control
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
void tp_init()
{
	uint8_t e;
	for(e = 0; e < EXTRUDERS; e++)
		maxttemp[e] = maxttemp[0];
#ifdef PIDTEMP
    temp_iState_min[e] = 0.0;
    temp_iState_max[e] = PID_INTEGRAL_DRIVE_MAX / Ki;
#endif //PIDTEMP
#if defined(HEATER_0_PIN) && (HEATER_0_PIN > -1) 
	//SET_OUTPUT(HEATER_0_PIN);
	//GPIO_SetBits();
#endif
#if defined(FAN_PIN) && (FAN_PIN > -1) 
    //SET_OUTPUT(FAN_PIN);
#endif

// Set analog inputs
#if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
#endif

#if defined(TEMP_1_PIN) && (TEMP_1_PIN > -1)
#endif

#if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1)
#endif

#if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
#endif

// Wait for temperature measurement to settle
//delayms(250);

#ifdef HEATER_0_MINTEMP

#endif //MINTEMP
#ifdef HEATER_0_MAXTEMP

#endif //MAXTEMP

#ifdef BED_MAXTEMP

#endif //BED_MAXTEMP
  
}

/****************************************************************************
name:		analog2temp
function:	
			transform the analog data to the temperature data
Parameters:
			[in]	-	int raw:the analog data
			[in]	-	uint8_t e:the extruder
Returns:
			[out]	-	float:the temperature data
Description:
			null
****************************************************************************/
static float analog2temp(int raw, uint8_t e)
{return 0;}

/****************************************************************************
name:		analog2tempBed
function:	
			transform the bed's analog data to the temperature data
Parameters:
			[in]	-	int raw:the analog data of the bed
Returns:
			[out]	-	float:the temperature data
Description:
			null
****************************************************************************/
static float analog2tempBed(int raw)
{return 0;}

/****************************************************************************
name:		updatePID
function:	
			main
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
void updatePID()
{
#ifdef PIDTEMP
	int e;
	for(e = 0; e < EXTRUDERS; e++) { 
    	temp_iState_max[e] = PID_INTEGRAL_DRIVE_MAX / Ki;  
  	}
#endif
}


float scalePID_i(float i)
{
	return i * PID_dT;
}

float unscalePID_i(float i)
{
	return i/PID_dT;
}

float scalePID_d(float d)
{
    return d/PID_dT;
}

float unscalePID_d(float d)
{
	return d*PID_dT;
}

float degHotend(uint8_t extruder) 
{  
  return current_temperature[extruder];
}
