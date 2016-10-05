#include "Marlin.h"
#include "temperature.h"

//public values
int target_temperature[EXTRUDERS] = { 0 };

int target_temperature_bed = 0;
float current_temperature_bed = 0.0;
int current_temperature_raw[EXTRUDERS] = { 0 };
int current_temperature_bed_raw = 0;
unsigned char soft_pwm_bed;

//private value
#ifdef PIDTEMP
static float temp_iState_min[EXTRUDERS];
static float temp_iState_max[EXTRUDERS];
#endif

static volatile int temp_meas_ready = false;

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
static unsigned char soft_pwm[EXTRUDERS];

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

int isHeatingHotend(uint8_t extruder){  
  return target_temperature[extruder] > current_temperature[extruder];
}

/*******************************************************************************
name:		PID_autotune()
funtion:	use PID to control the extruder's temperature
			[in]	-	temp:		input temperature
						extruder:	current extruder
						ncycles:	option
			[out]	-	void
*******************************************************************************/
void PID_autotune(float temp, int extruder, int ncycles)
{}

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

float degTargetHotend(uint8_t extruder) {  
  return target_temperature[extruder];
}

int isHeatingBed() {
  return target_temperature_bed > current_temperature_bed;
}

int isCoolingBed() {
  return target_temperature_bed < current_temperature_bed;
}

/**************************************************************************************
name:		disable_heater()
function:	disable the heater
			[in]	-	void
			[out]	-	void
**************************************************************************************/
void disable_heater(void)
{}

/***********************************************************************************************
name:		manage_heater()
function:	use the pid to control the temp of the heater
			[in]	-	void
			[out]	-	void
***********************************************************************************************/
void manage_heater()
{
	float pid_input;
	float pid_output;
	int e;
	
	if(temp_meas_ready != true)   //better readability
		return; 

	updateTemperaturesFromRawValues();

	for(e = 0; e < EXTRUDERS; e++)
	{

  #ifdef PIDTEMP
    pid_input = current_temperature[e];

    pid_output = constrain(target_temperature[e], 0, PID_MAX);//constrain(x, a, b);将x的值约束到a和b之间

  #else /* PID off */
    pid_output = 0;
    if(current_temperature[e] < target_temperature[e]) {
      pid_output = PID_MAX;
    }
  #endif

    // Check if temperature is within the correct range
    if((current_temperature[e] > minttemp[e]) && (current_temperature[e] < maxttemp[e])) 
    {
      soft_pwm[e] = (int)pid_output >> 1;
    }
    else {
      soft_pwm[e] = 0;
    }
  	}

	#if (defined(EXTRUDER_0_AUTO_FAN_PIN) && EXTRUDER_0_AUTO_FAN_PIN > -1) || \
	  (defined(EXTRUDER_1_AUTO_FAN_PIN) && EXTRUDER_1_AUTO_FAN_PIN > -1) || \
	  (defined(EXTRUDER_2_AUTO_FAN_PIN) && EXTRUDER_2_AUTO_FAN_PIN > -1)
	if(millis() - extruder_autofan_last_check > 2500)  // only need to check fan state very infrequently
	{
	checkExtruderAutoFans();
	extruder_autofan_last_check = millis();
	}  
	#endif
	
	#if TEMP_SENSOR_BED != 0
      // Check if temperature is within the correct band
      if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP))
      {
        if(current_temperature_bed > target_temperature_bed + BED_HYSTERESIS)
        {
          soft_pwm_bed = 0;
        }
        else if(current_temperature_bed <= target_temperature_bed - BED_HYSTERESIS)
        {
          soft_pwm_bed = MAX_BED_POWER>>1;
        }
      }
      else
      {
        soft_pwm_bed = 0;
        //WRITE(HEATER_BED_PIN,LOW);
      }
	#endif  
}

/* Called to get the raw values into the the actual temperatures. The raw values are created in interrupt context,
    and this function is called from normal context as it is too slow to run in interrupts and will block the stepper routine otherwise */
/******************************************************************************************************
name:		updateTemperaturesFromRawValues()
function:	update the temperatrue from the rawvalues
			[in]	-	void
			[out]	-	void
******************************************************************************************************/
static void updateTemperaturesFromRawValues()
{
	uint8_t e;
    for(e=0;e<EXTRUDERS;e++)
    {
        current_temperature[e] = analog2temp(current_temperature_raw[e], e);
    }
    current_temperature_bed = analog2tempBed(current_temperature_bed_raw);

    //Reset the watchdog after we know we have a temperature measurement.
    //watchdog_reset();

    CRITICAL_SECTION_START;//临界区
    temp_meas_ready = false;
    CRITICAL_SECTION_END;
}


