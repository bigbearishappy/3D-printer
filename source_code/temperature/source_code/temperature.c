#include "Marlin.h"
#include "temperature.h"
#include "thermistortable.h"
#include "BSP.h"
#include "Planner.h"

//public values
int target_temperature[EXTRUDERS] = { 0 };

int target_temperature_bed = 0;
float current_temperature_bed = 25.0;
int current_temperature_raw[EXTRUDERS] = { 0 };
int current_temperature_bed_raw = 0;
unsigned char soft_pwm_bed;

//private value
#ifdef PIDTEMP
volatile static float temp_iState_min[EXTRUDERS];//PID temp's min value
volatile static float temp_iState_max[EXTRUDERS];//PID temp's max value
#endif

static volatile int temp_meas_ready = false;

//static int maxttemp[EXTRUDERS] = {16383};
//static int minttemp[EXTRUDERS] = {0};

#ifdef PIDTEMP
  	float Kp=DEFAULT_Kp;
  	float Ki=(DEFAULT_Ki*PID_dT);
  	float Kd=(DEFAULT_Kd/PID_dT);
  	#ifdef PID_ADD_EXTRUSION_RATE
    	float Kc=DEFAULT_Kc;
  	#endif
#endif

unsigned char soft_pwm_bed;

float current_temperature[EXTRUDERS] = { 30.0 };
static unsigned char soft_pwm[EXTRUDERS];

#if EXTRUDERS > 3
  # error Unsupported number of extruders
#elif EXTRUDERS > 2
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2, v3 }
#elif EXTRUDERS > 1
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2 }
#else
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1 }
#endif

// Init min and max temp with extreme values to prevent false errors during startup
static int minttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP , HEATER_2_RAW_LO_TEMP );
static int maxttemp_raw[EXTRUDERS] = ARRAY_BY_EXTRUDERS( HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP , HEATER_2_RAW_HI_TEMP );
static int minttemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS( 0, 0, 0 );
static int maxttemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS( 16383, 16383, 16383 );

#ifdef BED_MAXTEMP
static int bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;
#endif

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
	int e;
	for(e = 0; e < EXTRUDERS; e++){
		maxttemp[e] = maxttemp[0];
#ifdef PIDTEMP
    temp_iState_min[e] = 0.0;
    temp_iState_max[e] = PID_INTEGRAL_DRIVE_MAX / Ki;
#endif //PIDTEMP
	}

	#if defined(HEATER_0_PIN) & (HEATER_0_PIN > -1) 
	//SET_OUTPUT(HEATER_0_PIN);
	//GPIO_SetBits();
#endif
#if defined(HEATER_BED_PIN)// && (HEATER_BED_PIN > -1) 
	//SET_OUTPUT(HEATER_BED_PIN);
#endif 
#if defined(FAN_PIN) //&& (FAN_PIN > -1) 
    //SET_OUTPUT(FAN_PIN);
#endif

// Set analog inputs
#if defined(TEMP_0_PIN)// && (TEMP_0_PIN > -1)
#endif

#if defined(TEMP_BED_PIN)// && (TEMP_BED_PIN > -1)
#endif

// Use timer0 for temperature measurement
// Interleave temperature interrupt with millies interrupt

// Wait for temperature measurement to settle
//delayms(250);

#ifdef HEATER_0_MINTEMP
	minttemp[0] = HEATER_0_MINTEMP;
	while(analog2temp(minttemp_raw[0], 0) < HEATER_0_MINTEMP) {
	#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
	    minttemp_raw[0] += OVERSAMPLENR;
	#else
	    minttemp_raw[0] -= OVERSAMPLENR;
	#endif
	}
#endif //MINTEMP

#ifdef HEATER_0_MAXTEMP
	maxttemp[0] = HEATER_0_MAXTEMP;
	while(analog2temp(maxttemp_raw[0], 0) > HEATER_0_MAXTEMP) {
	#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
	    maxttemp_raw[0] -= OVERSAMPLENR;
	#else
	    maxttemp_raw[0] += OVERSAMPLENR;
	#endif
  	}
#endif //MAXTEMP

#ifdef BED_MAXTEMP
  	while(analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
	#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
	    bed_maxttemp_raw -= OVERSAMPLENR;
	#else
	    bed_maxttemp_raw += OVERSAMPLENR;
	#endif
  	}
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
{
  float input = 0.0;
  int cycles=0;
  bool heating = true;

  //unsigned long temp_millis = millis();
  unsigned long temp_millis = tim_millis;
  unsigned long t1=temp_millis;
  unsigned long t2=temp_millis;
  long t_high = 0;
  long t_low = 0;

  long bias, d;
  float Ku, Tu;
  float Kp, Ki, Kd;
  float max = 0, min = 10000;

  if ((extruder > EXTRUDERS)
  #if (TEMP_BED_PIN <= -1)||(extruder < 0)
  #endif
       ){
          SERIAL_ECHOLN("PID Autotune failed. Bad extruder number.");
          return;
        }
	
  SERIAL_ECHOLN("PID Autotune start");
  
  disable_heater(); // switch off all heaters.

  if (extruder<0)
  {
     soft_pwm_bed = (MAX_BED_POWER)/2;
     bias = d = (MAX_BED_POWER)/2;
   }
   else
   {
     soft_pwm[extruder] = (PID_MAX)/2;
     bias = d = (PID_MAX)/2;
  }




 for(;;) {

    if(temp_meas_ready == true) { // temp sample ready
      updateTemperaturesFromRawValues();

      input = (extruder<0)?current_temperature_bed:current_temperature[extruder];//if the extruder number is less than 0,input = current_temperature_bed

      max=max(max,input);
      min=min(min,input);
      if(heating == true && input > temp) {
        //if(millis() - t2 > 5000) { 
		if(tim_millis - t2 > 5000) { 
          heating=false;
          if (extruder<0)
            soft_pwm_bed = (bias - d) >> 1;
          else
            soft_pwm[extruder] = (bias - d) >> 1;
          //t1=millis();
		  t1=tim_millis;
          t_high=t1 - t2;
          max=temp;
        }
      }
      if(heating == false && input < temp) {
        //if(millis() - t1 > 5000) {
		if(tim_millis - t1 > 5000) {
          heating=true;
          //t2=millis();
		  t2=tim_millis;
          t_low=t2 - t1;
          if(cycles > 0) {
            bias += (d*(t_high - t_low))/(t_low + t_high);
            bias = constrain(bias, 20 ,(extruder<0?(MAX_BED_POWER):(PID_MAX))-20);
            if(bias > (extruder<0?(MAX_BED_POWER):(PID_MAX))/2) 
				d = (extruder<0?(MAX_BED_POWER):(PID_MAX)) - 1 - bias;
            else 
				d = bias;

            SERIAL_PROTOCOLPGM(" bias: "); SERIAL_PROTOCOL(bias);
            SERIAL_PROTOCOLPGM(" d: "); SERIAL_PROTOCOL(d);
            SERIAL_PROTOCOLPGM(" min: "); SERIAL_PROTOCOL(min);
            SERIAL_PROTOCOLPGM(" max: "); SERIAL_PROTOCOLLN(max);
            if(cycles > 2) {
              Ku = (4.0*d)/(3.14159*(max-min)/2.0);
              Tu = ((float)(t_low + t_high)/1000.0);
              SERIAL_PROTOCOLPGM(" Ku: "); SERIAL_PROTOCOL(Ku);
              SERIAL_PROTOCOLPGM(" Tu: "); SERIAL_PROTOCOLLN(Tu);
              Kp = 0.6*Ku;
              Ki = 2*Kp/Tu;
              Kd = Kp*Tu/8;
              SERIAL_PROTOCOLLNPGM(" Clasic PID ");
              SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
              SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
              SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
              /*
              Kp = 0.33*Ku;
              Ki = Kp/Tu;
              Kd = Kp*Tu/3;
              SERIAL_PROTOCOLLNPGM(" Some overshoot ")
              SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
              SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
              SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
              Kp = 0.2*Ku;
              Ki = 2*Kp/Tu;
              Kd = Kp*Tu/3;
              SERIAL_PROTOCOLLNPGM(" No overshoot ")
              SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
              SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
              SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
              */
            }
          }
          if (extruder<0)
            soft_pwm_bed = (bias + d) >> 1;
          else
            soft_pwm[extruder] = (bias + d) >> 1;
          cycles++;
          min=temp;
        }
      } 
    }
    if(input > (temp + 20)) {
      SERIAL_PROTOCOLLNPGM("PID Autotune failed! Temperature too high");
      return;
    }
    //if(millis() - temp_millis > 2000) {
	if(tim_millis - temp_millis > 2000) {
      int p;
      if (extruder<0){
        p=soft_pwm_bed;       
        SERIAL_PROTOCOLPGM("ok B:");
      }else{
        p=soft_pwm[extruder];       
        SERIAL_PROTOCOLPGM("ok T:");
      }
			
      SERIAL_PROTOCOL(input);   
      SERIAL_PROTOCOLPGM(" @:");
      SERIAL_PROTOCOLLN(p);       

      //temp_millis = millis();
	  temp_millis = tim_millis;
    }
    //if(((millis() - t1) + (millis() - t2)) > (10L*60L*1000L*2L)) {
	if(((tim_millis - t1) + (tim_millis - t2)) > (10L*60L*1000L*2L)) {
      SERIAL_PROTOCOLLNPGM("PID Autotune failed! timeout");
      return;
    }
    if(cycles > ncycles) {
      SERIAL_PROTOCOLLNPGM("PID Autotune finished! Put the Kp, Ki and Kd constants into Configuration.h");
      return;
    }
    //lcd_update();
  }
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
{
  return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET;
}

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

int getHeaterPower(int heater) {
	if (heater<0)
		return soft_pwm_bed;
  return soft_pwm[heater];
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

float degBed(void)
{
	return current_temperature_bed;
}

float degTargetBed(void)
{
	return target_temperature_bed;
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
	volatile float pid_input;
	float pid_output;
	int e;

//comment for debug 20170412	
//	if(temp_meas_ready != true)   //better readability
//		return; 

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
#if 1
	uint16_t Status;
	uint16_t DataValue;
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	Status = ADC_GetSoftwareStartConvStatus(ADC1);
	DataValue = ADC_GetConversionValue(ADC1);
	current_temperature_raw[0] = DataValue;
#endif

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


