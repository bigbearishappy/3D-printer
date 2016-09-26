//这个文件里面涉及到的主要部分是对flash的操作还有显示到控制台的数据
#include"Marlin.h"
#include"ConfigurationStore.h"
#include"planner.h"
#include "temperature.h"

/****************************************************************************
name:		Config_RetrieveSettings
function:	
			the configuration before the printing
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
void Config_RetrieveSettings(void)
{
	Config_ResetDefault(); 
	Config_PrintSettings();
}

/****************************************************************************
name:		Config_ResetDefault
function:	
			reset the configuration to default
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
void Config_ResetDefault(void)
{
    float tmp1[4]=DEFAULT_AXIS_STEPS_PER_UNIT;
    float tmp2[4]=DEFAULT_MAX_FEEDRATE;
    long tmp3[4]=DEFAULT_MAX_ACCELERATION;
	short i = 0;
	for (i = 0;i < 4; i++) 
    {
        axis_steps_per_unit[i] = 					tmp1[i];  
        max_feedrate[i] =							tmp2[i];  
        max_acceleration_units_per_sq_second[i] =	tmp3[i];
    }
	reset_acceleration_rates();

    acceleration		= DEFAULT_ACCELERATION;
    retract_acceleration= DEFAULT_RETRACT_ACCELERATION;
    minimumfeedrate		= DEFAULT_MINIMUMFEEDRATE;
    minsegmenttime		= DEFAULT_MINSEGMENTTIME;       
    mintravelfeedrate	= DEFAULT_MINTRAVELFEEDRATE;
    max_xy_jerk			= DEFAULT_XYJERK;
    max_z_jerk			= DEFAULT_ZJERK;
    max_e_jerk			= DEFAULT_EJERK;

	add_homeing[0] = 0;
	add_homeing[1] = 0;
	add_homeing[2] = 0;
#ifdef ULTIPANEL
    plaPreheatHotendTemp 	= PLA_PREHEAT_HOTEND_TEMP;
    plaPreheatHPBTemp		= PLA_PREHEAT_HPB_TEMP;
    plaPreheatFanSpeed 		= PLA_PREHEAT_FAN_SPEED;
    absPreheatHotendTemp 	= ABS_PREHEAT_HOTEND_TEMP;
    absPreheatHPBTemp 		= ABS_PREHEAT_HPB_TEMP;
    absPreheatFanSpeed 		= ABS_PREHEAT_FAN_SPEED;
#endif
#ifdef PIDTEMP
    Kp = DEFAULT_Kp;
    Ki = scalePID_i(DEFAULT_Ki);
    Kd = scalePID_d(DEFAULT_Kd);
    
    // call updatePID (similar to when we have processed M301)
    updatePID();
    
#ifdef PID_ADD_EXTRUSION_RATE
    Kc = DEFAULT_Kc;
#endif//PID_ADD_EXTRUSION_RATE
#endif//PIDTEMP

printf("echo");
printf("ardcoded Default Settings Loaded\n");

}

/****************************************************************************
name:		Config_PrintSettings
function:	
			configurate the print
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
void Config_PrintSettings()
{}

/****************************************************************************
name:		reset_acceleration_rates
function:	
			initialize the value of the every axis's steps per second
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			公式意义：某个轴上一秒钟走的步数 = 某个轴上每秒最大加速度单元 * 某个轴上每个单位移动的步数
****************************************************************************/
void reset_acceleration_rates(void)
{
	uint8_t i;
	for(i = 0;i < NUM_AXIS; i++)
		axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
}
