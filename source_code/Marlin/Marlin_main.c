#include"Marlin.h"

#include"ConfigurationStore.h"
#include"temperature.h"
#include"stepper.h"
#include"planner.h"
#include"language.h"
#include"BSP.h"

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G10 - retract filament according to settings of M207
// G11 - retract recover filament according to settings of M208
// G28 - Home all Axis
// G29 - Detailed Z-Probe, probes the bed at 3 points.  You must de at the home position for this to work correctly.
// G30 - Single Z Probe, probes bed at current XY location.
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

// M Codes
// M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
// M1   - Same as M0
// M17  - Enable/Power all stepper motors
// M18  - Disable all stepper motors; same as M84
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M30  - Delete file from SD (M30 filename.g)
// M31  - Output time since last M109 or SD card start to serial
// M32  - Select file and start SD print (Can be used _while_ printing from SD card files):
//        syntax "M32 /path/filename#", or "M32 S<startpos bytes> !filename#"
//        Call gcode file : "M32 P !filename#" and return to caller file after finishing (simiarl to #include).
//        The '#' is necessary when calling from within sd files, as it stops buffer prereading
// M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move,
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Sxxx Wait for extruder current temp to reach target temp. Waits only when heating
//        Rxxx Wait for extruder current temp to reach target temp. Waits when heating and cooling
// M114 - Output current position to serial port
// M115 - Capabilities string
// M117 - display message
// M119 - Output Endstop status to serial port
// M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
// M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
// M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M140 - Set bed target temp
// M150 - Set BlinkM Colour Output R: Red<0-255> U(!): Green<0-255> B: Blue<0-255> over i2c, G for green does not work.
// M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating
//        Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
// M200 - Set filament diameter
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
// M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) im mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer underruns and M20 minimum feedrate
// M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
// M206 - set additional homeing offset
// M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
// M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
// M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
// M218 - set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
// M220 S<factor in percent>- set speed factor override percentage
// M221 S<factor in percent>- set extrude factor override percentage
// M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
// M240 - Trigger a camera to take a photograph
// M250 - Set LCD contrast C<contrast value> (value 0..63)
// M280 - set servo position absolute. P: servo index, S: angle or microseconds
// M300 - Play beepsound S<frequency Hz> P<duration ms>
// M301 - Set PID parameters P I and D
// M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
// M304 - Set bed PID parameters P I and D
// M400 - Finish all moves
// M401 - Lower z-probe if present
// M402 - Raise z-probe if present
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
// M503 - print the current settings (from memory not from eeprom)
// M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
// M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
// M666 - set delta endstop adjustemnt
// M605 - Set dual x-carriage movement mode: S<mode> [ X<duplication x-offset> R<duplication temp offset> ]
// M907 - Set digital trimpot motor current using axis codes.
// M908 - Control digital trimpot directly.
// M350 - Set microstepping mode.
// M351 - Toggle MS1 MS2 pins directly.
// M928 - Start SD logging (M928 filename.g) - ended by M29
// M999 - Restart after being stopped by error

//Stepper Movement Variables

#define VERSION_STRING  "1.0.0"

#ifdef ULTIPANEL
  #ifdef PS_DEFAULT_OFF
    int powersupply = false;
  #else
	int powersupply = true;
  #endif
#endif

#if 1
static char connect_flag = 0;
static int buflen = 0;
static char serial_char;
static uint32_t serial_count = 0;

static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
volatile static uint8_t fromsd[BUFSIZE];
static int bufindw = 0;
static int bufindr = 0;

static long gcode_N, gcode_LastN,Stopped_gcode_LastN = 0;
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc
uint8_t Stopped = false;
static unsigned long previous_millis_cmd = 0;

const int sensitive_pins[] = {0};//SENSITIVE_PINS; // Sensitive pin list for M42

//public value
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
int extrudemultiply=100; //100->1 200->2
float add_homeing[3]={0,0,0};
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };

int fanSpeed=0;
uint8_t active_extruder = 0;

static float feedrate = 1500.0, next_feedrate, saved_feedrate;

//private value
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
uint8_t axis_relative_modes[] = AXIS_RELATIVE_MODES;
static uint8_t relative_mode = false;  //Determines Absolute or Relative Coordinates
volatile static float offset[3] = {0.0, 0.0, 0.0};
static uint8_t home_all_axis = true;
uint8_t axis_known_position[3] = {false, false, false};

static unsigned long max_inactive_time = 0;

unsigned long starttime = 0;
unsigned long stoptime = 0;

static uint8_t tmp_extruder;
int CooldownNoWait = true;
int target_direction;

static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

int8_t i = 0;

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const PROGMEM type array##_P[3] =        \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(int axis)          \
    { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,    MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,   HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,      MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);


#endif

/****************************************************************************
name:		main
function:	
			main
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
void SystemInit(void){}
int main()
{
	system_init();
	for(;;)
	{
		//printf("123");
		if(buflen < (BUFSIZE - 1))
			get_command();
		if(buflen)
		{
			process_command();
			buflen -= 1;
			bufindr = (bufindr + 1)%BUFSIZE;
		}
		manage_heater();
		manage_inactivity();
		checkHitEndstops();/**/		 
	}
}


//function code**********************************************************************************************************
/****************************************************************************
name:		system_init
function:	
			initialize the system before printing
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
void system_init(void)
{
	//int8_t i;
	RCC_Configuration();
	NVIC_Configuration();
	TIM_Configuration();
	USART_Configuration();
	ADC_Configuration();

	printf("%d\n",BAUDRATE);
	//printf("start\n");
	SERIAL_PROTOCOLLNPGM("start");

//这里有很多开机时的打印信息，留在以后专门整理20160412
	SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_ECHOLNPGM(VERSION_STRING);
  #ifdef STRING_VERSION_CONFIG_H
    #ifdef STRING_CONFIG_H_AUTHOR
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
      SERIAL_ECHOPGM(STRING_VERSION_CONFIG_H);
      SERIAL_ECHOPGM(MSG_AUTHOR);
      SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
      SERIAL_ECHOPGM("Compiled: ");
      SERIAL_ECHOLNPGM(__DATE__);
    #endif
  #endif
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  //SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);


  for(i = 0; i < BUFSIZE; i++)
  {
    fromsd[i] = false;
  }
  
#if 1
  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();

  tp_init();				// Initialize temperature loop
  plan_init();				// Initialize planner;
  st_init();				// Initialize stepper, this enables interrupts!
  //servo_init();
  //delay(1000);
#endif
}

/*****************************************************************************************
name:		setTargetHotend()
function:	set a extruder to a degree
			[in]	-	celsius:target degree
						extruder:the target extruder
			[out]	-	void
*****************************************************************************************/
#if 1
void setTargetHotend(const float celsius, uint8_t extruder) {  
  target_temperature[extruder] = celsius;
}

/***************************************************************************************
name:		setTargetBed()
function:	set the bed to a degree
			[in]	-	celsius:target degree
			[out]	-	void
***************************************************************************************/
void setTargetBed(const float celsius) {  
  target_temperature_bed = celsius;
}

/****************************************************************************
name:		get_command
function:	
			get command from the USART
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/

void get_command()
{
	uint8_t i = 0;
	char buf[4] = "M110";
	const char *p;
	char checksum;//byte data can be + or -,but char just can be +
	char count;
	//while(USART1_DATA_OK == READY)
	while(!Queueisempty(&queue))
	{
		//serial_char = USART1_Cache[i++];
		serial_char = Queuegetc(&queue);
		if(serial_char == '\n' || serial_char == '\r'||(serial_char == ':' && comment_mode == false)||serial_count >= (MAX_CMD_SIZE - 1))
		{
			if(!serial_count){						//if empty line
				comment_mode = false;				//for new line
				return;
				}
			cmdbuffer[bufindw][serial_count] = 0;	//terminate string
			if(!comment_mode){
				comment_mode = false;				//for new command
				fromsd[bufindw] = false;
				if(strchr(cmdbuffer[bufindw], 'N') != NULL){
					strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
					gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
					p = buf;
					if(gcode_N != gcode_LastN+1 && (strstr(cmdbuffer[bufindw], p) == NULL) ) {	//strstr_P function:find out if PSTR("M110") is in the cmdbuffer[bufindew]
          																								//if so,return the pointer that the PSTR("M110") first exist in cmdbuffer[bufindw],or return NULL.
						printf(MSG_ERR_LINE_NO);
						printf("%d",gcode_LastN);
            			FlushSerialRequestResend();
            			serial_count = 0;
            			return;
          				}  
						//if no errors, continue parsing
					if(strchr(cmdbuffer[bufindw], '*') != NULL)
					{
					checksum = 0;//byte data can be + or -,but char just can be +
					count = 0;
					while(cmdbuffer[bufindw][count] != '*') 
						checksum = checksum^cmdbuffer[bufindw][count++];//^ function:XOR
					
					strchr_pointer = strchr(cmdbuffer[bufindw], '*');
					
					if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {//strtod:translate string to float number
					  SERIAL_ERROR_START;
					  SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
					  SERIAL_ERRORLN(gcode_LastN);
					  FlushSerialRequestResend();
					  serial_count = 0;
					  return;
					}
					//if no errors, continue parsing
					}else{
            			printf(MSG_ERR_NO_CHECKSUM);
            			printf("%d",gcode_LastN);
            			FlushSerialRequestResend();
            			serial_count = 0;
            			//return;
          			}
					gcode_LastN = gcode_N;					
          			//if no errors, continue parsing
				}					
				else  // if we don't receive 'N' but still see '*'
        		{
          			if((strchr(cmdbuffer[bufindw], '*') != NULL))
          			{
            			printf(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
            			printf("%d\r\n",gcode_LastN);
            			serial_count = 0;
            			//return;
          			}
        		}
				if((strchr(cmdbuffer[bufindw], 'G') != NULL)){
          			strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
          			switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))){
          				case 0:
          				case 1:
          				case 2:
          				case 3:
            			if(Stopped == false) { // If printer is stopped by an error the G[0-3] codes are ignored.
              				printf(MSG_OK); printf("\r\n");
            			}
            			else {
             				printf(MSG_ERR_STOPPED);printf("\r\n");
            			}
           					break;
          				default:
            				break;
          			}
				}
				bufindw = (bufindw + 1)%BUFSIZE;
        		buflen += 1;
			}
			serial_count = 0; //clear buffer
		} 
		else
    	{
      	if(serial_char == ';') 
	  		comment_mode = true;
      	if(!comment_mode) 
	  		cmdbuffer[bufindw][serial_count++] = serial_char;
    	}
	}
}
#endif

/****************************************************************************
name:		FlushSerialRequestResend
function:	
			flush the serial port and request to resend the data
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
#if 1
void FlushSerialRequestResend(void)
{
  printf(MSG_RESEND);
  printf("%d\r\n",gcode_LastN + 1);
  ClearToSend();
}


/****************************************************************************
name:		ClearToSend
function:	
			count the system time and printf "ok"
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
void ClearToSend(void)
{
  previous_millis_cmd = tim_millis;//millis();
  printf(MSG_OK);printf("\r\n");
}
#endif

/****************************************************************************
name:		homeaxis
function:	
			make the specify axis to move home
Parameters:
			[in]	-	X or Y or Z axis
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
static void homeaxis(int axis) {
#define HOMEAXIS_DO(LETTER) \
  ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

  if (axis==X_AXIS ? HOMEAXIS_DO(X) : axis==Y_AXIS ? HOMEAXIS_DO(Y) : axis==Z_AXIS ? HOMEAXIS_DO(Z) : 0) {//if axis == X_AXIS,HOMEAXIS_DO(X).if axis == Y_AXIS,HOMEAXIS_DO(Y).if axis == Z_AXIS,HOMEAXIS_DO(Z).
    int axis_home_dir = home_dir(axis);
#ifdef DUAL_X_CARRIAGE
    if (axis == X_AXIS)
      axis_home_dir = x_home_dir(active_extruder);
#endif

    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);


    // Engage Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
      #if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
        if (axis==Z_AXIS) {
          engage_z_probe();
        }
	    else
      #endif
      if (servo_endstops[axis] > -1) {
        servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2]);
      }
    #endif

    destination[axis] = 1.5 * max_length(axis) * axis_home_dir;
    feedrate = homing_feedrate[axis];
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = -home_retract_mm(axis) * axis_home_dir;
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    destination[axis] = 2 * home_retract_mm(axis) * axis_home_dir;
#ifdef DELTA
    feedrate = homing_feedrate[axis]/10;
#else
    feedrate = homing_feedrate[axis]/2 ;
#endif
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();
#ifdef DELTA
    // retrace by the amount specified in endstop_adj
    if (endstop_adj[axis] * axis_home_dir < 0) {
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      destination[axis] = endstop_adj[axis];
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
      st_synchronize();
    }
#endif
    axis_is_at_home(axis);
    destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose();
    axis_known_position[axis] = true;

    // Retract Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
      if (servo_endstops[axis] > -1) {
        servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2 + 1]);
      }
    #endif
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
    if (axis==Z_AXIS) retract_z_probe();
#endif

  }
}

#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)


/****************************************************************************
name:		process_command
function:	
			deal with the g_code from the usart
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
#if 1
void process_command(void)
{
	unsigned long codenum; //throw away variable
	char *starpos = NULL;
	int8_t i;
	volatile char time[30];
    unsigned long t = 0;
    volatile int sec,min;
    int pin_status;
    int pin_number;
	int8_t cur_extruder;
	long residencyStart;
	volatile float tt;
	int all_axis;
	float value;
	float factor;
    int pin_state;	// required pin state - default is inverted
	volatile int target;
	const uint8_t NUM_PULSES = 16;
	//const float PULSE_LENGTH = 0.01524;
	float temp;
	int e;
    int c;
	float target1[4];
    float lastpos[4];
	volatile uint8_t cnt;
	volatile uint8_t channel,current;
	volatile int make_move;
	volatile int temp1 = 0,temp2 = 0;

	uint32_t current_time = 0;

	if(code_seen('G'))
	{
	#if defined(DEBUG)
		printf("G%d\r\n",(int)code_value());
	#endif
		switch((int)code_value())
    	{
    		case 0: 									// G0 -> G1							//fast move
    		case 1: 									// G1								//fast move
			if(Stopped == false) {
        	get_coordinates(); // For X Y Z E F
        	prepare_move();
        	return;
      		}
    		case 2: 									// G2  - CW ARC
      		if(Stopped == false) {
        		get_arc_coordinates();
        		//prepare_arc_move(true);
        		return;
      		}
    		case 3: 									// G3  - CCW ARC
      		if(Stopped == false) {
        		get_arc_coordinates();
       			//prepare_arc_move(false);
        		return;
      		}
    		case 4: 									// G4 dwell							//wait for n milliseconds
      		codenum = 0;
      		if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      		if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

      		st_synchronize();
      		////codenum += millis();  // keep track of when we started waiting
			codenum += tim_millis;	//added at 20160518
      		previous_millis_cmd = tim_millis;
      		while(tim_millis  < codenum ){				//when the printer wait,the extruder's temperature can be control.same with lcd
        		manage_heater();
        		manage_inactivity();
      		}
      		break;
    		case 28: 									//G28 Home all Axis one at a time			//home all axis at a time
      		saved_feedrate = feedrate;
      		saved_feedmultiply = feedmultiply;
      		feedmultiply = 100;
			////previous_millis_cmd = millis();
      		previous_millis_cmd = tim_millis;

      		enable_endstops(true);

      		for(i=0; i < NUM_AXIS; i++) {
        	destination[i] = current_position[i];
      		}
      		feedrate = 0.0;

      		home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));

      		#if Z_HOME_DIR > 0                      // If homing away from BED do Z first
      		if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
        		HOMEAXIS(Z);
      		}
			#endif									//end if Z_HOME_DIR

      		if((home_all_axis) || (code_seen(axis_codes[X_AXIS])))
      		{
        		HOMEAXIS(X);
      		}
      		if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
        		HOMEAXIS(Y);
      		}
      		if(code_seen(axis_codes[X_AXIS]))
      		{
        		if(code_value_long() != 0) {
          		current_position[X_AXIS]=code_value() + add_homeing[0];
        		}
      		}
      		if(code_seen(axis_codes[Y_AXIS])) {
        		if(code_value_long() != 0) {
          		current_position[Y_AXIS]=code_value() + add_homeing[1];
        		}
      		}

		      #if Z_HOME_DIR < 0                      // If homing towards BED do Z last
		        #ifndef Z_SAFE_HOMING
		          if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
		            #if defined (Z_RAISE_BEFORE_HOMING) && (Z_RAISE_BEFORE_HOMING > 0)
		              destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING/* * home_dir(Z_AXIS) */* (-1);    // Set destination away from bed
		              feedrate = max_feedrate[Z_AXIS];
		              plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
		              st_synchronize();
		            #endif
		            //HOMEAXIS(Z);
		          }
		        #else                      // Z Safe mode activated.
		          if(home_all_axis) {
		            destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - X_PROBE_OFFSET_FROM_EXTRUDER);
		            destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - Y_PROBE_OFFSET_FROM_EXTRUDER);
		            destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING /** home_dir(Z_AXIS) */* (-1);    // Set destination away from bed
		            feedrate = XY_TRAVEL_SPEED;
		            current_position[Z_AXIS] = 0;
		
		            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		            plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
		            st_synchronize();
		            current_position[X_AXIS] = destination[X_AXIS];
		            current_position[Y_AXIS] = destination[Y_AXIS];
		
		            //HOMEAXIS(Z);
		          }
		                                                // Let's see if X and Y are homed and probe is inside bed area.
		          if(code_seen(axis_codes[Z_AXIS])) {
		            if ( (axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]) \
		              && (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER >= X_MIN_POS) \
		              && (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER <= X_MAX_POS) \
		              && (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER >= Y_MIN_POS) \
		              && (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER <= Y_MAX_POS)) {
		
		              current_position[Z_AXIS] = 0;
		              plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		              destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING /** home_dir(Z_AXIS) */* (-1);    // Set destination away from bed
		              feedrate = max_feedrate[Z_AXIS];
		              plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
		              st_synchronize();
		
		              //HOMEAXIS(Z);
		            } else if (!((axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]))) {
		                //LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
		                SERIAL_ECHO_START;
		                SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
		            } else {
		                //LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
		                SERIAL_ECHO_START;
		                SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
		            }
		          }
		        #endif
		      #endif


		      if(code_seen(axis_codes[Z_AXIS])) {
		        if(code_value_long() != 0) {
		          current_position[Z_AXIS]=code_value()+add_homeing[2];
		        }
		      }
		      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

      		#ifdef ENDSTOPS_ONLY_FOR_HOMING
        		enable_endstops(false);
      		#endif

      		feedrate = saved_feedrate;
      		feedmultiply = saved_feedmultiply;
      		previous_millis_cmd = tim_millis;//millis();
      		endstops_hit_on_purpose();
      		break;
    		case 90: // G90					//set to absolute positioning
     		relative_mode = false;
			break;
			case 91: // G91					//set to relative positioning
  			relative_mode = true;
  			break;
			case 92:
      		if(!code_seen(axis_codes[E_AXIS]))
        		st_synchronize();
      		for(i=0; i < NUM_AXIS; i++) {
        		if(code_seen(axis_codes[i])) {
           			if(i == E_AXIS) {
             			current_position[i] = code_value();
             			plan_set_e_position(current_position[E_AXIS]);
           			}
           			else {
             		current_position[i] = code_value()+add_homeing[i];
             		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
           			}
        		}
      		}
			break;
		}
	}

	else if(code_seen('M')){
		#if defined(DEBUG)
			printf("M%d\r\n",(int)code_value());
		#endif
	    switch( (int)code_value() ){
#ifdef ULTIPANEL
    	case 0: // M0 - 					//Unconditional stop - Wait for user button press on LCD
    	case 1: // M1 - 					//Conditional stop - Wait for user button press on LCD
    	{
      		//LCD_MESSAGEPGM(MSG_USERWAIT);
      		codenum = 0;
      		if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      		if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

      		st_synchronize();
		    ////previous_millis_cmd = millis();
      		previous_millis_cmd = tim_millis;//millis();
      		if (codenum > 0){
        		codenum += tim_millis;//millis();  // keep track of when we started waiting
	        	while(tim_millis  < codenum /*&& !lcd_clicked()*/){
	          		manage_heater();
	          		manage_inactivity();
	          		////lcd_update();
	        	}
      		}
			/*else{
	        while(!lcd_clicked()){
	          manage_heater();
	          manage_inactivity();
	          lcd_update();
	        }
	      }*/
      		//LCD_MESSAGEPGM(MSG_RESUMING);
    	}
    	break;
#endif
    case 17:						//start all the motor
        //LCD_MESSAGEPGM(MSG_NO_MOVE);
        enable_x();
        enable_y();
        enable_z();
        enable_e0();
      break;

    case 31: //M31 					//take time since the start of the SD print or an M109 command
      {
	  ////stoptime=millis();
      stoptime = tim_millis;//millis();
	  t = (stoptime-starttime)/1000;
      min = t/60;
      sec = t%60;
      //sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
      SERIAL_ECHO_START;
      SERIAL_ECHOLN(time);
      //lcd_setstatus(time);
      //autotempShutdown();
      }
      break;
    case 42: //M42 -					//Change pin status via gcode
      if (code_seen('S'))
      {
        pin_status = code_value();
        pin_number = 0;//LED_PIN;
        if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
          pin_number = code_value();
        for(i = 0; i < (int8_t)sizeof(sensitive_pins); i++)
        {
          if (sensitive_pins[i] == pin_number)
          {
            pin_number = -1;
            break;
          }
        }
      #if defined(FAN_PIN)// && FAN_PIN > -1
        if (pin_number == FAN_PIN)
          fanSpeed = pin_status;
      #endif
        if (pin_number > -1)
        {
          //pinMode(pin_number, OUTPUT);
          //digitalWrite(pin_number, pin_status);
          //analogWrite(pin_number, pin_status);
        }
      }
      break;

    case 104: // M104					//set the extruder temperature
      if(setTargetedHotend(104)){
        break;
      }
      if (code_seen('S')) 
	  	setTargetHotend(code_value(), tmp_extruder);
      //setWatch();
      break;
	  case 140: // M140 					//set bed temp
      if (code_seen('S')) 
	  	setTargetBed(code_value());
	  break;
    case 105: // M105					//get the temperature of extruder
      if(setTargetedHotend(105)){
        break;
        }
      #if defined(TEMP_0_PIN)// && TEMP_0_PIN > -1
        SERIAL_PROTOCOLPGM("ok T:");
        SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
        SERIAL_PROTOCOLPGM(" /");
        SERIAL_PROTOCOL_F(degTargetHotend(tmp_extruder),1);
        #if defined(TEMP_BED_PIN)// && TEMP_BED_PIN > -1
          SERIAL_PROTOCOLPGM("\r\nB:");
          SERIAL_PROTOCOL_F(degBed(),1);
          SERIAL_PROTOCOLPGM(" /");
          SERIAL_PROTOCOL_F(degTargetBed(),1);
        #endif //TEMP_BED_PIN
		connect_flag++;
		if(connect_flag >= 10)
			connect_flag = 10;
		if(connect_flag == 1){
			//NVIC_SystemReset();
		}
        for(cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
          SERIAL_PROTOCOLPGM("\r\nT");
          SERIAL_PROTOCOL(cur_extruder);
          SERIAL_PROTOCOLPGM(":");
          SERIAL_PROTOCOL_F(degHotend(cur_extruder),1);
          SERIAL_PROTOCOLPGM(" /");
          SERIAL_PROTOCOL_F(degTargetHotend(cur_extruder),1);
        }
      #else
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
      #endif

        SERIAL_PROTOCOLPGM("\r\n@:");
        SERIAL_PROTOCOL(getHeaterPower(tmp_extruder));

        SERIAL_PROTOCOLPGM("\r\nB@:");
        SERIAL_PROTOCOL(getHeaterPower(-1));

        SERIAL_PROTOCOLLN("");
      return;
	  break;

      case 109:							//set the target temperature and wait to reach it
      {// M109 - Wait for extruder heater to reach target.
	  	if(setTargetedHotend(109)){
      		break;
      	}
      //LCD_MESSAGEPGM(MSG_HEATING);
      #ifdef AUTOTEMP
        autotemp_enabled=false;
      #endif
	      if (code_seen('S')) {
	        setTargetHotend(code_value(), tmp_extruder);
	        CooldownNoWait = true;
	      }else if (code_seen('R')) {
	        setTargetHotend(code_value(), tmp_extruder);
	        CooldownNoWait = false;
	      }
      #ifdef AUTOTEMP
        if (code_seen('S')) autotemp_min=code_value();
        if (code_seen('B')) autotemp_max=code_value();
        if (code_seen('F'))
        {
          autotemp_factor = code_value();
          autotemp_enabled = true;
        }
      #endif
      //setWatch();
      codenum = tim_millis;//millis();

      /* See if we are heating up or cooling down */
      target_direction = isHeatingHotend(tmp_extruder); // true if heating, false if cooling

      #ifdef TEMP_RESIDENCY_TIME
        //long residencyStart;
        residencyStart = -1;
        /* continue to loop until we have reached the target temp
          _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
        while((residencyStart == -1) ||
              (residencyStart >= 0 && (((unsigned int) (current_time - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL))) ) {
      #endif //TEMP_RESIDENCY_TIME
	      current_time = tim_millis;
          if( (current_time - codenum) > 1000UL )
          { //Print Temp Reading and remaining time every 1 second while heating up/cooling down
            SERIAL_PROTOCOLPGM("T:");
            SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
            SERIAL_PROTOCOLPGM(" E:");
            SERIAL_PROTOCOL((int)tmp_extruder);
            #ifdef TEMP_RESIDENCY_TIME
              SERIAL_PROTOCOLPGM(" W:");
              if(residencyStart > -1)
              {
			     current_time = tim_millis;
                 codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (current_time - residencyStart)) / 1000UL;
                 SERIAL_PROTOCOLLN( codenum );
              }
              else
              {
                 SERIAL_PROTOCOLLN( "?" );
              }
            #else
              SERIAL_PROTOCOLLN("");
            #endif
            codenum = tim_millis;
          }
          manage_heater();
          manage_inactivity();
          //lcd_update();
        #ifdef TEMP_RESIDENCY_TIME
            /* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
              or when current temp falls outside the hysteresis after target temp was reached */
          if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
              (residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
              (residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS) )
          {
            residencyStart = tim_millis;//millis();
          }
        #endif //TEMP_RESIDENCY_TIME
        
        //LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
        starttime = tim_millis;//millis();
        previous_millis_cmd = tim_millis;//millis();
      }
      break;

	  }
		case 190: // M190 - 						//Wait for bed heater to reach target.
#if defined(TEMP_BED_PIN)// && TEMP_BED_PIN > -1
		    //LCD_MESSAGEPGM(MSG_BED_HEATING);
		    if (code_seen('S')) {
		      setTargetBed(code_value());
		      CooldownNoWait = true;
		    } else if (code_seen('R')) {
		      setTargetBed(code_value());
		      CooldownNoWait = false;
		    }
		    codenum = tim_millis;//millis();
		
		    target_direction = isHeatingBed(); // true if heating, false if cooling
		
		    while ( target_direction ? (isHeatingBed()) : (isCoolingBed()&&(CooldownNoWait==false)) )
		    {
			  current_time = tim_millis;
		      if(( current_time - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
		      {
		        tt = degHotend(active_extruder);
		        SERIAL_PROTOCOLPGM("T:");
		        SERIAL_PROTOCOLF(tt);
		        SERIAL_PROTOCOLPGM(" E:");
		        SERIAL_PROTOCOL((int)active_extruder);
		        SERIAL_PROTOCOLPGM(" B:");
		        SERIAL_PROTOCOL_F(degBed(),1);
		        SERIAL_PROTOCOLLN("");
		        codenum = tim_millis;//millis();
		      }
		      manage_heater();
		      manage_inactivity();
		      ////lcd_update();
		    }
		    //LCD_MESSAGEPGM(MSG_BED_DONE);
		    previous_millis_cmd = tim_millis;//millis();
#endif
		    break;
#if defined(FAN_PIN)// && FAN_PIN > -1
      case 106: //M106 Fan On					//turn the fan
        if (code_seen('S')){
           fanSpeed=constrain(code_value(),0,255);
        }
        else {
          fanSpeed=255;
        }
        break;
      case 107: //M107 Fan Off				//turn off the fan
        fanSpeed = 0;
        break;
#endif //FAN_PIN
#if defined(PS_ON_PIN) && PS_ON_PIN > -1
      case 80: // M80 - 						//Turn on Power Supply
        //SET_OUTPUT(PS_ON_PIN); //GND
        //WRITE(PS_ON_PIN, PS_ON_AWAKE);

        // If you have a switch on suicide pin, this is useful
        // if you want to start another print with suicide feature after
        // a print without suicide...
        /*#if defined SUICIDE_PIN && SUICIDE_PIN > -1
            SET_OUTPUT(SUICIDE_PIN);
            WRITE(SUICIDE_PIN, HIGH);
        #endif

        #ifdef ULTIPANEL
          powersupply = true;
          LCD_MESSAGEPGM(WELCOME_MSG);
          lcd_update();
        #endif*/
        break;
#endif

		case 81: // M81 - 						//Turn off Power Supply
		disable_heater();
		st_synchronize();
		disable_e0();
		finishAndDisableSteppers();
		fanSpeed = 0;
		//delay(1000); // Wait a little before to switch off
	#if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
		st_synchronize();
		suicide();
	#elif defined(PS_ON_PIN) && PS_ON_PIN > -1
		SET_OUTPUT(PS_ON_PIN);
		WRITE(PS_ON_PIN, PS_ON_ASLEEP);
	#endif
	#ifdef ULTIPANEL
		powersupply = false;
		//LCD_MESSAGEPGM(MACHINE_NAME" "MSG_OFF".");
		//lcd_update();
	#endif
		break;
    case 82:								//set the printer to the absolute position mode
      axis_relative_modes[3] = false;
      break;
    case 83:								//set the printer to the relative position mode
      axis_relative_modes[3] = true;
      break;
    case 18: //compatibility
    case 84: // M84
      if(code_seen('S')){
        stepper_inactive_time = code_value() * 1000;
      }
      else
      {
        all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2]))|| (code_seen(axis_codes[3])));
        if(all_axis)
        {
          st_synchronize();
          disable_e0();
          finishAndDisableSteppers();
        }
        else
        {
          st_synchronize();
          if(code_seen('X')) disable_x();
          if(code_seen('Y')) disable_y();
          if(code_seen('Z')) disable_z();
		  temp1 = E0_ENABLE_PIN;
		  temp2 = X_ENABLE_PIN;
          #if ((temp1 != temp2)/* && (E1_ENABLE_PIN != Y_ENABLE_PIN)*/) // Only enable on boards that have seperate ENABLE_PINS
            if(code_seen('E')) {
              disable_e0();
            }
          #endif
        }
      }
      break;
	case 85: // M85							//set the inactive time
		code_seen('S');
		max_inactive_time = code_value() * 1000;
	break;
    case 92: // M92							//set the value of axis_steps_per_unit
      for(i = 0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          if(i == 3) { // E
            value = code_value();
            if(value < 20.0) {
              factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
              max_e_jerk *= factor;
              max_feedrate[i] *= factor;
              axis_steps_per_sqr_second[i] *= factor;
            }
            axis_steps_per_unit[i] = value;
          }
          else {
            axis_steps_per_unit[i] = code_value();
          }
        }
      }
      break;
    case 115: // M115						//print the printer's message
      //SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
    break;
    case 117: // M117 display message			//print the message to the lcd
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos!=NULL)
        *(starpos-1)='\0';
      //lcd_setstatus(strchr_pointer + 5);
      break;
    case 114: // M114						//print the x,y,z,e message
      SERIAL_PROTOCOLPGM("X:");
      SERIAL_PROTOCOLF(current_position[X_AXIS]);
      SERIAL_PROTOCOLPGM("Y:");
      SERIAL_PROTOCOLF(current_position[Y_AXIS]);
      SERIAL_PROTOCOLPGM("Z:");
      SERIAL_PROTOCOLF(current_position[Z_AXIS]);
      SERIAL_PROTOCOLPGM("E:");
      SERIAL_PROTOCOLF(current_position[E_AXIS]);

      SERIAL_PROTOCOLPGM(MSG_COUNT_X);
      SERIAL_PROTOCOLF((float)(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
      SERIAL_PROTOCOLPGM("Y:");
      SERIAL_PROTOCOLF((float)(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
      SERIAL_PROTOCOLPGM("Z:");
      SERIAL_PROTOCOLF((float)(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);

      SERIAL_PROTOCOLLN("");
	  
      break;
    case 120: // M120						//disable the endstops
      enable_endstops(false) ;
      break;
    case 121: // M121						//enable the endstops
      enable_endstops(true) ;
      break;
    case 119: // M119						//report endstops' status
    //SERIAL_PROTOCOLLN(MSG_M119_REPORT);
      #if defined(X_MIN_PIN) && X_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_X_MIN);
        //SERIAL_PROTOCOLLN(((READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(X_MAX_PIN) && X_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_X_MAX);
        //SERIAL_PROTOCOLLN(((READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Y_MIN);
        //SERIAL_PROTOCOLLN(((READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Y_MAX);
        //SERIAL_PROTOCOLLN(((READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Z_MIN);
        //SERIAL_PROTOCOLLN(((READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Z_MAX);
        //SERIAL_PROTOCOLLN(((READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      break;
    case 201: // M201						//set the value of max_acceleration_units_per_sq_second
      for(i = 0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          max_acceleration_units_per_sq_second[i] = code_value();
        }
      }
      // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
      reset_acceleration_rates();
      break;
    case 203: // M203 						//set the max feedrate mm/sec
      for(i = 0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
      }
      break;
    case 204: // M204 						//acclereration S normal moves T filmanent only moves
      {
        if(code_seen('S')) acceleration = code_value() ;
        if(code_seen('T')) retract_acceleration = code_value() ;
      }
      break;
    case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
    {
      if(code_seen('S')) minimumfeedrate = code_value();
      if(code_seen('T')) mintravelfeedrate = code_value();
      if(code_seen('B')) minsegmenttime = code_value() ;
      if(code_seen('X')) max_xy_jerk = code_value() ;
      if(code_seen('Z')) max_z_jerk = code_value() ;
      if(code_seen('E')) max_e_jerk = code_value() ;
    }
    break;
    case 206: // M206 						//set the additional homeing offset
      for(i = 0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) add_homeing[i] = code_value();
      }
    break;

    case 220: 									// M220 S<factor in percent>- set speed factor override percentage
    {
      if(code_seen('S'))
      {
        feedmultiply = code_value() ;
      }
    }
    break;
    case 221: 									// M221 S<factor in percent>- set extrude factor override percentage
    {
      if(code_seen('S'))
      {
        extrudemultiply = code_value() ;
      }
    }
    break;
	case 226: 									// M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
	{
      if(code_seen('P')){
        pin_number = code_value(); // pin number
        pin_state = -1; // required pin state - default is inverted

        if(code_seen('S')) pin_state = code_value(); // required pin state

        if(pin_state >= -1 && pin_state <= 1){

          for(i = 0; i < (int8_t)sizeof(sensitive_pins); i++)
          {
            if (sensitive_pins[i] == pin_number)
            {
              pin_number = -1;
              break;
            }
          }

          if (pin_number > -1)
          {
            st_synchronize();

            //pinMode(pin_number, INPUT);

            //int target;
            switch(pin_state){
            case 1:
              target = 1;//HIGH;
              break;

            case 0:
              target = 0;//LOW;
              break;

            case -1:
              //target = !digitalRead(pin_number);
              break;
            }

            /*while(digitalRead(pin_number) != target){
              manage_heater();
              manage_inactivity();
              lcd_update();
            }*/
          }
        }
      }
    }
    break;
    #ifdef PIDTEMP
    case 301: // M301							//set the value of P I D
      {
        if(code_seen('P')) Kp = code_value();
        if(code_seen('I')) Ki = scalePID_i(code_value());
        if(code_seen('D')) Kd = scalePID_d(code_value());

        #ifdef PID_ADD_EXTRUSION_RATE
        if(code_seen('C')) Kc = code_value();
        #endif

        updatePID();
        SERIAL_PROTOCOL(MSG_OK);
        SERIAL_PROTOCOL(" p:");
        SERIAL_PROTOCOLF(Kp);
        SERIAL_PROTOCOL(" i:");
        SERIAL_PROTOCOLF(unscalePID_i(Ki));
        SERIAL_PROTOCOL(" d:");
        SERIAL_PROTOCOLF(unscalePID_d(Kd));
        #ifdef PID_ADD_EXTRUSION_RATE
        SERIAL_PROTOCOL(" c:");
        //Kc does not have scaling applied above, or in resetting defaults
        SERIAL_PROTOCOLF(Kc);
        #endif
        SERIAL_PROTOCOLLN("");
      }
      break;
    #endif //PIDTEMP
    case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
     {
      #if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
        //NUM_PULSES = 16;
        //PULSE_LENGTH = 0.01524;
        for(i=0; i < NUM_PULSES; i++) {
          //WRITE(PHOTOGRAPH_PIN, HIGH);
          //_delay_ms(PULSE_LENGTH);
          //WRITE(PHOTOGRAPH_PIN, LOW);
          //_delay_ms(PULSE_LENGTH);
        }
        //delay(7.33);
        for(i=0; i < NUM_PULSES; i++) {
          //WRITE(PHOTOGRAPH_PIN, HIGH);
          //_delay_ms(PULSE_LENGTH);
          //WRITE(PHOTOGRAPH_PIN, LOW);
          //_delay_ms(PULSE_LENGTH);
        }
      #endif
     }
    break;
#ifdef PREVENT_DANGEROUS_EXTRUDE
    case 302: 											// allow cold extrudes, or set the minimum extrude temperature
    {
	  temp = 0.0;
	  if (code_seen('S')) 
	  	temp=code_value();
      set_extrude_min_temp(temp);
    }
    break;
#endif
    case 303: 											// M303 PID autotune
    {
      temp = 150.0;
      e = 0;
      c = 5;
      if (code_seen('E')) e = code_value();
        if (e<0)
          temp=70;
      if (code_seen('S')) temp = code_value();
      if (code_seen('C')) c = code_value();
      PID_autotune(temp, e, c);
    }
    break;
    case 400: 											// M400 finish all moves
    {
      st_synchronize();
    }
    break;
    case 500: // M500 Store settings in EEPROM
    {
        //Config_StoreSettings();
    }
    break;
    case 501: // M501 Read settings from EEPROM
    {
        //Config_RetrieveSettings();
    }
    break;
    case 502: // M502 Revert to default settings
    {
        Config_ResetDefault();
    }
    break;
    case 503: // M503 print settings currently in memory
    {
        Config_PrintSettings();
    }
    break;

    #ifdef FILAMENTCHANGEENABLE
    case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
    {
        //float target[4];
        //float lastpos[4];
        target1[X_AXIS] =current_position[X_AXIS];
        target1[Y_AXIS] =current_position[Y_AXIS];
        target1[Z_AXIS] =current_position[Z_AXIS];
        target1[E_AXIS] =current_position[E_AXIS];
        lastpos[X_AXIS]=current_position[X_AXIS];
        lastpos[Y_AXIS]=current_position[Y_AXIS];
        lastpos[Z_AXIS]=current_position[Z_AXIS];
        lastpos[E_AXIS]=current_position[E_AXIS];
        //retract by E
        if(code_seen('E'))
        {
          target1[E_AXIS] += code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_FIRSTRETRACT
            target1[E_AXIS] += FILAMENTCHANGE_FIRSTRETRACT ;
          #endif
        }
        plan_buffer_line(target1[X_AXIS], target1[Y_AXIS], target1[Z_AXIS], target1[E_AXIS], feedrate/60, active_extruder);

        //lift Z
        if(code_seen('Z'))
        {
          target1[Z_AXIS] += code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_ZADD
            target1[Z_AXIS] += FILAMENTCHANGE_ZADD ;
          #endif
        }
        plan_buffer_line(target1[X_AXIS], target1[Y_AXIS], target1[Z_AXIS], target1[E_AXIS], feedrate/60, active_extruder);

        //move xy
        if(code_seen('X'))
        {
          target1[X_AXIS] += code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_XPOS
            target1[X_AXIS]= FILAMENTCHANGE_XPOS ;
          #endif
        }
        if(code_seen('Y'))
        {
          target1[Y_AXIS]= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_YPOS
            target1[Y_AXIS]= FILAMENTCHANGE_YPOS ;
          #endif
        }

        plan_buffer_line(target1[X_AXIS], target1[Y_AXIS], target1[Z_AXIS], target1[E_AXIS], feedrate/60, active_extruder);

        if(code_seen('L'))
        {
          target1[E_AXIS] += code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_FINALRETRACT
            target1[E_AXIS] += FILAMENTCHANGE_FINALRETRACT ;
          #endif
        }

        plan_buffer_line(target1[X_AXIS], target1[Y_AXIS], target1[Z_AXIS], target1[E_AXIS], feedrate/60, active_extruder);

        //finish moves
        st_synchronize();
        //disable extruder steppers so filament can be removed
        disable_e0();
        //delay(100);
        //LCD_ALERTMESSAGEPGM(MSG_FILAMENTCHANGE);
        cnt=0;
        /*while(!lcd_clicked()){
          cnt++;
          manage_heater();
          manage_inactivity();
          lcd_update();
          if(cnt==0)
          {
          #if BEEPER > 0
            SET_OUTPUT(BEEPER);

            WRITE(BEEPER,HIGH);
            delay(3);
            WRITE(BEEPER,LOW);
            delay(3);
          #else
            lcd_buzz(1000/6,100);
          #endif
          }
        }*/

        //return to normal
        if(code_seen('L'))
        {
          target1[E_AXIS] += -code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_FINALRETRACT
            target1[E_AXIS] += (-1)*FILAMENTCHANGE_FINALRETRACT ;
          #endif
        }
        current_position[E_AXIS]=target1[E_AXIS]; //the long retract of L is compensated by manual filament feeding
        plan_set_e_position(current_position[E_AXIS]);
        plan_buffer_line(target1[X_AXIS], target1[Y_AXIS], target1[Z_AXIS], target1[E_AXIS], feedrate/60, active_extruder); //should do nothing
        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], target1[Z_AXIS], target1[E_AXIS], feedrate/60, active_extruder); //move xy back
        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target1[E_AXIS], feedrate/60, active_extruder); //move z back
        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder); //final untretract
    }
    break;
    #endif //FILAMENTCHANGEENABLE
    case 907: // M907 Set digital trimpot motor current using axis codes.
    {
      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        for(i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) digipot_current(i,code_value());
        if(code_seen('B')) digipot_current(4,code_value());
        if(code_seen('S')) for(i=0;i<=4;i++) digipot_current(i,code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_XY_PIN
        if(code_seen('X')) digipot_current(0, code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_Z_PIN
        if(code_seen('Z')) digipot_current(1, code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_E_PIN
        if(code_seen('E')) digipot_current(2, code_value());
      #endif
    }
    break;
    case 908: // M908 Control digital trimpot directly.
    {
      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        //uint8_t channel,current;
        if(code_seen('P')) channel=code_value();
        if(code_seen('S')) current=code_value();
        //digitalPotWrite(channel, current);
      #endif
    }
    break;
	case 350:
	//
	break;
	case 351:
	//
	break;
    case 999: // M999: Restart after being stopped
      Stopped = false;
      //lcd_reset_alert_level();
      gcode_LastN = Stopped_gcode_LastN;
      FlushSerialRequestResend();
    break;
		}
	}

	else if(code_seen('T'))
	{
    tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      SERIAL_ECHO("T");
      SERIAL_ECHO(tmp_extruder);
      SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
    }
	else
	{
      make_move = false;
      if(code_seen('F')) {
        make_move = true;
        next_feedrate = code_value();
        if(next_feedrate > 0.0) {
          feedrate = next_feedrate;
        }
      }
      #if EXTRUDERS > 1
      if(tmp_extruder != active_extruder) {
        // Save current position to return to after applying extruder offset
        memcpy(destination, current_position, sizeof(destination));
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        // Move to the old position if 'F' was in the parameters
        if(make_move && Stopped == false) {
           prepare_move();
        }
      }
      #endif
      SERIAL_ECHO_START;
      SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
      SERIAL_PROTOCOLLN((int)active_extruder);
      }
	}

	else
	{
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
    SERIAL_ECHO(cmdbuffer[bufindr]);
    SERIAL_ECHOLNPGM("\"");
	}
	ClearToSend();
}
#endif

/****************************************************************************
name:		code_seen
function:	
			try to find a code in a string,return true if it is found,else false
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
#if 1
uint8_t code_seen(char code)
{
	strchr_pointer = strchr(cmdbuffer[bufindr], code);//strchr(const char *s,char c)查找字符串s中首次出现字符c的位置
  	return (strchr_pointer != NULL);  //Return True if a character was found
}

/****************************************************************************
name:		code_value
function:	
			transform the string to the float number
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
float code_value(void)
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}


/****************************************************************************
name:		code_value_long
function:	
			transform the string to the long int number
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
long code_value_long(void)
{
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

/****************************************************************************
name:		prepare_move
function:	
			movement
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
void clamp_to_software_endstops(float target[3])
{
  if (min_software_endstops) {
    if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
    if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
    if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
  }

  if (max_software_endstops) {
    if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
    if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
  }
}

/****************************************************************************
name:		prepare_move
function:	
			movement
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
void prepare_move(void)
{
	int8_t i;
	clamp_to_software_endstops(destination);

	previous_millis_cmd = tim_millis;//millis();

	// Do not use feedmultiply for E or Z only moves
  	if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
    	plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
  	}
  	else {
    	plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
  	}

	for(i=0; i < NUM_AXIS; i++) {
    	current_position[i] = destination[i];
  	}
}


/****************************************************************************
name:		next_block_index
function:	
			calculate the next block's index
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
int8_t next_block_index(int8_t block_index) {
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { 
    block_index = 0; 
  }
  return(block_index);
}

/****************************************************************************
name:		code_value
function:	
			transform the string to the float number
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
void get_coordinates(void)
{
	volatile uint8_t seen[4]={false,false,false,false};
	int8_t i;
	for(i=0; i < NUM_AXIS; i++) {
    	if(code_seen(axis_codes[i]))
    	{
      		destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
      		seen[i]=true;
    	}
    	else 
			destination[i] = current_position[i]; //Are these else lines really needed?
  	}
	if(code_seen('F')) {
    	next_feedrate = code_value();
    	if(next_feedrate > 0.0) 
			feedrate = next_feedrate;
	}
}

/****************************************************************************
name:		get_arc_coordinates
function:	
			get the arc coordinate of the extruder
Parameters:
			[in]	-	void
Returns:
			[out]	-	void
Description:
			null
****************************************************************************/
void get_arc_coordinates(void)
{
   get_coordinates();

   if(code_seen('I')) {
     offset[0] = code_value();
   }
   else {
     offset[0] = 0.0;
   }
   if(code_seen('J')) {
     offset[1] = code_value();
   }
   else {
     offset[1] = 0.0;
   }
}

/***************************************************************************************
name:		setTargetedHotend()
function:	set the target hotend
			[in]	-	code:the code to chose the hotend
			[out]	-	void
***************************************************************************************/
int setTargetedHotend(int code){
  tmp_extruder = active_extruder;
  if(code_seen('T')) {
    tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) {
      //SERIAL_ECHO_START;
      switch(code){
        case 104:
          //SERIAL_ECHO(MSG_M104_INVALID_EXTRUDER);
          break;
        case 105:
          //SERIAL_ECHO(MSG_M105_INVALID_EXTRUDER);
          break;
        case 109:
          //SERIAL_ECHO(MSG_M109_INVALID_EXTRUDER);
          break;
        case 218:
          //SERIAL_ECHO(MSG_M218_INVALID_EXTRUDER);
          break;
      }
      SERIAL_ECHOLN(tmp_extruder);
      return true;
    }
  }
  return false;
}

/***************************************************************************************
name:		manage_inactivity()
function:	manage the printer when it has nothing to do
			[in]	-	void
			[out]	-	void
***************************************************************************************/
void manage_inactivity()
{
	unsigned int temp = 0;
	temp = tim_millis;
	if( (temp - previous_millis_cmd) >  max_inactive_time )
	if(max_inactive_time)
	  kill();

	if(stepper_inactive_time)  {
	temp = tim_millis;
	if( (temp - previous_millis_cmd) >  stepper_inactive_time )
		{
		  if(blocks_queued() == false) {
		    disable_x();
		    disable_y();
		    disable_z();
		    disable_e0();
		  }
		}
	}

	check_axes_activity();
}

/*********************************************************************************************
name:		kill()
function:	disable all the function of the printer
			[in]	-	void
			[out]	-	void
*********************************************************************************************/
void kill()
{
  //cli(); // Stop interrupts
  disable_heater();

  disable_x();
  disable_y();
  disable_z();
  disable_e0();
#if defined(PS_ON_PIN) && PS_ON_PIN > -1
  //pinMode(PS_ON_PIN,INPUT);
#endif
  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
  //LCD_ALERTMESSAGEPGM(MSG_KILLED);
  suicide();
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

/**************************************************************************************
name:		suicide()
function:	stop the printer
**************************************************************************************/
void suicide()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    //SET_OUTPUT(SUICIDE_PIN);
    //WRITE(SUICIDE_PIN, LOW);
  #endif
}
#endif

// 发送数据
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (unsigned char) ch);// USART1 可以换成 USART2 等
    while (!(USART1->SR & USART_FLAG_TXE));
    return (ch);
}
// 接收数据
int GetKey (void)  
{
	while (!(USART1->SR & USART_FLAG_RXNE));
    return ((int)(USART1->DR & 0x1FF));
}
