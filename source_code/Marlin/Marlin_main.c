#include"Marlin.h"

#include"ConfigurationStore.h"
#include"temperature.h"
#include"stepper.h"
#include"planner.h"
#include"language.h"
#include"BSP.h"

static int buflen = 0;
static char serial_char;
static uint32_t serial_count = 0;

static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static uint8_t fromsd[BUFSIZE];
static int bufindw = 0;
static int bufindr = 0;

static long gcode_N, gcode_LastN;
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc
uint8_t Stopped = false;
static unsigned long previous_millis_cmd = 0;

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
static float offset[3] = {0.0, 0.0, 0.0};
static uint8_t home_all_axis = true;
uint8_t axis_known_position[3] = {false, false, false};

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
int main()
{
	system_init();
	for(;;)
	{
		if(buflen < (BUFSIZE - 1))
			get_command();
		if(buflen)
		{
			process_command();
			buflen -= 1;
		}
		/*manage_heater();
		manage_inactivity();
		checkHitEndstops();*/
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
	RCC_Configuration();
	NVIC_Configuration();
	TIM_Configuration();
	USART_Configuration();

	printf("%d\n",BAUDRATE);
	printf("start\n");

//这里有很多开机时的打印信息，留在以后专门整理20160412

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();

  tp_init();				// Initialize temperature loop
  plan_init();				// Initialize planner;
  //watchdog_init();
  st_init();				// Initialize stepper, this enables interrupts!
  //servo_init();
  //delay(1000);
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
	while(USART1_DATA_OK == READY)
	{
		serial_char = USART1_Cache[i++];
		if(serial_char == '\n' || serial_char == '\r'||(serial_char == ':' && comment_mode == false)||serial_count >= (MAX_CMD_SIZE - 1))
		{
			if(!serial_count){						//if empty line
				comment_mode = false;				//for new line
				//return;
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
            			//return;
          				}  
						//if no errors, continue parsing
					
					else
          			{
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
  previous_millis_cmd = 1;//millis();
  printf(MSG_OK);printf("\r\n");
}

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
void process_command(void)
{
	unsigned long codenum; //throw away variable
	char *starpos = NULL;
	int8_t i;
	if(code_seen('G'))
	{
		switch((int)code_value())
    	{
    		case 0: 									// G0 -> G1							//fast move
    		case 1: 									// G1								//fast move
			if(Stopped == false) {
        	get_coordinates(); // For X Y Z E F
        	//prepare_move();
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
      		//codenum += millis();  // keep track of when we started waiting
			codenum += tim_millis;	//added at 20160518
      		previous_millis_cmd = tim_millis;
      		while(tim_millis  < codenum ){				//when the printer wait,the extruder's temperature can be control.same with lcd
        		//manage_heater();
        		//manage_inactivity();
      		}
      		break;
    		case 28: 									//G28 Home all Axis one at a time			//home all axis at a time
      		saved_feedrate = feedrate;
      		saved_feedmultiply = feedmultiply;
      		feedmultiply = 100;
      		previous_millis_cmd = tim_millis;

      		enable_endstops(true);

      		for(i=0; i < NUM_AXIS; i++) {
        	destination[i] = current_position[i];
      		}
      		feedrate = 0.0;
      		home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));

      		#if Z_HOME_DIR > 0                      // If homing away from BED do Z first
      		if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
        	//HOMEAXIS(Z);
      		}
			#endif									//end if Z_HOME_DIR

      		if((home_all_axis) || (code_seen(axis_codes[X_AXIS])))
      		{
        		//HOMEAXIS(X);
      		}
      		if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
        		//HOMEAXIS(Y);
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
		              //plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
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
		
		            //plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		            //plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
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
		              //plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		              destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING /** home_dir(Z_AXIS) */* (-1);    // Set destination away from bed
		              feedrate = max_feedrate[Z_AXIS];
		              //plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
		              st_synchronize();
		
		              //HOMEAXIS(Z);
		            } else if (!((axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]))) {
		                //LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
		                //SERIAL_ECHO_START;
		                //SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
		            } else {
		                //LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
		                //SERIAL_ECHO_START;
		                //SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
		            }
		          }
		        #endif
		      #endif


		      if(code_seen(axis_codes[Z_AXIS])) {
		        if(code_value_long() != 0) {
		          current_position[Z_AXIS]=code_value()+add_homeing[2];
		        }
		      }
		      //plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

      		#ifdef ENDSTOPS_ONLY_FOR_HOMING
        		enable_endstops(false);
      		#endif

      		feedrate = saved_feedrate;
      		feedmultiply = saved_feedmultiply;
      		previous_millis_cmd = tim_millis;//millis();
      		endstops_hit_on_purpose();
      		break;

		}
	}
}

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
    	//plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
  	}
  	else {
    	//plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
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
	uint8_t seen[4]={false,false,false,false};
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
