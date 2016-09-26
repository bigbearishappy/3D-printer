#include"Marlin_base.h"
#include"planner.h"

void system_init(void)
{
	printf("%d\n",BAUDRATE);
	printf("start\n");

//这里有很多开机时的打印信息，留在以后专门整理20160412

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();

  tp_init();				// Initialize temperature loop
  plan_init();				// Initialize planner;
  //watchdog_init();
  st_init();				// Initialize stepper, this enables interrupts!
  setup_photpin();			// initialize the photopin
  //servo_init();
  //delay(1000);
}


void setup_photpin()
{}

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
