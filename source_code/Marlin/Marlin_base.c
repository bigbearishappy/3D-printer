#include"Marlin_base.h"
#include"planner.h"

void system_init(void)
{
	printf("%d\n",BAUDRATE);
	printf("start\n");

//�����кܶ࿪��ʱ�Ĵ�ӡ��Ϣ�������Ժ�ר������20160412

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

   // ��������
   int fputc(int ch, FILE *f)
   {
      USART_SendData(USART1, (unsigned char) ch);// USART1 ���Ի��� USART2 ��
      while (!(USART1->SR & USART_FLAG_TXE));
      return (ch);
   }
   // ��������
   int GetKey (void)  
   {
      while (!(USART1->SR & USART_FLAG_RXNE));
      return ((int)(USART1->DR & 0x1FF));
   }
