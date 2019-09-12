/**
  ******************************************************************************
  * @file    main.c
  * @author  liu Yong
  * @version V1.1.0
  * @date    31-July-2015
  * @brief   Main program body
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "board_init.h"
#include "main.h"
#include "uart.h"
#include "rs485.h"
#include "led.h"
#include "mechanicalcontrol.h"
#include "ads1255.h"
#include "direction.h"
#include "malloc.h"

/*--------------- Tasks Priority -------------*/
#define MAIN_TASK_PRIO   ( tskIDLE_PRIORITY + 1 )      
#define LED_TASK_PRIO    ( tskIDLE_PRIORITY + 2 )
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void ToggleLed(void * pvParameters);
void Main_task(void * pvParameters);

/* Private functions ---------------------------------------------------------*/

xQueueHandle xQueue_Uart;
xQueueHandle xQueue_ADStart;
xQueueHandle xQueue_ADPreStart;
xQueueHandle xQueue_ADFinish;
xQueueHandle xQueue_LoadData;
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured to 
       168 MHz, this is done through SystemInit() function which is called from
       startup file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */  
  board_init();
//	while(1)
//	{
//		while(USART_GetFlagStatus(RS485_USART, USART_FLAG_TC)==RESET);
//    USART_SendData(RS485_USART,0x55);		
//	}
	my_mem_init(SRAMIN);  	//初始化内部内存池
	my_mem_init(SRAMCCM); 	//初始化CCM内存池
	xQueue_Uart = xQueueCreate( 3, sizeof(Uart_Queue_Struct) );
	xQueue_ADStart = xQueueCreate( 2, sizeof(uint8_t) );
	xQueue_ADPreStart = xQueueCreate( 2, sizeof(uint8_t) );
	xQueue_ADFinish = xQueueCreate( 1, sizeof(uint8_t) );
	xQueue_LoadData = xQueueCreate( 2, sizeof(uint8_t) );
  /* Configures the priority grouping: 4 bits pre-emption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	/* Init task */
  xTaskCreate(Main_task, (int8_t *) "Main", configMINIMAL_STACK_SIZE * 2, NULL,MAIN_TASK_PRIO, NULL);

  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  for( ;; );
}
void Main_task(void * pvParameters)
{	
	/* Initialize Mechanical task  */
	mechanical_init();
	
	/* Initialize Uart task  */
	uart_init();
	
	/* Initialize ADC task  */
	ADC_init();
	
	/* Initialize Direction task  */
//	Direction_init();
	
  /* Start toogleLed task : Toggle LED  every 250ms */
  //xTaskCreate(ToggleLed, (int8_t *) "LED", configMINIMAL_STACK_SIZE, NULL, LED_TASK_PRIO, NULL);

  for( ;; )
  {
      vTaskDelete(NULL);
  }
}

/**
  * @brief  Toggle Led4 task
  * @param  pvParameters not used
  * @retval None
  */
void ToggleLed(void * pvParameters)
{
  for( ;; )
  {
    /* toggle LED4 each 250ms */
    STM_EVAL_LEDToggle();
    vTaskDelay(250);
  }
}
/**
  * @brief  Inserts a delay time.
  * @param  nCount: number of Ticks to delay.
  * @retval None
  */
void Delay(uint32_t nCount)
{
  vTaskDelay(nCount);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
