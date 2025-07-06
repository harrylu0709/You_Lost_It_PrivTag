

#include <stdio.h>
#include "LIS3DSH.h"
#include "led.h"
#include "ble.h"

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_pcd.h"

#define SYSTICK_TIM_CLK   	16000000UL //16MHz
#define NON_DISCOVER_SEC	10
#define LED_PERIOD_MS	  	500
#define ONE_MIN_CNT	 	  	(NON_DISCOVER_SEC*(1000/LED_PERIOD_MS))
#define IRQNO_TIMER5  	  	50
#define TEN_SEC			  	3000
#define DWT_CTRL    		(*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT  		(*(volatile uint32_t*)0xE0001004)
#define DEMCR       		(*(volatile uint32_t*)0xE000EDFC)
#define READ_Z_AXIS			0
extern I2C_Handle_t g_ds1307I2CHandle;
TIM_Handle_t Timer5_Handle;
extern int16_t connectionHandler[2];
USBD_HandleTypeDef hUsbDeviceFS;
extern TIM_HandleTypeDef        htim6;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
/*
 * Embedded Systems Programming on ARM Cortex-M3/M4 Processor lecture 79

 * Section 4.4 of Cortex -M4 Devices Generic User Guide
*/

void TIM5_init(void)
{
	Timer5_Handle.pTIMx = TIM5;
	if(((RCC->CFGR >> 2) & 0x3) == 0)
		Timer5_Handle.Prescalar = 15999;
	else
		Timer5_Handle.Prescalar = 47999;
	//Timer5_Handle.Reload_Val = 1000;
	timer_set_ms(&Timer5_Handle, LED_PERIOD_MS);
	timer_init(&Timer5_Handle);
}	

void dwt_init(void) 
{
    DEMCR |= (1 << 24);     // Enable trace and debug
    DWT_CTRL |= 1;          // Enable the cycle counter
    DWT_CYCCNT = 0;         // Reset the counter
}

void dwt_delay_us(uint32_t us) 
{
    uint32_t cycles = us * 16;  // 16 MHz = 16 cycles per microsecond
    uint32_t start = DWT_CYCCNT;
    while ((DWT_CYCCNT - start) < cycles);
}

void dwt_delay_ms(uint32_t ms) 
{
    while (ms--) 
	{
        dwt_delay_us(1000);
    }
}


//semihosting init function
extern void initialise_monitor_handles(void);
uint32_t *pNVIC_ISPR1 = (uint32_t*)0xE000E204;
volatile uint8_t start_cnt = 0;
volatile uint8_t id_cnt = 0;
volatile int8_t start_flag = 0;
volatile uint32_t one_min_cnt = ONE_MIN_CNT;
int dataAvailable = 0;
volatile uint8_t is_discoverable = 1;
void SystemClock_Config(void);
void HAL_EnableCompensationCell(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
uint8_t start_frame[4]= {0x10, 0x01, 0x10, 0x01};
uint8_t student_id[8]= {0x00, 0x01, 0x01, 0x01, 0x10, 0x11, 0x00, 0x11}; //ID is 5555
int16_t x;
int16_t y;
int16_t z;

/*
	part1
	https://docs.google.com/document/d/16My2VHtrzub1NycOldHII0mE8Xq-i0LB490brx3gpcc/edit?tab=t.0#heading=h.uu6cpqw6zjbz

	part2
	https://docs.google.com/document/d/17jYCwIollEr9pZSDwi219dB4Bd-WEjr2jH_KUeVLsSc/edit?tab=t.0#heading=h.lc5fcf8se9xg

	part3
	https://docs.google.com/document/d/1LF4pFkJgdbv1WE2SsijvQHM_YJ8eBC7BEsMf994AXnU/edit?tab=t.0#heading=h.lc5fcf8se9xg

https://github.com/ucsd-cse190b-w25/project-3-adding-low-energy-radio-communication-teamone?tab=readme-ov-file

For stm32f4 USB
https://github.com/STMicroelectronics/STM32CubeF4

target remote localhost:3333
monitor reset init
monitor flash write_image erase final_sh.elf
monitor arm semihosting enable
monitor reset halt
monitor resume

	*/

int twos_complement_to_signed(int value, int bitWidth) 
{
    if (value & (1U << (bitWidth - 1))) 
	{
        // Negative number
        value |= ~((1U << bitWidth) - 1);  // Sign-extend the value
    }
    return value;
}

uint8_t Read_movement(void)
{
	int res_x;
	int res_y;
#if READ_Z_AXIS
	int res_z;
#endif
	LIS3DSH_read_xyz(&x, &y, &z);
	res_x = twos_complement_to_signed(x, 16);
	res_y = twos_complement_to_signed(y, 16);
#if READ_Z_AXIS
	res_z = twos_complement_to_signed(z, 16);
#endif
	
	if(res_x > ACC_TH_X || res_x  < -ACC_TH_X
		|| res_y > ACC_TH_Y || res_y  < -ACC_TH_Y
#if READ_Z_AXIS
		|| res_z > ACC_TH_Z || res_z  < -ACC_TH_Z
#endif
	  )
	{
		one_min_cnt = ONE_MIN_CNT;
		return 1;
	}
	return 0;
}

#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_NO_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC

#define OTG_FS_OverCurrent_Pin GPIO_PIN_NO_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
GPIO_Handle_t	USB_FS_GPIO;
void USB_FS_GPIO_INIT(void)
{
    USB_FS_GPIO.pGPIOx = OTG_FS_PowerSwitchOn_GPIO_Port;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinNumber = OTG_FS_PowerSwitchOn_Pin;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    GPIO_Init(&USB_FS_GPIO);
	GPIO_WriteToOutputPin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, 1);

    USB_FS_GPIO.pGPIOx = OTG_FS_OverCurrent_GPIO_Port;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinNumber = OTG_FS_OverCurrent_Pin;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&USB_FS_GPIO);
}

int main(void)
{
	dwt_init();
	int lost_cnt_sec = 0;
	//initialise_monitor_handles();

	HAL_Init();
	//Set_PLL_Clock();
	SystemClock_Config();
	//HAL_EnableCompensationCell();

	//USB_FS_GPIO_INIT();

	if(USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
	{
		//Error_Handler();
		printf("fail 1\n");
	}else printf("ok\n");
	
	if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
	{
		//Error_Handler();
		printf("fail 2\n");
	}else printf("ok\n");

	if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
	{
		//Error_Handler();
		printf("fail 3\n");
	}else printf("ok\n");

	if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
	{
		//Error_Handler();
		printf("fail 4\n");
	}else printf("ok\n");

	while(1);
	return 0;

	
	//
	dwt_init();
	LIS3DSH_init();
	led_init();
	
	xnucleo_init();

	GPIO_WriteToOutputPin(BLE_GPIO_PORT,BLE_RST_Pin,GPIO_PIN_RESET);
	dwt_delay_ms(10);
	GPIO_WriteToOutputPin(BLE_GPIO_PORT,BLE_RST_Pin,GPIO_PIN_SET);

	ble_init();
	dwt_delay_ms(10);
	GPIO_WriteToOutputPin(LED_GPIO_PORT, LED_GPIO_BLUE, 0);
	TIM5_init();
    // /* Manually trigger TIM5 interrupt */
	//*pNVIC_ISPR1 |= (1 << (IRQNO_TIMER5 % 32));

	uint8_t nonDiscoverable = 0;

	while(1)
	{
		if(one_min_cnt)
		{
			setDiscoverability(0);
			Read_movement();
		}
		else
		{
			//dwt_delay_ms(30);
			if(!is_discoverable)
			{
				setDiscoverability(1);
				//leds_set(1);
			}	
			if(!nonDiscoverable && GPIO_ReadFromInputPin(BLE_GPIO_PORT, BLE_INT_Pin))
			{
				catchBLE();
			}
			else
			{
				int i = TEN_SEC;
				int move_flag = 1;				
				while (i--) 
				{
					dwt_delay_ms(1);
					if(Read_movement())
					{
						lost_cnt_sec = 0;
						move_flag = 0;
						GPIO_WriteToOutputPin(LED_GPIO_PORT, LED_GPIO_GREEN, 0);
						disconnectBLE();
						break;
					}
    			}
				if(move_flag)
				{
					lost_cnt_sec += (TEN_SEC/1000);
					unsigned char test_str[] = "lost for     sec";
					test_str[9] = (lost_cnt_sec / 100) +'0';
					test_str[10] = (lost_cnt_sec / 10) +'0';
					test_str[11] = (lost_cnt_sec % 10)+'0';
					updateCharValue(NORDIC_UART_SERVICE_HANDLE, WRITE_CHAR_HANDLE, 0, sizeof(test_str), test_str);	
				}
			}
			// Wait for interrupt, only uncomment if low power is needed
			//__WFI();
		}
	}
	return 0;
}


void TIM5_IRQHandler(void)
{
	if(one_min_cnt > 0)
	{
		one_min_cnt--;
		GPIO_ToggleOutputPin(LED_GPIO_PORT, LED_GPIO_RED);
	}
	TIM_IRQHandling(&Timer5_Handle);
 	// Timer5_Handle.pTIMx->SR &= ~(1<<0);
}

void TIM6_DAC_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim6);
}

void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */
  //printf("otg fs\n");
  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

void EXTI9_5_IRQHandler(void)
{
	dataAvailable=1;
	GPIO_IRQHandling(BLE_INT_Pin); //clear the pending event from exti line
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	return;
}

void SystemClock_Config(void)
{
  printf("set clock 1\n");
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    //Error_Handler();
	printf("fail\n");
  }
  printf("set clock 2\n");
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != 0x00)
  {
    //Error_Handler();
	printf("fail\n");
  }
  printf("set clock ok\n");
}

void HAL_EnableCompensationCell(void)
{
  *(volatile uint32_t *)CMPCR_CMP_PD_BB = (uint32_t)ENABLE;
}


void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);             /* only values 0..7 are used          */

  reg_value  =  SCB->AIRCR;                                                   /* read old register configuration    */
  reg_value &= ~((uint32_t)(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk)); /* clear bits to change               */
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
                (PriorityGroupTmp << 8U)                      );              /* Insert write key and priorty group */
  SCB->AIRCR =  reg_value;
}


void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) 
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}
// #endif