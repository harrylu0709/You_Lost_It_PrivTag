

#include <stdio.h>
#include "LIS3DSH.h"
#include "led.h"
#include "ble.h"

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
/*
 * Embedded Systems Programming on ARM Cortex-M3/M4 Processor lecture 79

 * Section 4.4 of Cortex -M4 Devices Generic User Guide
*/

void TIM5_init(void)
{
	Timer5_Handle.pTIMx = TIM5;
	Timer5_Handle.Prescalar = 15999;
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

int main(void)
{
	int lost_cnt_sec = 0;
	//return 0;
	//initialise_monitor_handles();
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

void EXTI9_5_IRQHandler(void)
{
	dataAvailable=1;
	GPIO_IRQHandling(BLE_INT_Pin); //clear the pending event from exti line
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	return;
}
// #endif