

#include <stdio.h>
#include "LIS3DSH.h"
#include "led.h"
#include "ble.h"

#define SYSTICK_TIM_CLK   16000000UL //16MHz
#define LED_PERIOD_MS	  50
#define ONE_MIN_CNT	 	  100
#define IRQNO_TIMER5  	  50
#define DWT_CTRL    (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT  (*(volatile uint32_t*)0xE0001004)
#define DEMCR       (*(volatile uint32_t*)0xE000EDFC)
extern I2C_Handle_t g_ds1307I2CHandle;
TIM_Handle_t Timer5_Handle;

/*
 * Embedded Systems Programming on ARM Cortex-M3/M4 Processor lecture 79

 * Section 4.4 of Cortex -M4 Devices Generic User Guide
*/
#if ENABLE_SysTick
void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;// if 1 second, then tick_hz = 1, if 1 millisecond, then tick_hz = 1000

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:

    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}
#endif
void close_systick_timer(void)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);


    //do some settings
    *pSCSR &= ~(0x07);
}

void Systick_PriorityConfig(uint32_t IRQPriority)
{
	uint32_t *pSystick_Prioirty = (uint32_t*)0xE000ED20;

	*pSystick_Prioirty  |=  ( IRQPriority << 24);
}

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
    while (ms--) {
        dwt_delay_us(1000);
    }
}


//semihosting init function
extern void initialise_monitor_handles(void);
uint32_t *pNVIC_ISPR1 = (uint32_t*)0XE000E204;
volatile uint8_t start_cnt = 0;
volatile uint8_t id_cnt = 0;
volatile int8_t start_flag = 0;
volatile int8_t one_min_cnt = ONE_MIN_CNT;
int dataAvailable = 0;

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
    if (value & (1U << (bitWidth - 1))) {
        // Negative number
        value |= ~((1U << bitWidth) - 1);  // Sign-extend the value
    }
    return value;
}
uint8_t GPIO_ReadFromOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
   uint8_t value;

   value = (uint8_t )((pGPIOx->ODR  >> PinNumber) & 0x00000001 ) ;

   return value;
}
int main(void)
{
	int res_x;
	int res_y;
	int res_z;	

	initialise_monitor_handles();
	printf("hello world\n");

	dwt_init();
	LIS3DSH_init();
	led_init();
	//TIM5_init();

	xnucleo_init();
	// printf("xnucleo_init finish\n");
	//return 0;
	GPIO_WriteToOutputPin(BLE_GPIO_PORT,BLE_RST_Pin,GPIO_PIN_RESET);
	dwt_delay_ms(10);
	GPIO_WriteToOutputPin(BLE_GPIO_PORT,BLE_RST_Pin,GPIO_PIN_SET);
	// printf("mosi=%d\n",GPIO_ReadFromOutputPin(GPIOB, SPI2_MOSI));
	// printf("clk=%d\n",GPIO_ReadFromOutputPin(GPIOB, SPI2_SCK));
	// printf("miso=%d\n",GPIO_ReadFromOutputPin(GPIOB, SPI2_MISO));

	// printf("mosi=%d\n",GPIO_ReadFromInputPin(GPIOA, SPI1_MOSI));
	// printf("clk=%d\n",GPIO_ReadFromInputPin(GPIOA, SPI1_SCK));
	// printf("miso=%d\n",GPIO_ReadFromInputPin(GPIOA, SPI1_MISO));

	// printf("%d\n",GPIO_ReadFromOutputPin(GPIOE, 3));
	//return 0;
	
	ble_init();
	//return 0;
	printf("ble_init finish\n");
	dwt_delay_ms(10);

	uint8_t nonDiscoverable = 0;
	printf("init finish\n");
	while (1)
	{
		if(!nonDiscoverable && GPIO_ReadFromInputPin(BLE_GPIO_PORT,BLE_INT_Pin))
		{
			printf("1\n");
			catchBLE();
		}
		else
		{
			printf("2\n");
			dwt_delay_ms(1000);
			// Send a string to the NORDIC UART service, remember to not include the newline
			unsigned char test_str[] = "youlostit BLE test";
			//updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, sizeof(test_str)-1, test_str);
			//uint8_t test_str[20] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F','G','H','I','J'};
			updateCharValue(NORDIC_UART_SERVICE_HANDLE, WRITE_CHAR_HANDLE, 0, sizeof(test_str) - 1, test_str);

		}
		// Wait for interrupt, only uncomment if low power is needed
		//__WFI();
	}

	return 0;
    /* Manually trigger TIM5 interrupt */
    *pNVIC_ISPR1 |= (1 << (IRQNO_TIMER5 % 32));

	while(1)
	{
		if(one_min_cnt)
		{
			start_flag = 0;
			start_cnt = 0;
			id_cnt = 0;
			LIS3DSH_read_xyz(&x, &y, &z);
			res_x = twos_complement_to_signed(x, 16);
			res_y = twos_complement_to_signed(y, 16);
			res_z = twos_complement_to_signed(z, 16);
			if(	   res_x > LED_TH_X || res_x  < -LED_TH_X\
				|| res_y > LED_TH_Y || res_y  < -LED_TH_Y\
				|| res_z > LED_TH_Z || res_z  < -LED_TH_Z)
			{
				one_min_cnt = ONE_MIN_CNT;
			}
		}

	}
	
	return 0;
	//I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	//I2C_IRQPriorityConfig(IRQ_NO_I2C1_EV, NVIC_IRQ_PRI0);


// 	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	
// 	I2C_IRQPriorityConfig(IRQ_NO_I2C1_EV, NVIC_IRQ_PRI0);


// 	while(1);

 	return 0;
}


void TIM5_IRQHandler(void)
{
	if(one_min_cnt > 0)
	{
		one_min_cnt--;
	}
	else
	{
		if(start_flag == 0)
		{
			leds_set(start_frame[start_cnt++]);
			if(start_cnt == 4) start_flag = 1;
		}
		else if(start_flag == 1)
		{
			leds_set(student_id[id_cnt++]);
			if(id_cnt == 8)
			{
				start_flag = -1;
			} 
		}
		// else
		// 	one_min_cnt = ONE_MIN_CNT;
	}
	TIM_IRQHandling(&Timer5_Handle);
 	//Timer5_Handle.pTIMx->SR &= ~(1<<0);
}

void EXTI9_5_IRQHandler(void)
{
	dataAvailable=1;
	GPIO_IRQHandling(BLE_INT_Pin); //clear the pending event from exti line
}
// #if I2C_INT_ENABLE
// void I2C1_EV_IRQHandler(void)
// {
//     //printf("i2c1 ev\n");
// 	//printf("Handler %d\n",*(g_ds1307I2CHandle.pTxBuffer));
//     I2C_EV_IRQHandling(&g_ds1307I2CHandle);
// }

// void I2C1_ER_IRQHandler(void)
// {
//     //printf("i2c1 er\n");
//     I2C_ER_IRQHandling(&g_ds1307I2CHandle);
// }

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{

    if(AppEv == I2C_EV_TX_CMPLT)
    {
        //printf("Tx is complete\n");
		pI2CHandle->TxRxComplt = SET;
    }
    else if(AppEv == I2C_EV_RX_CMPLT)
    {
        //printf("Rx is complete\n");
        pI2CHandle->TxRxComplt = SET;
    }
    else if(AppEv == I2C_ERROR_AF)
    {
        printf("Error: Ack failure\n");

        // in master ack failure happens when slave fails to send ack for the byte
        // sent from master
        I2C_CloseSendData(pI2CHandle);

        // /generate stop condition to release bus
        I2C_GenerateStopCondition(I2C1);

        //Hang in infinite loop
        while(1);
    }
    else if(AppEv == I2C_ERROR_BERR)
    {
        printf("Error: bus error\n");
    }
    else if(AppEv == I2C_ERROR_ARLO)
    {
        printf("Error: arb error\n");
    }
    else if(AppEv == I2C_ERROR_OVR)
    {
        printf("Error: overrun error\n");
    }
    else if(AppEv == I2C_ERROR_TIMEOUT)
    {
        printf("Error: timeout error\n");
    }

}
// #endif

#if ENABLE_SysTick
void SysTick_Handler(void)
{
	RTC_time_t current_time;
	RTC_date_t current_date;

	ds1307_get_current_time(&current_time);

	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS){
		am_pm = (current_time.time_format) ? "PM" : "AM";
#ifndef PRINT_LCD
		printf("Current time = %s %s\n",time_to_string(&current_time),am_pm); // 04:25:41 PM
#else
		lcd_set_cursor(1, 1);
		lcd_print_string(time_to_string(&current_time));
		lcd_print_string(am_pm);
#endif

	}else{
#ifndef PRINT_LCD
		printf("Current time = %s\n",time_to_string(&current_time)); // 04:25:41
#else
		lcd_set_cursor(1, 1);
		lcd_print_string(time_to_string(&current_time));
#endif
	}

	ds1307_get_current_date(&current_date);

#ifndef PRINT_LCD
	printf("Current date = %s <%s>\n",date_to_string(&current_date), get_day_of_week(current_date.day));
#else
	lcd_set_cursor(2, 1);
	lcd_print_string(date_to_string(&current_date));
	lcd_print_char('<');
	lcd_print_string(get_day_of_week(current_date.day));
	lcd_print_char('>');
#endif
}
#endif
