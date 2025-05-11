/*
 * ble.c
 *
 *  Created on: 18 gen 2021
 *      Author: UTPM9 (modifications by ADS)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ble.h"
#include "ble_commands.h"
#include "led.h"
/*
01 0c fc 08 00 06 aa 00 00 e1 80 02
01 8a fc 03 01 00 07
01 86 fc 1a 01 00 8a fc 80 ff 01 20 59 72 00 08 d2 ff 01 20 00 07 07 10 00 40 e2 01 00 01
01 02 fd 13 02 UUID 01 07 (primary service)


tx: 01 04 fd 19 0c 00 02 9e ca dc 24 0e ca dc 24 0e e5 a9 e0 93 f3 a3 b5 03 00 40 6e 14 10 00 00 10 01
rx: 01 04 fd 19 0c 00 02 9e ca dc 24 0e ca dc 24 0e e5 a9 e0 93 f3 a3 b5 02 00 40 6e 14 0C 00 00 10 01
01 0f fc 02 01 04
01 09 20 20 00*34
01 83 fc 1a 00 00 08 00 10 00 00 0d 09 42 6c 75 65 4e 52 47 5f 43 68 61 74 00 00 00 00 00

*/
uint8_t EVENT_STATUP_DATA[] = {0x04, 0xff, 0x03, 0x01, 0x00, 0x01};

uint8_t ACI_GATT_INIT[] = {0x01, 0x01, 0xfd, 0x00};
uint8_t ACI_GATT_INIT_COMPLETE[] = {0x04, 0x0e, 0x04, 0x01, 0x01, 0xfd, 0x00};

//uint8_t ACI_GAP_INIT[]={0x01,0x8a,0xfc,0x03,0x01,0x00,0x0d};
uint8_t ACI_GAP_INIT[] = {0x01, 0x8a, 0xfc, 0x03, 0x01, 0x00, 0x07};
uint8_t ACI_GAP_INIT_COMPLETE[] = {0x04, 0x0e, 0x0a, 0x01, 0x8a, 0xfc, 0x00};
uint8_t GAP_SERVICE_HANDLE[2];
uint8_t GAP_CHAR_NAME_HANDLE[2];
uint8_t GAP_CHAR_APP_HANDLE[2];

uint8_t ACI_GATT_UPDATE_CHAR_VALUE[] = {0x01, 0x06, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t ACI_GATT_UPDATE_CHAR_COMPLETE[] = {0x04, 0x0e, 0x04, 0x01, 0x06, 0xfd, 0x00};

//uint8_t ACI_GAP_SET_AUTH[]={0x01,0x86,0xfc,0x1a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x10,0x00,0x40,0xe2,0x01,0x00,0x00};
uint8_t ACI_GAP_SET_AUTH[] = {0x01, 0x86, 0xfc, 0x1a, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x10, 0x00, 0x40, 0xe2, 0x01, 0x00, 0x01};

uint8_t ACI_GAP_SET_AUTH_RESP[] = {0x04, 0x0e, 0x04, 0x01, 0x86, 0xfc, 0x00};
//uint8_t ACI_GAP_SET_AUTH_RESP[]={0x04,0x0e,0x04,0x01,0x86,0xfc,0x12};

uint8_t ACI_HAL_SET_TX_POWER_LEVEL[] = {0x01, 0x0f, 0xfc, 0x02, 0x01, 0x04};
uint8_t ACI_HAL_SET_TX_POWER_LEVEL_COMPLETE[] = {0x04, 0x0e, 0x04, 0x01, 0x0f, 0xfc, 0x00};

uint8_t ACI_HAL_SET_STANDBY[] = {0x01, 0x13, 0xfc, 0x00};
uint8_t ACI_HAL_SET_STANDBY_COMPLETE[] = {0x04, 0x0e, 0x04, 0x01, 0x13, 0xfc, 0x00};
uint8_t HCI_LE_SET_SCAN_RESPONSE_DATA[] = {0x01, 0x09, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t HCI_LE_SET_SCAN_RESPONSE_DATA_COMPLETE[] = {0x04, 0x0e, 0x04, 0x01, 0x09, 0x20, 0x00};

uint8_t ACI_GAP_SET_DISCOVERABLE[] = {0x01, 0x83, 0xfc, 0xff, 0x00, 0x40, 0x06, 0x40, 0x06, 0x01, 0x00, 0xff, 0x09};
uint8_t ACI_GAP_SET_DISCOVERABLE_COMPLETE[] = {0x04, 0x0e, 0x04, 0x01, 0x83, 0xfc, 0x00};

uint8_t ADD_PRIMARY_SERVICE[] = {0x01, 0x02, 0xFD, 0x13, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x05, 0x00, 0x01, 0x09}; //3C 60 bytes il massimo di memoria per il servizio
uint8_t ADD_PRIMARY_SERVICE_COMPLETE[] = {0x04, 0x0e, 0x06, 0x01, 0x02, 0xFD, 0x00};
//			14 10 00 00 10 01
uint8_t ADD_CUSTOM_CHAR[] = {0x01, 0x04, 0xFD, 0x19, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x10, 0x01};
//uint8_t ADD_CUSTOM_CHAR[]={0x01,0x04,0xFD,0x1A,0xff,0xff,0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x05,0x00,0x14,0x00,0x00,0x00,0x01,0x10,0x01};

uint8_t ADD_CUSTOM_CHAR_COMPLETE_UART[] = {0x04, 0x0e, 0x06, 0x01, 0x04, 0xFD, 0x00};
//uint8_t ADD_CUSTOM_CHAR_COMPLETE_UART[]={0x04,0x0e,0x06,0x01,0x04,0xFD,0x00};
uint8_t ADD_CUSTOM_CHAR_COMPLETE[] = {0x04, 0x0e, 0x04, 0x01, 0x06, 0xFD, 0x00};
//uint8_t ADD_CUSTOM_CHAR_COMPLETE[]={0x04,0x0e,0x04,0x01,0x04,0xFD,0x00};

//uint8_t ADD_CUSTOM_CHAR_COMPLETE_connect[0={0x04,0x3e,0x13,0x01,0x00,0x01,0x08};

uint8_t UPDATE_CHAR[] = {0x01, 0x06, 0xFD, 0x09, 0xff, 0xff, 0xff, 0xff, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};

uint8_t DISCONNECT[] = {0x01, 0x06, 0x04, 0x03}; // TODO - fill this in
uint8_t EVENT_DISCONNECTED[] = {0x04, 0x05, 0x04, 0x00};
uint8_t EVENT_DISCONNECT_PENDING[] = {0x04, 0x0F, 0x04, 0x00, 0x01, 0x06, 0x04};

//uint8_t EVENT_CONNECTED[] = {0x04,0x3E,0x13,0x01,0x00,0x01,0x08};
uint8_t EVENT_CONNECTED[] = {0x04, 0x3E, 0x13, 0x01, 0x00};
uint8_t EVENT_GATT_CHANGED[] = {0x04, 0xFF, 0x0B, 0x01, 0x0C};

uint8_t ACI_GAP_SET_NON_DISCOVERABLE[] = {0x01, 0x81, 0xFC, 0x00}; // TODO - fill this in
uint8_t ACI_GAP_SET_NON_DISCOVERABLE_COMPLETE[] = {0x04, 0x0E, 0x04, 0x01, 0x81, 0xFC, 0x00}; // TODO - fill this in

extern int dataAvailable;
extern void dwt_delay_ms(uint32_t ms);

// Device name sent in BLE advertisement packets
uint8_t deviceName[] = {'P', 'r', 'i', 'v', 'T', 'a', 'g'};

uint8_t buffer[255];

// NORDIC UART Service
uint8_t UUID_NORDIC_UART_SERVICE[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};

//uint8_t UUID_NORDIC_UART_SERVICE[]={0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe0,0xf2,0x73,0xd9};

uint8_t NORDIC_UART_SERVICE_HANDLE[2];

// NORDIC TX UART Characteristic
uint8_t UUID_CHAR_WRITE[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E};


//uint8_t UUID_CHAR_WRITE[]={0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
uint8_t WRITE_CHAR_HANDLE[2];

// NORDIC RX UART Characteristic
uint8_t UUID_CHAR_READ[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E};
//uint8_t UUID_CHAR_READ[]={0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};

uint8_t READ_CHAR_HANDLE[2];

uint16_t stackInitCompleteFlag = 0;
uint8_t * rxEvent;
int16_t connectionHandler[2] = {-1, -1}; // Little Endian Format for connection handler
uint8_t flag = 0;
/**
 * Initializes the BLE module with appropriate settings
 */
GPIO_Handle_t BLE;
SPI_Handle_t  SPI2Handle;
void ble_gpio_init()
{
	BLE.pGPIOx = BLE_GPIO_PORT;
	BLE.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	BLE.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	BLE.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	BLE.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	BLE.GPIO_PinConfig.GPIO_PinNumber = BLE_RST_Pin;
	GPIO_Init(&BLE);

	// LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	// LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	// LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	BLE.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	BLE.GPIO_PinConfig.GPIO_PinNumber = BLE_CS_Pin;
	GPIO_Init(&BLE);


	BLE.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	BLE.GPIO_PinConfig.GPIO_PinNumber = BLE_INT_Pin;
	GPIO_Init(&BLE);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
}

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = SPI2_SCK;
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = SPI2_MOSI;
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = SPI2_MISO;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV4;    //generate 2MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;
	SPI_Init(&SPI2Handle);
}
void xnucleo_init()
{
	ble_gpio_init();

	SPI2_GPIOInits();

	SPI2_Inits();

	SPI_SSIConfig(SPI2, ENABLE);

}

void SPI_TransmitReceive(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint8_t *pRxBuffer, uint32_t Len, uint32_t RxLen)
{
    while (Len > 0)
    {
        // Wait until TXE is set
        while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        // 1. Check DFF bit for 8-bit or 16-bit mode
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            // 16-bit
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            Len -= 2;

            // Wait until RXNE is set
            while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

            *((uint16_t*)pRxBuffer) = pSPIx->DR;

            pTxBuffer += 2;
            pRxBuffer += 2;
        }
        else
        {
            // 8-bit
            pSPIx->DR = *pTxBuffer;
            Len--;

            // Wait until RXNE is set
            while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

            *pRxBuffer = pSPIx->DR;

            pTxBuffer++;
            pRxBuffer++;
        }
    }
}

void ble_init()
{
	// uint8_t tx_dummy[] = {0x0b, 0x00, 0x00, 0x00, 0x00};
	// uint8_t rx_dummy[5];
	// SPI_PeripheralControl(SPI2, ENABLE);
	// GPIO_WriteToOutputPin(BLE_GPIO_PORT, BLE_CS_Pin, 0);

	// //SPI2 in this case, it could change according to the board
	// //we send a byte containing a request of reading followed by 4 dummy bytes
	
	// SPI_TransmitReceive(SPI2, tx_dummy, rx_dummy, 5, 5);
	// GPIO_WriteToOutputPin(BLE_GPIO_PORT, BLE_CS_Pin, 1);
	// SPI_PeripheralControl(SPI2, DISABLE);
	printf("ble init\n");
	//fetching the reset event
	rxEvent = (uint8_t *)malloc(EVENT_STARTUP_SIZE);
	int res;

	while (!dataAvailable);
	res = fetchBleEvent(rxEvent, EVENT_STARTUP_SIZE);
	printf("fetchble\n");
	if (res == BLE_OK)
	{
		res = checkEventResp(rxEvent, EVENT_STATUP_DATA, EVENT_STARTUP_SIZE);
		if (res == BLE_OK)
		{

			stackInitCompleteFlag |= 0x01;
		}
	}
	dwt_delay_ms(10);
	free(rxEvent);
	printf("gatt_init\n");
	//INIT GATT
	if (BLE_command(ACI_GATT_INIT, sizeof(ACI_GATT_INIT), ACI_GATT_INIT_COMPLETE, sizeof(ACI_GATT_INIT_COMPLETE), 0) == BLE_OK)
	{
		stackInitCompleteFlag |= 0x02;

	}
	free(rxEvent);
	printf("gap_init\n");
	//INIT GAP, actually the handle that i get is a GATT handle of a service, will change the name later
	if (BLE_command(ACI_GAP_INIT, sizeof(ACI_GAP_INIT), ACI_GAP_INIT_COMPLETE, sizeof(ACI_GAP_INIT_COMPLETE), 3) == BLE_OK)
	{
		stackInitCompleteFlag |= 0x04;
		memcpy(GAP_SERVICE_HANDLE, rxEvent + 7, 2);
		memcpy(GAP_CHAR_NAME_HANDLE, rxEvent + 9, 2);
		memcpy(GAP_CHAR_APP_HANDLE, rxEvent + 11, 2);
	}
	free(rxEvent);
	printf("set gap service\n");
	//SET THE NAME OF THE BOARD IN THE SERVICE CREATED AUTOMATICALLY
	updateCharValue(GAP_SERVICE_HANDLE, GAP_CHAR_NAME_HANDLE, 0, sizeof(deviceName), deviceName);
	stackInitCompleteFlag |= 0x08;
	free(rxEvent);
	printf("set gap auth\n");
	//INIT AUTH
	if (BLE_command(ACI_GAP_SET_AUTH, sizeof(ACI_GAP_SET_AUTH), ACI_GAP_SET_AUTH_RESP, sizeof(ACI_GAP_SET_AUTH_RESP), 0) == BLE_OK)
	{
		stackInitCompleteFlag |= 0x10;
		//HAL_GPIO_WritePin(GPIOD,LD6_Pin,GPIO_PIN_SET);
	}
	free(rxEvent);

	//SET_TX_LEVEL
	printf("set tx power\n");
	if (BLE_command(ACI_HAL_SET_TX_POWER_LEVEL, sizeof(ACI_HAL_SET_TX_POWER_LEVEL), ACI_HAL_SET_TX_POWER_LEVEL_COMPLETE, sizeof(ACI_HAL_SET_TX_POWER_LEVEL_COMPLETE), 0) == BLE_OK)
	{
		stackInitCompleteFlag |= 0x20;

	}
	free(rxEvent);

	//SET SCAN RESPONSE DATA
	printf("set scan response\n");
	if (BLE_command(HCI_LE_SET_SCAN_RESPONSE_DATA, sizeof(HCI_LE_SET_SCAN_RESPONSE_DATA), HCI_LE_SET_SCAN_RESPONSE_DATA_COMPLETE, sizeof(HCI_LE_SET_SCAN_RESPONSE_DATA_COMPLETE), 0) == BLE_OK)
	{
		stackInitCompleteFlag |= 0x40;

	}
	free(rxEvent);

	//This will start the advertisment,
	setConnectable();

	//add the nordic UART service

	addService(UUID_NORDIC_UART_SERVICE, NORDIC_UART_SERVICE_HANDLE, SET_ATTRIBUTES(7)); //SET_ATTRIBUTES(1+2+3*2+3+3));//1 atribute service +2 attribute char readable+3*(2 NOTIFYABLE READABLE charachteristics)
	addCharacteristic(UUID_CHAR_WRITE, WRITE_CHAR_HANDLE, NORDIC_UART_SERVICE_HANDLE, 20, NOTIFIBLE, 0x00, 0, 16, 1);
	//add the nordic UART charachteristics
	printf("tx\n");
	addCharacteristic(UUID_CHAR_READ, READ_CHAR_HANDLE, NORDIC_UART_SERVICE_HANDLE, 20, (WRITABLE | WRITE_WITHOUT_RESP), 0x00, 0x01, 16, 1);
	printf("rx\n");


	printf("flag = %d\n", stackInitCompleteFlag);
	if (stackInitCompleteFlag == 255)
	{
		//turn on led blue if everything was fine
		GPIO_WriteToOutputPin(BLE_GPIO_PORT, LED_GPIO_BLUE, 1);
	}
	return;
}

void standbyBle()
{
	//STANDBY MODE
	if (BLE_command(ACI_HAL_SET_STANDBY, sizeof(ACI_HAL_SET_STANDBY), ACI_HAL_SET_STANDBY_COMPLETE, sizeof(ACI_HAL_SET_STANDBY_COMPLETE), 0) == BLE_OK)
	{
	}
	free(rxEvent);
}

int fetchBleEvent(uint8_t * container, int size)
{
	//printf("start fetch\n");
	uint8_t master_header[] = {0x0b, 0x00, 0x00, 0x00, 0x00};
	uint8_t slave_header[5];

	//Wait until it is available an event coming from the BLE module (GPIO PIN COULD CHANGE ACCORDING TO THE BOARD)
	if (GPIO_ReadFromInputPin(BLE_GPIO_PORT, BLE_INT_Pin))
	{

		dwt_delay_ms(5);
		SPI_PeripheralControl(SPI2, ENABLE);

		//SPI2 in this case, it could change according to the board
		//we send a byte containing a request of reading followed by 4 dummy bytes
		GPIO_WriteToOutputPin(BLE_GPIO_PORT, BLE_CS_Pin, 0);
		dwt_delay_ms(1);
		SPI_TransmitReceive(SPI2, master_header, slave_header, 5, 5);
		GPIO_WriteToOutputPin(BLE_GPIO_PORT, BLE_CS_Pin, 1);
		SPI_PeripheralControl(SPI2, DISABLE);
	
		//PIN_CS of SPI2 LOW
		SPI_PeripheralControl(SPI2, ENABLE);

		//SPI2 in this case, it could change according to the board
		//we send a byte containing a request of reading followed by 4 dummy bytes
		GPIO_WriteToOutputPin(BLE_GPIO_PORT, BLE_CS_Pin, 0);
		dwt_delay_ms(1);
		SPI_TransmitReceive(SPI2, master_header, slave_header, 5, 5);
		GPIO_WriteToOutputPin(BLE_GPIO_PORT, BLE_CS_Pin, 1);
		SPI_PeripheralControl(SPI2, DISABLE);
		int j;
		for(j=0;j<5;j++)
		{
			printf("%x ",slave_header[j]);
		}
		printf(" 1\n");
		dwt_delay_ms(1);

		SPI_PeripheralControl(SPI2, ENABLE);
		GPIO_WriteToOutputPin(BLE_GPIO_PORT, BLE_CS_Pin, 0);

		SPI_TransmitReceive(SPI2, master_header, slave_header, 5, 5);

		//let's get the size of data available
		int dataSize;
		dataSize = (slave_header[3] | slave_header[4] << 8);
		for(j=0;j<5;j++)
		{
			printf("%x ",slave_header[j]);
		}
		printf(" 2\n");
		printf("datasize=%d\n",dataSize);
		int i;
		char dummy = 0xff;

		if (dataSize > size)
		{
			dataSize = size;
		}

		if (dataSize > 0)
		{
			//let's fill the get the bytes availables and insert them into the container variable
			for (i = 0; i < dataSize; i++)
			{
				SPI_TransmitReceive(SPI2, (uint8_t *)&dummy, container + i, 1, 1);

			}
			GPIO_WriteToOutputPin(BLE_GPIO_PORT, BLE_CS_Pin, 1);
			SPI_PeripheralControl(SPI2, DISABLE);
		}
		else
		{
			GPIO_WriteToOutputPin(BLE_GPIO_PORT, BLE_CS_Pin, 1);
			SPI_PeripheralControl(SPI2, DISABLE);
			return -1;
		}

		//let's stop the SPI2
		dataAvailable = 0;
		return BLE_OK;
	}
	else
	{
		return -2;
	}
}


int checkEventResp(uint8_t * event, uint8_t * reference, int size)
{
	int j = 0;
	// if(flag ==1)
	// {
	// 	for(j=0;j<size;j++){
	// 		printf("%x %x\n", event[j], reference[j]);
	// 	}
	// 	printf("----------\n");
	// }

	for (j = 0; j < size; j++)
	{
		//printf("%x %x\n", event[j], reference[j]);
		if (event[j] != reference[j])
		{
			//printf("diff %x %x %d\n", event[j], reference[j], j);
			return -1;
		}
	}

	return BLE_OK;
}

void sendCommand(uint8_t * command, int size)
{

	uint8_t master_header[] = {0x0a, 0x00, 0x00, 0x00, 0x00};
	uint8_t slave_header[5];

	int result;

	do
	{
		
		SPI_PeripheralControl(SPI2, ENABLE);
		GPIO_WriteToOutputPin(BLE_GPIO_PORT, BLE_CS_Pin, 0);

		//wait until it is possible to write
		//while(!dataAvailable);
		SPI_TransmitReceive(SPI2, master_header, slave_header, 5, 5);
		int bufferSize = (slave_header[2] << 8 | slave_header[1]);
		if (bufferSize >= size)
		{
			SPI_SendData(SPI2, command, size);
			result = 0;
		}
		else
		{
			result = -1;
		}
		GPIO_WriteToOutputPin(BLE_GPIO_PORT, BLE_CS_Pin, 1);
		SPI_PeripheralControl(SPI2, DISABLE);
		dataAvailable = 0;
	} while (result != 0);

}

void catchBLE(uint8_t * byte1, uint8_t * byte2)
{

	int result = fetchBleEvent(buffer, 127);
	if (result == BLE_OK)
	{
		flag = 1;
		printf("ok\n");
		if (checkEventResp(buffer, EVENT_DISCONNECTED, 3) == BLE_OK)
		{
			printf("disconnect\n");
			//setConnectable();
		}
		if (checkEventResp(buffer, EVENT_CONNECTED, 5) == BLE_OK)
		{
			printf("connect\n");
			// Little Endian Format
			*(connectionHandler) = buffer[5];
			*(connectionHandler + 1) = buffer[6];
		}
		if (checkEventResp(buffer, EVENT_GATT_CHANGED, 5) == BLE_OK)
		{
			printf("gatt\n");
			*(connectionHandler) = buffer[5];
			*(connectionHandler + 1) = buffer[6];
		}
	}
	else
	{
		printf("fail\n");
		//something bad is happening if I am here
	}
}

void setConnectable()
{
	uint8_t * rxEvent;
	//Start advertising
	uint8_t * localname;
	int res;
	localname = (uint8_t *)malloc(sizeof(deviceName) + 5); //carattere di terminazione+listauid+slavetemp
	memcpy(localname, deviceName, sizeof(deviceName));
	localname[sizeof(deviceName) + 1] = 0x00;
	localname[sizeof(deviceName) + 2] = 0x00;
	localname[sizeof(deviceName) + 3] = 0x00;
	localname[sizeof(deviceName) + 4] = 0x00;
	localname[sizeof(deviceName)] = 0x00;


	ACI_GAP_SET_DISCOVERABLE[11] = sizeof(deviceName) + 1;
	ACI_GAP_SET_DISCOVERABLE[3] = sizeof(deviceName) + 5 + sizeof(ACI_GAP_SET_DISCOVERABLE) - 4;

	uint8_t * discoverableCommand;
	discoverableCommand = (uint8_t *)malloc(sizeof(ACI_GAP_SET_DISCOVERABLE) + sizeof(deviceName) + 5);
	memcpy(discoverableCommand, ACI_GAP_SET_DISCOVERABLE, sizeof(ACI_GAP_SET_DISCOVERABLE));
	memcpy(discoverableCommand + sizeof(ACI_GAP_SET_DISCOVERABLE), localname, sizeof(deviceName) + 5);

	sendCommand(discoverableCommand, sizeof(deviceName) + 5 + sizeof(ACI_GAP_SET_DISCOVERABLE));
	rxEvent = (uint8_t *)malloc(7);
	while (!dataAvailable);
	res = fetchBleEvent(rxEvent, 7);
	if (res == BLE_OK)
	{
		res = checkEventResp(rxEvent, ACI_GAP_SET_DISCOVERABLE_COMPLETE, 7);
		if (res == BLE_OK)
		{
			stackInitCompleteFlag |= 0x80;
		}
	}

	free(rxEvent);
	free(discoverableCommand);
	free(localname);
	dwt_delay_ms(10);
}

/**
 * @brief Sends a BLE command and processes the response event.
 *
 * This function transmits a command to the BLE module, waits for the BLE interrupt pin to
 * signal that a response is available, retrieves the event data, and then checks the event
 * response.
 *
 * @param command Pointer to the buffer containing the command to be sent.
 * @param size Size of the command buffer.
 * @param result Pointer to the buffer where the response result will be stored.
 * @param sizeRes Expected size of the response result.
 * @param returnHandles Number of handles expected in the response (each handle occupies 2 bytes).
 * @return int Returns BLE_OK if the command was successfully executed and the event response is valid,
 *             or an error code if something went wrong.
 */
int BLE_command(uint8_t * command, int size, uint8_t * result, int sizeRes, int returnHandles)
{
	int response;

	//if(size == 28 && command[3] == 0x18) result[1]=0x3e;
	sendCommand(command, size);
	rxEvent = (uint8_t *)malloc(sizeRes + 2 * returnHandles);

	long contatore = 0;
	while (!GPIO_ReadFromInputPin(BLE_GPIO_PORT, BLE_INT_Pin))
	{
		contatore++;
		if (contatore > 30000)
		{
			break;
		}
	}

	response = fetchBleEvent(rxEvent, sizeRes + returnHandles * 2);
	// for(i = 0; i<size;i++)
	// {
	// 	printf("%x ",command[i]);
	// }
	// printf("\n");
	if (response == BLE_OK)
	{
		response = checkEventResp(rxEvent, result, sizeRes);
		//printf("ble ok\n");
	}
	//dwt_delay_ms(10);
	if (response != BLE_OK)
	{
		// int j;
		// for(j=0;j<sizeRes;j++){
		// 	printf("0x%02X 0x%02X\n", rxEvent[j], result[j]);
		// 	 if(rxEvent[j]!=result[j])
		// 	 {
		// 		printf("diff 0x%02X 0x%02X %d\n", rxEvent[j], result[j], j);
		// 		//break;
		// 	 }
		//  }


		// printf("s=%d ",size);
		// for(j= 0; j<size;j++)
		// 	printf("0x%02X ",command[j]);
		// {
		// }
		// printf("\n");
	}
	// if(response==BLE_OK && size == 16) HAL_GPIO_WritePin(GPIOD,LD6_Pin,GPIO_PIN_SET);
	return response;
}

void addService(uint8_t * UUID, uint8_t * handle, int attributes)
{
	//memcpy
	memcpy(ADD_PRIMARY_SERVICE + 5, UUID, 16);
	ADD_PRIMARY_SERVICE[21] = 0x01;
	ADD_PRIMARY_SERVICE[22] = attributes;
	if (BLE_command(ADD_PRIMARY_SERVICE, sizeof(ADD_PRIMARY_SERVICE), ADD_PRIMARY_SERVICE_COMPLETE, sizeof(ADD_PRIMARY_SERVICE_COMPLETE), 1) == BLE_OK)
	{
		handle[0] = rxEvent[7];
		handle[1] = rxEvent[8];
	}
	free(rxEvent);
}

void addCharacteristic(uint8_t * UUID, uint8_t * handleChar, uint8_t * handleService, uint8_t maxsize, uint8_t proprieties, uint8_t secPermissions, uint8_t gattEvtMask, uint8_t encryKeySize, uint8_t isVariable)
{
	memcpy(ADD_CUSTOM_CHAR + 7, UUID, 16);

	//  ADD_CUSTOM_CHAR[4]= handleService[0];
	//  ADD_CUSTOM_CHAR[5]= handleService[1];
	ADD_CUSTOM_CHAR[4] = 0x0C;
	ADD_CUSTOM_CHAR[5] = 0x00;
	ADD_CUSTOM_CHAR[23] = maxsize;
	ADD_CUSTOM_CHAR[24] = proprieties;


	ADD_CUSTOM_CHAR[25] = secPermissions;

	ADD_CUSTOM_CHAR[26] = gattEvtMask;
	ADD_CUSTOM_CHAR[27] = encryKeySize;
	ADD_CUSTOM_CHAR[28] = isVariable;


	if (BLE_command(ADD_CUSTOM_CHAR, sizeof(ADD_CUSTOM_CHAR), ADD_CUSTOM_CHAR_COMPLETE_UART, sizeof(ADD_CUSTOM_CHAR_COMPLETE_UART), 1) == BLE_OK)
	{
		handleChar[0] = rxEvent[7];
		handleChar[1] = rxEvent[8];
		printf("work %x %x\n", rxEvent[7], rxEvent[8]);
	}
	else
		printf("fail\n");
	free(rxEvent);
}

void updateCharValue(uint8_t * handleService, uint8_t * handleChar, int offset, int size, uint8_t * data)
{
	UPDATE_CHAR[3] = size + 6;
	UPDATE_CHAR[4] = handleService[0];
	UPDATE_CHAR[5] = handleService[1];
	UPDATE_CHAR[6] = handleChar[0];
	UPDATE_CHAR[7] = handleChar[1];
	UPDATE_CHAR[8] = offset;
	UPDATE_CHAR[9] = size;

	uint8_t * commandComplete;
	commandComplete = (uint8_t *)malloc(10 + size);
	memcpy(commandComplete, UPDATE_CHAR, 10);
	memcpy(commandComplete + 10, data, size);

	printf("%x\n",commandComplete[1]);
	BLE_command(commandComplete, 10 + size, ADD_CUSTOM_CHAR_COMPLETE, sizeof(ADD_CUSTOM_CHAR_COMPLETE), 0);

	free(commandComplete);
	free(rxEvent);
}

/**
 * @brief Disconnects the peripheral from the central
*/
void disconnectBLE()
{
	if (connectionHandler[0] == -1 && connectionHandler[1] == -1)
	{
		// should not be -1
		return;
	}
	uint8_t command[7];
	memcpy(command, DISCONNECT, 4);
	command[4] = connectionHandler[0];
	command[5] = connectionHandler[1];
	command[6] = 0x13;
	if (BLE_command(command, sizeof(command), EVENT_DISCONNECT_PENDING, 7, 0) == BLE_OK)
	{
		int result = fetchBleEvent(buffer, 127);
		if (result == BLE_OK)
		{
			if (checkEventResp(buffer, EVENT_DISCONNECTED, 4) == BLE_OK)
			{
				//setConnectable();
				connectionHandler[0] = -1;
				connectionHandler[1] = -1;
			}
		}
		free(rxEvent);
	}
}

/**
 * DO NOT CHANGE FUNCTION definition
 * @brief Sets the discoverability of the peripheral
 * @param mode 0 => Non Discoverable, 1 => Discoverable
 * */
void setDiscoverability(uint8_t mode)
{
	if (mode == 1)
	{
		setConnectable();
	}
	else if (mode == 0)
	{
		if (BLE_command(ACI_GAP_SET_NON_DISCOVERABLE, sizeof(ACI_GAP_SET_NON_DISCOVERABLE), ACI_GAP_SET_NON_DISCOVERABLE_COMPLETE, sizeof(ACI_GAP_SET_NON_DISCOVERABLE_COMPLETE), 0) == BLE_OK)
		{
		}
		free(rxEvent);
	}
	else
	{
		// Do nothing
	}
}
