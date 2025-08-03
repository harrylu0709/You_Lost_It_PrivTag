/*
 * stm3f407xx.h
 *
 *  Created on: Jan 29, 2019
 *      Author: admin
 */

#ifndef INC_STM3F407XX_H_
#define INC_STM3F407XX_H_

#include<stddef.h>
#include<stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))
#define ARRAY_LENGTH(a)  (sizeof(a)/sizeof(a[0]))

/**********************************START:Processor Specific Details **********************************/

/*!< USB registers base address */
#define USB_OTG_HS_PERIPH_BASE               0x40040000UL
#define USB_OTG_FS_PERIPH_BASE               0x50000000UL

#define USB_OTG_GLOBAL_BASE                  0x000UL
#define USB_OTG_DEVICE_BASE                  0x800UL
#define USB_OTG_IN_ENDPOINT_BASE             0x900UL
#define USB_OTG_OUT_ENDPOINT_BASE            0xB00UL
#define USB_OTG_EP_REG_SIZE                  0x20UL
#define USB_OTG_HOST_BASE                    0x400UL
#define USB_OTG_HOST_PORT_BASE               0x440UL
#define USB_OTG_HOST_CHANNEL_BASE            0x500UL
#define USB_OTG_HOST_CHANNEL_SIZE            0x20UL
#define USB_OTG_PCGCCTL_BASE                 0xE00UL
#define USB_OTG_FIFO_BASE                    0x1000UL
#define USB_OTG_FIFO_SIZE                    0x1000UL

#define UID_BASE                     0x1FFF7A10UL           /*!< Unique device ID register base address */
#define FLASHSIZE_BASE               0x1FFF7A22UL           /*!< FLASH Size register base address       */
#define PACKAGE_BASE                 0x1FFF7BF0UL           /*!< Package size register base address     */
#define USB_OTG_FS          ((USB_OTG_GlobalTypeDef *) USB_OTG_FS_PERIPH_BASE)
#define USB_OTG_HS          ((USB_OTG_GlobalTypeDef *) USB_OTG_HS_PERIPH_BASE)


#define RCC_CFGR_PPRE1_Pos                 (10U)                               
#define RCC_CFGR_PPRE1_Msk                 (0x7UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00001C00 */
#define RCC_CFGR_PPRE1                     RCC_CFGR_PPRE1_Msk                  /*!< PRE1[2:0] bits (APB1 prescaler) */
#define RCC_CFGR_PPRE1_0                   (0x1UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00000400 */
#define RCC_CFGR_PPRE1_1                   (0x2UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00000800 */
#define RCC_CFGR_PPRE1_2                   (0x4UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00001000 */

/*!< PPRE2 configuration */
#define RCC_CFGR_PPRE2_Pos                 (13U)                               
#define RCC_CFGR_PPRE2_Msk                 (0x7UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x0000E000 */
#define RCC_CFGR_PPRE2                     RCC_CFGR_PPRE2_Msk                  /*!< PRE2[2:0] bits (APB2 prescaler) */
#define RCC_CFGR_PPRE2_0                   (0x1UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x00002000 */
#define RCC_CFGR_PPRE2_1                   (0x2UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x00004000 */
#define RCC_CFGR_PPRE2_2                   (0x4UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x00008000 */

#define RCC_PLLCFGR_PLLSRC_HSE_Pos         (22U)                               
#define RCC_PLLCFGR_PLLSRC_HSE_Msk         (0x1UL << RCC_PLLCFGR_PLLSRC_HSE_Pos) /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC_HSE             RCC_PLLCFGR_PLLSRC_HSE_Msk    

#define PWR_CR_VOS_Pos         (14U)                                           
#define PWR_CR_VOS_Msk         (0x1UL << PWR_CR_VOS_Pos)                        /*!< 0x00004000 */
#define PWR_CR_VOS             PWR_CR_VOS_Msk                                  /*!< VOS bit (Regulator voltage scaling output selection) */


#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define RCC_APB1ENR_PWREN_Pos              (28U)                               
#define RCC_APB1ENR_PWREN_Msk              (0x1UL << RCC_APB1ENR_PWREN_Pos)     /*!< 0x10000000 */
#define RCC_APB1ENR_PWREN                  RCC_APB1ENR_PWREN_Msk 


#define FLASH_ACR_LATENCY_Pos          (0U)
#define FLASH_ACR_LATENCY_Msk          (0x7UL << FLASH_ACR_LATENCY_Pos)         /*!< 0x00000007 */
#define FLASH_ACR_LATENCY              FLASH_ACR_LATENCY_Msk
#define FLASH_ACR_ICEN_Pos             (9U)                                    
#define FLASH_ACR_ICEN_Msk             (0x1UL << FLASH_ACR_ICEN_Pos)            /*!< 0x00000200 */
#define FLASH_ACR_ICEN                 FLASH_ACR_ICEN_Msk                      
#define FLASH_ACR_DCEN_Pos             (10U)                                   
#define FLASH_ACR_DCEN_Msk             (0x1UL << FLASH_ACR_DCEN_Pos)            /*!< 0x00000400 */
#define FLASH_ACR_DCEN                 FLASH_ACR_DCEN_Msk

#define FLASH_ACR_PRFTEN_Pos           (8U)                                    
#define FLASH_ACR_PRFTEN_Msk           (0x1UL << FLASH_ACR_PRFTEN_Pos)          /*!< 0x00000100 */
#define FLASH_ACR_PRFTEN               FLASH_ACR_PRFTEN_Msk

#define NVIC_PRIORITYGROUP_4         0x00000003U /*!< 4 bits for pre-emption priority
                                                      0 bits for subpriority */
#define  TICK_INT_PRIORITY            0U   /*!< tick interrupt priority */
#define SYSCFG_OFFSET             (SYSCFG_BASEADDR - PERIPH_BASE)
#define CMPCR_OFFSET              (SYSCFG_OFFSET + 0x20U) 
#define SYSCFG_CMPCR_CMP_PD_Pos              (0U)    
#define CMP_PD_BIT_NUMBER         SYSCFG_CMPCR_CMP_PD_Pos
#define CMPCR_CMP_PD_BB           (uint32_t)(PERIPH_BB_BASE + (CMPCR_OFFSET * 32U) + (CMP_PD_BIT_NUMBER * 4U))

#define SCB_AIRCR_VECTKEY_Pos              16U                                            /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_VECTKEY_Msk              (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos)            /*!< SCB AIRCR: VECTKEY Mask */
#define SCB_AIRCR_PRIGROUP_Pos              8U                                            /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk             (7UL << SCB_AIRCR_PRIGROUP_Pos)                /*!< SCB AIRCR: PRIGROUP Mask */
#define SCB_AIRCR_VECTKEY_Pos              16U                                            /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_VECTKEY_Msk              (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos)            /*!< SCB AIRCR: VECTKEY Mask */
#define SCB_AIRCR_PRIGROUP_Pos              8U                                            /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk             (7UL << SCB_AIRCR_PRIGROUP_Pos)                /*!< SCB AIRCR: PRIGROUP Mask */
#define SCB_SCR_SLEEPDEEP_Pos               2U                                            /*!< SCB SCR: SLEEPDEEP Position */
#define SCB_SCR_SLEEPDEEP_Msk              (1UL << SCB_SCR_SLEEPDEEP_Pos)                 /*!< SCB SCR: SLEEPDEEP Mask */
#define SCB_SCR_SLEEPONEXIT_Pos             1U                                            /*!< SCB SCR: SLEEPONEXIT Position */
#define SCB_SCR_SLEEPONEXIT_Msk            (1UL << SCB_SCR_SLEEPONEXIT_Pos)               /*!< SCB SCR: SLEEPONEXIT Mask */


/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4
#define   __NVIC_PRIO_BITS        NO_PR_BITS_IMPLEMENTED

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR						0x08000000U   		/*!<explain this macro briefly here  */
#define SRAM1_BASEADDR						0x20000000U  		/*!<explain this macro briefly here  */
#define SRAM2_BASEADDR						0x2001C000U 		/*!<explain this macro briefly here  */
#define ROM_BASEADDR						0x1FFF0000U
#define SRAM 								SRAM1_BASEADDR


/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR 						0x40000000U
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR						0x40010000U
#define AHB1PERIPH_BASEADDR						0x40020000U
#define AHB2PERIPH_BASEADDR						0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO : Complete for all other peripherals
 */
//RF 2.3 memory map
#define GPIOA_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR                     (AHB1PERIPH_BASEADDR + 0x3800)
#define FLASH_R_BASE          (AHB1PERIPH_BASEADDR + 0x3C00UL)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO : Complete for all other peripherals
 */
#define PWR_BASE                (APB1PERIPH_BASEADDR + 0x7000UL)
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)
#define TIM6_BASEADDR						(APB1PERIPH_BASEADDR + 0x1000)

#define TIM5_BASEADDR						(APB1PERIPH_BASEADDR + 0x0C00)
#define TIM2_BASEADDR						(APB1PERIPH_BASEADDR + 0x0000)
/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR        				(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)





/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

typedef struct
{
	__vo uint32_t MODER;                        /*!< GPIO port mode register,                    	Address offset: 0x00      */
	__vo uint32_t OTYPER;                       /*!< TODO,     										Address offset: 0x04      */
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];					 /*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;



/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
  __vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
  __vo uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
  __vo uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
  __vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
  __vo uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
  __vo uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
  __vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
  __vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
  __vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
  __vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
  __vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
  __vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
  __vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
  __vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
  __vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
  __vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
  __vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
  __vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
  __vo uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */

} RCC_RegDef_t;



/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;    /*!< Give a short description,          	  	    Address offset: 0x00 */
	__vo uint32_t EMR;    /*!< TODO,                						Address offset: 0x04 */
	__vo uint32_t RTSR;   /*!< TODO,  									     Address offset: 0x08 */
	__vo uint32_t FTSR;   /*!< TODO, 										Address offset: 0x0C */
	__vo uint32_t SWIER;  /*!< TODO,  									   Address offset: 0x10 */
	__vo uint32_t PR;     /*!< TODO,                   					   Address offset: 0x14 */

}EXTI_RegDef_t;


/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CRCPR;      /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t RXCRCR;     /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t TXCRCR;     /*!< TODO,     										Address offset: 0x18 */
	__vo uint32_t I2SCFGR;    /*!< TODO,     										Address offset: 0x1C */
	__vo uint32_t I2SPR;      /*!< TODO,     										Address offset: 0x20 */
} SPI_RegDef_t;


/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;       /*!< Give a short description,                    Address offset: 0x00      */
	__vo uint32_t PMC;          /*!< TODO,     									  Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
	__vo uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */
	uint32_t      RESERVED2[2];  /*!<                                             Reserved, 0x24-0x28 	    */
	__vo uint32_t CFGR;         /*!< TODO                                         Address offset: 0x2C   	*/
} SYSCFG_RegDef_t;


/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;

/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t BRR;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t CR3;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t GTPR;       /*!< TODO,     										Address offset: 0x18 */
} USART_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t CR2;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t SMCR;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t DIER;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t SR;        /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t EGR;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t CCMR1;       /*!< TODO,     										Address offset: 0x18 */
	__vo uint32_t CCMR2;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t CCER;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t CNT;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t PSC;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t ARR;        /*!< TODO,     										Address offset: 0x10 */
	uint32_t RESERVED1;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t CCR1;       /*!< TODO,     										Address offset: 0x18 */
	__vo uint32_t CCR2;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t CCR3;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t CCR4;        /*!< TODO,     										Address offset: 0x08 */
	uint32_t RESERVED2;         /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t DCR;       /*!< TODO,     										Address offset: 0x18 */
	__vo uint32_t DMAR;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t OR2;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t OR5;        /*!< TODO,     										Address offset: 0x08 */
} TIM_RegDef_t;

typedef struct
{
  volatile uint32_t GOTGCTL;              /*!< USB_OTG Control and Status Register          000h */
  volatile uint32_t GOTGINT;              /*!< USB_OTG Interrupt Register                   004h */
  volatile uint32_t GAHBCFG;              /*!< Core AHB Configuration Register              008h */
  volatile uint32_t GUSBCFG;              /*!< Core USB Configuration Register              00Ch */
  volatile uint32_t GRSTCTL;              /*!< Core Reset Register                          010h */
  volatile uint32_t GINTSTS;              /*!< Core Interrupt Register                      014h */
  volatile uint32_t GINTMSK;              /*!< Core Interrupt Mask Register                 018h */
  volatile uint32_t GRXSTSR;              /*!< Receive Sts Q Read Register                  01Ch */
  volatile uint32_t GRXSTSP;              /*!< Receive Sts Q Read & POP Register            020h */
  volatile uint32_t GRXFSIZ;              /*!< Receive FIFO Size Register                   024h */
  volatile uint32_t DIEPTXF0_HNPTXFSIZ;   /*!< EP0 / Non Periodic Tx FIFO Size Register     028h */
  volatile uint32_t HNPTXSTS;             /*!< Non Periodic Tx FIFO/Queue Sts reg           02Ch */
  uint32_t Reserved30[2];             /*!< Reserved                                     030h */
  volatile uint32_t GCCFG;                /*!< General Purpose IO Register                  038h */
  volatile uint32_t CID;                  /*!< User ID Register                             03Ch */
  uint32_t  Reserved40[48];           /*!< Reserved                                0x40-0xFF */
  volatile uint32_t HPTXFSIZ;             /*!< Host Periodic Tx FIFO Size Reg               100h */
  volatile uint32_t DIEPTXF[0x0F];        /*!< dev Periodic Transmit FIFO                        */
} USB_OTG_GlobalTypeDef;

/** 
  * @brief USB_OTG_device_Registers
  */
typedef struct 
{
  volatile uint32_t DCFG;            /*!< dev Configuration Register   800h */
  volatile uint32_t DCTL;            /*!< dev Control Register         804h */
  volatile uint32_t DSTS;            /*!< dev Status Register (RO)     808h */
  uint32_t Reserved0C;           /*!< Reserved                     80Ch */
  volatile uint32_t DIEPMSK;         /*!< dev IN Endpoint Mask         810h */
  volatile uint32_t DOEPMSK;         /*!< dev OUT Endpoint Mask        814h */
  volatile uint32_t DAINT;           /*!< dev All Endpoints Itr Reg    818h */
  volatile uint32_t DAINTMSK;        /*!< dev All Endpoints Itr Mask   81Ch */
  uint32_t  Reserved20;          /*!< Reserved                     820h */
  uint32_t Reserved9;            /*!< Reserved                     824h */
  volatile uint32_t DVBUSDIS;        /*!< dev VBUS discharge Register  828h */
  volatile uint32_t DVBUSPULSE;      /*!< dev VBUS Pulse Register      82Ch */
  volatile uint32_t DTHRCTL;         /*!< dev threshold                830h */
  volatile uint32_t DIEPEMPMSK;      /*!< dev empty msk                834h */
  volatile uint32_t DEACHINT;        /*!< dedicated EP interrupt       838h */
  volatile uint32_t DEACHMSK;        /*!< dedicated EP msk             83Ch */
  uint32_t Reserved40;           /*!< dedicated EP mask            840h */
  volatile uint32_t DINEP1MSK;       /*!< dedicated EP mask            844h */
  uint32_t  Reserved44[15];      /*!< Reserved                 844-87Ch */
  volatile uint32_t DOUTEP1MSK;      /*!< dedicated EP msk             884h */
} USB_OTG_DeviceTypeDef;

/** 
  * @brief USB_OTG_IN_Endpoint-Specific_Register
  */
typedef struct 
{
  volatile uint32_t DIEPCTL;           /*!< dev IN Endpoint Control Reg    900h + (ep_num * 20h) + 00h */
  uint32_t Reserved04;             /*!< Reserved                       900h + (ep_num * 20h) + 04h */
  volatile uint32_t DIEPINT;           /*!< dev IN Endpoint Itr Reg        900h + (ep_num * 20h) + 08h */
  uint32_t Reserved0C;             /*!< Reserved                       900h + (ep_num * 20h) + 0Ch */
  volatile uint32_t DIEPTSIZ;          /*!< IN Endpoint Txfer Size         900h + (ep_num * 20h) + 10h */
  volatile uint32_t DIEPDMA;           /*!< IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h */
  volatile uint32_t DTXFSTS;           /*!< IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h */
  uint32_t Reserved18;             /*!< Reserved  900h+(ep_num*20h)+1Ch-900h+ (ep_num * 20h) + 1Ch */
} USB_OTG_INEndpointTypeDef;

/** 
  * @brief USB_OTG_OUT_Endpoint-Specific_Registers
  */
typedef struct 
{
  volatile uint32_t DOEPCTL;       /*!< dev OUT Endpoint Control Reg           B00h + (ep_num * 20h) + 00h */
  uint32_t Reserved04;         /*!< Reserved                               B00h + (ep_num * 20h) + 04h */
  volatile uint32_t DOEPINT;       /*!< dev OUT Endpoint Itr Reg               B00h + (ep_num * 20h) + 08h */
  uint32_t Reserved0C;         /*!< Reserved                               B00h + (ep_num * 20h) + 0Ch */
  volatile uint32_t DOEPTSIZ;      /*!< dev OUT Endpoint Txfer Size            B00h + (ep_num * 20h) + 10h */
  volatile uint32_t DOEPDMA;       /*!< dev OUT Endpoint DMA Address           B00h + (ep_num * 20h) + 14h */
  uint32_t Reserved18[2];      /*!< Reserved B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch */
} USB_OTG_OUTEndpointTypeDef;

/** 
  * @brief USB_OTG_Host_Mode_Register_Structures
  */
typedef struct 
{
  volatile uint32_t HCFG;             /*!< Host Configuration Register          400h */
  volatile uint32_t HFIR;             /*!< Host Frame Interval Register         404h */
  volatile uint32_t HFNUM;            /*!< Host Frame Nbr/Frame Remaining       408h */
  uint32_t Reserved40C;           /*!< Reserved                             40Ch */
  volatile uint32_t HPTXSTS;          /*!< Host Periodic Tx FIFO/ Queue Status  410h */
  volatile uint32_t HAINT;            /*!< Host All Channels Interrupt Register 414h */
  volatile uint32_t HAINTMSK;         /*!< Host All Channels Interrupt Mask     418h */
} USB_OTG_HostTypeDef;

/** 
  * @brief USB_OTG_Host_Channel_Specific_Registers
  */
typedef struct
{
  volatile uint32_t HCCHAR;           /*!< Host Channel Characteristics Register    500h */
  volatile uint32_t HCSPLT;           /*!< Host Channel Split Control Register      504h */
  volatile uint32_t HCINT;            /*!< Host Channel Interrupt Register          508h */
  volatile uint32_t HCINTMSK;         /*!< Host Channel Interrupt Mask Register     50Ch */
  volatile uint32_t HCTSIZ;           /*!< Host Channel Transfer Size Register      510h */
  volatile uint32_t HCDMA;            /*!< Host Channel DMA Address Register        514h */
  uint32_t Reserved[2];           /*!< Reserved                                      */
} USB_OTG_HostChannelTypeDef;


typedef struct
{
  uint32_t PLLState;   /*!< The new state of the PLL.
                            This parameter can be a value of @ref RCC_PLL_Config                      */

  uint32_t PLLSource;  /*!< RCC_PLLSource: PLL entry clock source.
                            This parameter must be a value of @ref RCC_PLL_Clock_Source               */

  uint32_t PLLM;       /*!< PLLM: Division factor for PLL VCO input clock.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 63    */

  uint32_t PLLN;       /*!< PLLN: Multiplication factor for PLL VCO output clock.
                            This parameter must be a number between Min_Data = 50 and Max_Data = 432 
                            except for STM32F411xE devices where the Min_Data = 192 */

  uint32_t PLLP;       /*!< PLLP: Division factor for main system clock (SYSCLK).
                            This parameter must be a value of @ref RCC_PLLP_Clock_Divider             */

  uint32_t PLLQ;       /*!< PLLQ: Division factor for OTG FS, SDIO and RNG clocks.
                            This parameter must be a number between Min_Data = 2 and Max_Data = 15    */
}RCC_PLLInitTypeDef;

typedef struct
{
  uint32_t OscillatorType;       /*!< The oscillators to be configured.
                                      This parameter can be a value of @ref RCC_Oscillator_Type                   */

  uint32_t HSEState;             /*!< The new state of the HSE.
                                      This parameter can be a value of @ref RCC_HSE_Config                        */

  uint32_t LSEState;             /*!< The new state of the LSE.
                                      This parameter can be a value of @ref RCC_LSE_Config                        */

  uint32_t HSIState;             /*!< The new state of the HSI.
                                      This parameter can be a value of @ref RCC_HSI_Config                        */

  uint32_t HSICalibrationValue;  /*!< The HSI calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
                                       This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F */

  uint32_t LSIState;             /*!< The new state of the LSI.
                                      This parameter can be a value of @ref RCC_LSI_Config                        */

  RCC_PLLInitTypeDef PLL;        /*!< PLL structure parameters                                                    */
}RCC_OscInitTypeDef;

/**
  * @brief  RCC System, AHB and APB busses clock configuration structure definition
  */
typedef struct
{
  uint32_t ClockType;             /*!< The clock to be configured.
                                       This parameter can be a value of @ref RCC_System_Clock_Type      */

  uint32_t SYSCLKSource;          /*!< The clock source (SYSCLKS) used as system clock.
                                       This parameter can be a value of @ref RCC_System_Clock_Source    */

  uint32_t AHBCLKDivider;         /*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
                                       This parameter can be a value of @ref RCC_AHB_Clock_Source       */

  uint32_t APB1CLKDivider;        /*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

  uint32_t APB2CLKDivider;        /*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
                                       This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

}RCC_ClkInitTypeDef;


typedef struct
{
  volatile uint32_t CR;   /*!< PWR power control register,        Address offset: 0x00 */
  volatile uint32_t CSR;  /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_TypeDef;

typedef struct
{
  volatile uint32_t ACR;      /*!< FLASH access control register,   Address offset: 0x00 */
  volatile uint32_t KEYR;     /*!< FLASH key register,              Address offset: 0x04 */
  volatile uint32_t OPTKEYR;  /*!< FLASH option key register,       Address offset: 0x08 */
  volatile uint32_t SR;       /*!< FLASH status register,           Address offset: 0x0C */
  volatile uint32_t CR;       /*!< FLASH control register,          Address offset: 0x10 */
  volatile uint32_t OPTCR;    /*!< FLASH option control register ,  Address offset: 0x14 */
  volatile uint32_t OPTCR1;   /*!< FLASH option control register 1, Address offset: 0x18 */
} FLASH_TypeDef;


typedef struct
{
  volatile uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
  volatile uint32_t LOAD;                   /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
  volatile uint32_t VAL;                    /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
  volatile const uint32_t CALIB;                  /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_Type;

#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */
#define     __IO    volatile            



typedef struct
{
  __IM  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  __IOM uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  __IOM uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  __IOM uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  __IOM uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  __IOM uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  __IOM uint8_t  SHP[12U];               /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  __IOM uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  __IOM uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  __IOM uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  __IOM uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  __IOM uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  __IOM uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  __IOM uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  __IM  uint32_t PFR[2U];                /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  __IM  uint32_t DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  __IM  uint32_t ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  __IM  uint32_t MMFR[4U];               /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  __IM  uint32_t ISAR[5U];               /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
        uint32_t RESERVED0[5U];
  __IOM uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;

typedef struct
{
  __IO uint32_t CR;     /*!< DMA stream x configuration register      */
  __IO uint32_t NDTR;   /*!< DMA stream x number of data register     */
  __IO uint32_t PAR;    /*!< DMA stream x peripheral address register */
  __IO uint32_t M0AR;   /*!< DMA stream x memory 0 address register   */
  __IO uint32_t M1AR;   /*!< DMA stream x memory 1 address register   */
  __IO uint32_t FCR;    /*!< DMA stream x FIFO control register       */
} DMA_Stream_TypeDef;

typedef struct
{
  __IO uint32_t LISR;   /*!< DMA low interrupt status register,      Address offset: 0x00 */
  __IO uint32_t HISR;   /*!< DMA high interrupt status register,     Address offset: 0x04 */
  __IO uint32_t LIFCR;  /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
  __IO uint32_t HIFCR;  /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
} DMA_TypeDef;

typedef struct
{
  __IOM uint32_t ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        uint32_t RESERVED0[24U];
  __IOM uint32_t ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        uint32_t RSERVED1[24U];
  __IOM uint32_t ISPR[8U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        uint32_t RESERVED2[24U];
  __IOM uint32_t ICPR[8U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        uint32_t RESERVED3[24U];
  __IOM uint32_t IABR[8U];               /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
        uint32_t RESERVED4[56U];
  __IOM uint8_t  IP[240U];               /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
        uint32_t RESERVED5[644U];
  __OM  uint32_t STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
}  NVIC_Type;

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1  				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2  				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3  				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  				((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)
#define TIM2  				((TIM_RegDef_t*)TIM2_BASEADDR)
#define TIM5  				((TIM_RegDef_t*)TIM5_BASEADDR)
#define TIM6          ((TIM_RegDef_t *)TIM6_BASEADDR)
#define PWR           ((PWR_TypeDef *) PWR_BASE)
#define FLASH         ((FLASH_TypeDef *) FLASH_R_BASE)
#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define SysTick_BASE        (SCS_BASE +  0x0010UL) 
  #define NVIC_BASE           (SCS_BASE +  0x0100UL) 

#define SysTick       ((SysTick_Type   *)SysTick_BASE)   /*!< SysTick configuration struct */

#define NVIC                ((NVIC_Type*)NVIC_BASE)
#define SCB_BASE            (SCS_BASE +  0x0D00UL)  
#define SCB                 ((SCB_Type *)SCB_BASE ) 
/*
 * Clock Enable Macros for GPIOx peripherals
 * RCC registers 6.3
 */

#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))


/*
 * Clock Enable Macros for SPIx peripheralsbu
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCCK_EN()  (RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()  (RCC->APB1ENR |= (1 << 20))
#define USART6_PCCK_EN() (RCC->APB1ENR |= (1 << 5))


#define TIM2_PCLK_EN() (RCC->APB1ENR |= (1 << 0))
#define TIM5_PCLK_EN() (RCC->APB1ENR |= (1 << 3))

#define TIM2_PCLK_DI() (RCC->APB1ENR &= ~ (1 << 0))
#define TIM5_PCLK_DI() (RCC->APB1ENR &= ~ (1 << 3))
/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()    	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))


#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))
/*
 * Clock Disable Macros for SPIx peripherals
 */

/*
 * Clock Disable Macros for USARTx peripherals
 */


/*
 * Clock Disable Macros for SYSCFG peripheral
 */


/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)


/*
 *  returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7: \
								        (x == GPIOI)?8:0)


/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 * reference manual 12.2 table 63 Acronym - position
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84

#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_I2C2_EV     33
#define IRQ_NO_I2C2_ER     34
#define IRQ_NO_I2C3_EV     79
#define IRQ_NO_I2C3_ER     80
#define IRQ_NO_USART1	     37
#define IRQ_NO_USART2	     38
#define IRQ_NO_TIM2		     28 //APB1 45MHz
#define IRQ_NO_TIM5		     50 //APB1 45MHz
#define OTG_FS_IRQn        67
#define TIM6_DAC_IRQn       54
#define SysTick_IRQn -1

//#define IRQ_NO_USART3	    39
//#define IRQ_NO_UART4	    52
//#define IRQ_NO_UART5	    53
//#define IRQ_NO_USART6	    71


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0     0
#define NVIC_IRQ_PRI1     1
#define NVIC_IRQ_PRI2     2
#define NVIC_IRQ_PRI3     3
#define NVIC_IRQ_PRI4     4
#define NVIC_IRQ_PRI5     5
#define NVIC_IRQ_PRI6     6
#define NVIC_IRQ_PRI7     7
#define NVIC_IRQ_PRI8     8
#define NVIC_IRQ_PRI9     9
#define NVIC_IRQ_PRI10    10
#define NVIC_IRQ_PRI11    11
#define NVIC_IRQ_PRI12    12
#define NVIC_IRQ_PRI13    13
#define NVIC_IRQ_PRI14    14
#define NVIC_IRQ_PRI15    15


//some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET         RESET
#define FLAG_SET 			SET


/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10 //buffer interrupt enable, txe or rxne

/*
 * Bit position definitions I2C_OAR1
 * own address register
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 * clock control register
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_tim_driver.h"
#endif /* INC_STM3F407XX_H_ */
//
///*
// * stm3f407xx.h
// *
// *  Created on: Jan 29, 2019
// *      Author: admin
// */
//
//#ifndef INC_STM3F407XX_H_
//#define INC_STM3F407XX_H_
//
//#include<stddef.h>
//#include<stdint.h>
//
//#define __vo volatile
//#define __weak __attribute__((weak))
//
//
//
///**********************************START:Processor Specific Details **********************************/
///*
// * ARM Cortex Mx Processor NVIC ISERx register Addresses
// */
//
//#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
//#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
//#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
//#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )
//
//
///*
// * ARM Cortex Mx Processor NVIC ICERx register Addresses
// */
//#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
//#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
//#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
//#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)
//
//
///*
// * ARM Cortex Mx Processor Priority Register Address Calculation
// */
//#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)
//
///*
// * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
// */
//#define NO_PR_BITS_IMPLEMENTED  4
//
///*
// * base addresses of Flash and SRAM memories
// */
//
//#define FLASH_BASEADDR						0x08000000U   		/*!<explain this macro briefly here  */
//#define SRAM1_BASEADDR						0x20000000U  		/*!<explain this macro briefly here  */
//#define SRAM2_BASEADDR						0x2001C000U 		/*!<explain this macro briefly here  */
//#define ROM_BASEADDR						0x1FFF0000U
//#define SRAM 								SRAM1_BASEADDR
//
//
///*
// * AHBx and APBx Bus Peripheral base addresses
// */
//
//#define PERIPH_BASEADDR 						0x40000000U
//#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
//#define APB2PERIPH_BASEADDR						0x40010000U
//#define AHB1PERIPH_BASEADDR						0x40020000U
//#define AHB2PERIPH_BASEADDR						0x50000000U
//
///*
// * Base addresses of peripherals which are hanging on AHB1 bus
// * TODO : Complete for all other peripherals
// */
//
//#define GPIOA_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0000)
//#define GPIOB_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0400)
//#define GPIOC_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0800)
//#define GPIOD_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0C00)
//#define GPIOE_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1000)
//#define GPIOF_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1400)
//#define GPIOG_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1800)
//#define GPIOH_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1C00)
//#define GPIOI_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x2000)
//#define RCC_BASEADDR                     (AHB1PERIPH_BASEADDR + 0x3800)
///*
// * Base addresses of peripherals which are hanging on APB1 bus
// * TODO : Complete for all other peripherals
// */
//#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
//#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
//#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)
//
//#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
//#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)
//
//#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
//#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
//#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
//#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)
//
///*
// * Base addresses of peripherals which are hanging on APB2 bus
// * TODO : Complete for all other peripherals
// */
//#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)
//#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
//#define SYSCFG_BASEADDR        				(APB2PERIPH_BASEADDR + 0x3800)
//#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
//#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)
//
//
//
//
//
///**********************************peripheral register definition structures **********************************/
//
///*
// * Note : Registers of a peripheral are specific to MCU
// * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
// * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
// * Please check your Device RM
// */
//
//typedef struct
//{
//	__vo uint32_t MODER;                        /*!< GPIO port mode register,                    	Address offset: 0x00      */
//	__vo uint32_t OTYPER;                       /*!< TODO,     										Address offset: 0x04      */
//	__vo uint32_t OSPEEDR;
//	__vo uint32_t PUPDR;
//	__vo uint32_t IDR;
//	__vo uint32_t ODR;
//	__vo uint32_t BSRR;
//	__vo uint32_t LCKR;
//	__vo uint32_t AFR[2];					 /*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
//}GPIO_RegDef_t;
//
//
//
///*
// * peripheral register definition structure for RCC
// */
//typedef struct
//{
//  __vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
//  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
//  __vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
//  __vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
//  __vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
//  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
//  __vo uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
//  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
//  __vo uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
//  __vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
//  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
//  __vo uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
//  __vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
//  __vo uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
//  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
//  __vo uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
//  __vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
//  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
//  __vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
//  __vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
//  __vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
//  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
//  __vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
//  __vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
//  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
//  __vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
//  __vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
//  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
//  __vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
//  __vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
//  __vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
//  __vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
//  __vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
//  __vo uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */
//
//} RCC_RegDef_t;
//
//
//
///*
// * peripheral register definition structure for EXTI
// */
//typedef struct
//{
//	__vo uint32_t IMR;    /*!< Give a short description,          	  	    Address offset: 0x00 */
//	__vo uint32_t EMR;    /*!< TODO,                						Address offset: 0x04 */
//	__vo uint32_t RTSR;   /*!< TODO,  									     Address offset: 0x08 */
//	__vo uint32_t FTSR;   /*!< TODO, 										Address offset: 0x0C */
//	__vo uint32_t SWIER;  /*!< TODO,  									   Address offset: 0x10 */
//	__vo uint32_t PR;     /*!< TODO,                   					   Address offset: 0x14 */
//
//}EXTI_RegDef_t;
//
//
///*
// * peripheral register definition structure for SPI
// */
//typedef struct
//{
//	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
//	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
//	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x08 */
//	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x0C */
//	__vo uint32_t CRCPR;      /*!< TODO,     										Address offset: 0x10 */
//	__vo uint32_t RXCRCR;     /*!< TODO,     										Address offset: 0x14 */
//	__vo uint32_t TXCRCR;     /*!< TODO,     										Address offset: 0x18 */
//	__vo uint32_t I2SCFGR;    /*!< TODO,     										Address offset: 0x1C */
//	__vo uint32_t I2SPR;      /*!< TODO,     										Address offset: 0x20 */
//} SPI_RegDef_t;
//
//
///*
// * peripheral register definition structure for SYSCFG
// */
//typedef struct
//{
//	__vo uint32_t MEMRMP;       /*!< Give a short description,                    Address offset: 0x00      */
//	__vo uint32_t PMC;          /*!< TODO,     									  Address offset: 0x04      */
//	__vo uint32_t EXTICR[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
//	uint32_t      RESERVED1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
//	__vo uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */
//	uint32_t      RESERVED2[2];  /*!<                                             Reserved, 0x24-0x28 	    */
//	__vo uint32_t CFGR;         /*!< TODO                                         Address offset: 0x2C   	*/
//} SYSCFG_RegDef_t;
//
//
///*
// * peripheral register definition structure for I2C
// */
//typedef struct
//{
//  __vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
//  __vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
//  __vo uint32_t OAR1;       /*!< TODO,     										Address offset: 0x08 */
//  __vo uint32_t OAR2;       /*!< TODO,     										Address offset: 0x0C */
//  __vo uint32_t DR;         /*!< TODO,     										Address offset: 0x10 */
//  __vo uint32_t SR1;        /*!< TODO,     										Address offset: 0x14 */
//  __vo uint32_t SR2;        /*!< TODO,     										Address offset: 0x18 */
//  __vo uint32_t CCR;        /*!< TODO,     										Address offset: 0x1C */
//  __vo uint32_t TRISE;      /*!< TODO,     										Address offset: 0x20 */
//  __vo uint32_t FLTR;       /*!< TODO,     										Address offset: 0x24 */
//}I2C_RegDef_t;
//
///*
// * peripheral register definition structure for USART
// */
//typedef struct
//{
//	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x00 */
//	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x04 */
//	__vo uint32_t BRR;        /*!< TODO,     										Address offset: 0x08 */
//	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x0C */
//	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x10 */
//	__vo uint32_t CR3;        /*!< TODO,     										Address offset: 0x14 */
//	__vo uint32_t GTPR;       /*!< TODO,     										Address offset: 0x18 */
//} USART_RegDef_t;
//
///*
// * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
// */
//
//#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
//#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
//#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
//#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
//#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
//#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
//#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
//#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)
//#define GPIOI  				((GPIO_RegDef_t*)GPIOI_BASEADDR)
//
//#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
//#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
//#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
//
//
//#define SPI1  				((SPI_RegDef_t*)SPI1_BASEADDR)
//#define SPI2  				((SPI_RegDef_t*)SPI2_BASEADDR)
//#define SPI3  				((SPI_RegDef_t*)SPI3_BASEADDR)
//
//#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR)
//#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR)
//#define I2C3  				((I2C_RegDef_t*)I2C3_BASEADDR)
//
//#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
//#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
//#define USART3  			((USART_RegDef_t*)USART3_BASEADDR)
//#define UART4  				((USART_RegDef_t*)UART4_BASEADDR)
//#define UART5  				((USART_RegDef_t*)UART5_BASEADDR)
//#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)
//
///*
// * Clock Enable Macros for GPIOx peripherals
// */
//
//#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))
//#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
//#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
//#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
//#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
//#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
//#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
//#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
//#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))
//
//
///*
// * Clock Enable Macros for I2Cx peripherals
// */
//#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
//#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
//#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))
//
//
///*
// * Clock Enable Macros for SPIx peripheralsbu
// */
//#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
//#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
//#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
//#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))
//
//
///*
// * Clock Enable Macros for USARTx peripherals
// */
//#define USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 4))
//#define USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17))
//#define USART3_PCCK_EN() (RCC->APB1ENR |= (1 << 18))
//#define UART4_PCCK_EN()  (RCC->APB1ENR |= (1 << 19))
//#define UART5_PCCK_EN()  (RCC->APB1ENR |= (1 << 20))
//#define USART6_PCCK_EN() (RCC->APB1ENR |= (1 << 5))
//
///*
// * Clock Enable Macros for SYSCFG peripheral
// */
//#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))
//
//
///*
// * Clock Disable Macros for GPIOx peripherals
// */
//#define GPIOA_PCLK_DI()    	(RCC->AHB1ENR &= ~(1 << 0))
//#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
//#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
//#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
//#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
//#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
//#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
//#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
//#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))
//
///*
// * Clock Disable Macros for SPIx peripherals
// */
//#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
//#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
//#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))
///*
// * Clock Disable Macros for USARTx peripherals
// */
//
//
///*
// * Clock Disable Macros for SYSCFG peripheral
// */
//
//
///*
// *  Macros to reset GPIOx peripherals
// */
//#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
//#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
//#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
//#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
//#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
//#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
//#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
//#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
//#define GPIOI_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)
//
//
///*
// *  returns port code for given GPIOx base address
// */
///*
// * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
// */

//
//
///*
// * IRQ(Interrupt Request) Numbers of STM32F407x MCU
// * NOTE: update these macros with valid values according to your MCU
// * TODO: You may complete this list for other peripherals
// */
//
//#define IRQ_NO_EXTI0 		6
//#define IRQ_NO_EXTI1 		7
//#define IRQ_NO_EXTI2 		8
//#define IRQ_NO_EXTI3 		9
//#define IRQ_NO_EXTI4 		10
//#define IRQ_NO_EXTI9_5 		23
//#define IRQ_NO_EXTI15_10 	40
//#define IRQ_NO_SPI1			35
//#define IRQ_NO_SPI2         36
//#define IRQ_NO_SPI3         51
//#define IRQ_NO_SPI4
//#define IRQ_NO_I2C1_EV     31
//#define IRQ_NO_I2C1_ER     32
//#define IRQ_NO_USART1	    37
//#define IRQ_NO_USART2	    38
//#define IRQ_NO_USART3	    39
//#define IRQ_NO_UART4	    52
//#define IRQ_NO_UART5	    53
//#define IRQ_NO_USART6	    71
//
//
///*
// * macros for all the possible priority levels
// */
//#define NVIC_IRQ_PRI0    0
//#define NVIC_IRQ_PRI15    15
//
//
////some generic macros
//
//#define ENABLE 				1
//#define DISABLE 			0
//#define SET 				ENABLE
//#define RESET 				DISABLE
//#define GPIO_PIN_SET        SET
//#define GPIO_PIN_RESET      RESET
//#define FLAG_RESET         RESET
//#define FLAG_SET 			SET
//
//
///******************************************************************************************
// *Bit position definitions of SPI peripheral
// ******************************************************************************************/
///*
// * Bit position definitions SPI_CR1
// */
//#define SPI_CR1_CPHA     				 0
//#define SPI_CR1_CPOL      				 1
//#define SPI_CR1_MSTR     				 2
//#define SPI_CR1_BR   					 3
//#define SPI_CR1_SPE     				 6
//#define SPI_CR1_LSBFIRST   			 	 7
//#define SPI_CR1_SSI     				 8
//#define SPI_CR1_SSM      				 9
//#define SPI_CR1_RXONLY      		 	10
//#define SPI_CR1_DFF     			 	11
//#define SPI_CR1_CRCNEXT   			 	12
//#define SPI_CR1_CRCEN   			 	13
//#define SPI_CR1_BIDIOE     			 	14
//#define SPI_CR1_BIDIMODE      			15
//
///*
// * Bit position definitions SPI_CR2
// */
//#define SPI_CR2_RXDMAEN		 			0
//#define SPI_CR2_TXDMAEN				 	1
//#define SPI_CR2_SSOE				 	2
//#define SPI_CR2_FRF						4
//#define SPI_CR2_ERRIE					5
//#define SPI_CR2_RXNEIE				 	6
//#define SPI_CR2_TXEIE					7
//
//
///*
// * Bit position definitions SPI_SR
// */
//#define SPI_SR_RXNE						0
//#define SPI_SR_TXE				 		1
//#define SPI_SR_CHSIDE				 	2
//#define SPI_SR_UDR					 	3
//#define SPI_SR_CRCERR				 	4
//#define SPI_SR_MODF					 	5
//#define SPI_SR_OVR					 	6
//#define SPI_SR_BSY					 	7
//#define SPI_SR_FRE					 	8
//
///******************************************************************************************
// *Bit position definitions of I2C peripheral
// ******************************************************************************************/
///*
// * Bit position definitions I2C_CR1
// */
//#define I2C_CR1_PE						0
//#define I2C_CR1_NOSTRETCH  				7
//#define I2C_CR1_START 					8
//#define I2C_CR1_STOP  				 	9
//#define I2C_CR1_ACK 				 	10
//#define I2C_CR1_SWRST  				 	15
//
///*
// * Bit position definitions I2C_CR2
// */
//#define I2C_CR2_FREQ				 	0
//#define I2C_CR2_ITERREN				 	8
//#define I2C_CR2_ITEVTEN				 	9
//#define I2C_CR2_ITBUFEN 			    10
//
///*
// * Bit position definitions I2C_OAR1
// */
//#define I2C_OAR1_ADD0    				 0
//#define I2C_OAR1_ADD71 				 	 1
//#define I2C_OAR1_ADD98  			 	 8
//#define I2C_OAR1_ADDMODE   			 	15
//
///*
// * Bit position definitions I2C_SR1
// */
//
//#define I2C_SR1_SB 					 	0
//#define I2C_SR1_ADDR 				 	1
//#define I2C_SR1_BTF 					2
//#define I2C_SR1_ADD10 					3
//#define I2C_SR1_STOPF 					4
//#define I2C_SR1_RXNE 					6
//#define I2C_SR1_TXE 					7
//#define I2C_SR1_BERR 					8
//#define I2C_SR1_ARLO 					9
//#define I2C_SR1_AF 					 	10
//#define I2C_SR1_OVR 					11
//#define I2C_SR1_TIMEOUT 				14
//
///*
// * Bit position definitions I2C_SR2
// */
//#define I2C_SR2_MSL						0
//#define I2C_SR2_BUSY 					1
//#define I2C_SR2_TRA 					2
//#define I2C_SR2_GENCALL 				4
//#define I2C_SR2_DUALF 					7
//
///*
// * Bit position definitions I2C_CCR
// */
//#define I2C_CCR_CCR 					 0
//#define I2C_CCR_DUTY 					14
//#define I2C_CCR_FS  				 	15
//
///******************************************************************************************
// *Bit position definitions of USART peripheral
// ******************************************************************************************/
//
///*
// * Bit position definitions USART_CR1
// */
//#define USART_CR1_SBK					0
//#define USART_CR1_RWU 					1
//#define USART_CR1_RE  					2
//#define USART_CR1_TE 					3
//#define USART_CR1_IDLEIE 				4
//#define USART_CR1_RXNEIE  				5
//#define USART_CR1_TCIE					6
//#define USART_CR1_TXEIE					7
//#define USART_CR1_PEIE 					8
//#define USART_CR1_PS 					9
//#define USART_CR1_PCE 					10
//#define USART_CR1_WAKE  				11
//#define USART_CR1_M 					12
//#define USART_CR1_UE 					13
//#define USART_CR1_OVER8  				15
//
//
//
///*
// * Bit position definitions USART_CR2
// */
//#define USART_CR2_ADD   				0
//#define USART_CR2_LBDL   				5
//#define USART_CR2_LBDIE  				6
//#define USART_CR2_LBCL   				8
//#define USART_CR2_CPHA   				9
//#define USART_CR2_CPOL   				10
//#define USART_CR2_STOP   				12
//#define USART_CR2_LINEN   				14
//
//
///*
// * Bit position definitions USART_CR3
// */
//#define USART_CR3_EIE   				0
//#define USART_CR3_IREN   				1
//#define USART_CR3_IRLP  				2
//#define USART_CR3_HDSEL   				3
//#define USART_CR3_NACK   				4
//#define USART_CR3_SCEN   				5
//#define USART_CR3_DMAR  				6
//#define USART_CR3_DMAT   				7
//#define USART_CR3_RTSE   				8
//#define USART_CR3_CTSE   				9
//#define USART_CR3_CTSIE   				10
//#define USART_CR3_ONEBIT   				11
//
///*
// * Bit position definitions USART_SR
// */
//
//#define USART_SR_PE        				0
//#define USART_SR_FE        				1
//#define USART_SR_NE        				2
//#define USART_SR_ORE       				3
//#define USART_SR_IDLE       			4
//#define USART_SR_RXNE        			5
//#define USART_SR_TC        				6
//#define USART_SR_TXE        			7
//#define USART_SR_LBD        			8
//#define USART_SR_CTS        			9
//
//#include "stm32f407xx_gpio_driver.h"
//#include "stm32f407xx_spi_driver.h"
//
//
//#endif /* INC_STM3F407XX_H_ */
//
///////*
////// * stm32f407xx.h
////// *
////// *  Created on: Jun 22, 2024
////// *      Author: bo
////// */
//////
//////#ifndef INC_STM32F407XX_H_
//////#define INC_STM32F407XX_H_
//////
//////#include <stdint.h>
//////
//////
////////ARM Cortex MX Processor NVIC ISERx register address Interrupt Set-Enable Register
//////#define NVIC_ISER0 	((volatile uint32_t*)(0xE000E100))
//////#define NVIC_ISER1 	((volatile uint32_t*)(0xE000E104))
//////#define NVIC_ISER2 	((volatile uint32_t*)(0xE000E108))
//////#define NVIC_ISER3 	((volatile uint32_t*)(0xE000E10C))
//////
////////ARM Cortex MX Processor NVIC ICERx register address Interrupt Clear-Enable Register
//////#define NVIC_ICER0 	((volatile uint32_t*)(0XE000E180))
//////#define NVIC_ICER1 	((volatile uint32_t*)(0xE000E184))
//////#define NVIC_ICER2 	((volatile uint32_t*)(0xE000E188))
//////#define NVIC_ICER3 	((volatile uint32_t*)(0xE000E18C))
//////
//////#define NVIC_PR_BASE_ADDR ((volatile uint32_t*)(0xE000E400))
//////
//////#define NO_PR_BITS_IMPLEMENTED 4
//////
////////base address of FLASH and SRAM memories
//////#define FLASH_BASEADDR 0x08000000U
//////#define SRAM1_BASEADDR 0x20000000U
//////#define SRAM2_BASEADDR 0x2001C000U //SRAM1 + 112KB(112*1024)
//////#define ROM_BASEADDR   0x1FFF0000U
//////#define SRAM 		   SRAM1_BASEADDR
//////
//////// AHB and APB bus address
//////#define PERIPH_BASEADDR     0x40000000U
//////#define APB1PERIPH_BASEADDR PERIPH_BASE
//////#define	APB2PERIPH_BASEADDR 0x40010000U
//////#define AHB1PERIPH_BASEADDR 0x40020000U
//////#define AHB2PERIPH_BASEADDR 0x0000000U
//////
//////
////////base address of AHB1 bus
//////#define GPIOA_BASEADDR 	(AHB1PERIPH_BASEADDR + 0x0000)
//////#define GPIOB_BASEADDR 	(AHB1PERIPH_BASEADDR + 0x0400)
//////#define GPIOC_BASEADDR 	(AHB1PERIPH_BASEADDR + 0x0800)
//////#define GPIOD_BASEADDR 	(AHB1PERIPH_BASEADDR + 0x0C00)
//////#define GPIOE_BASEADDR 	(AHB1PERIPH_BASEADDR + 0x1000)
//////#define GPIOF_BASEADDR 	(AHB1PERIPH_BASEADDR + 0x1400)
//////#define GPIOG_BASEADDR 	(AHB1PERIPH_BASEADDR + 0x1800)
//////#define GPIOH_BASEADDR 	(AHB1PERIPH_BASEADDR + 0x1C00)
//////#define GPIOI_BASEADDR 	(AHB1PERIPH_BASEADDR + 0x2000)
//////#define RCC_BASEADDR 	(AHB1PERIPH_BASEADDR + 0x3800)
//////
////////base address of APB1 bus
//////#define I2C1_BASEADDR 	(APB1PERIPH_BASEADDR + 0x5400)
//////#define I2C2_BASEADDR	(APB1PERIPH_BASEADDR + 0x5800)
//////#define I2C3_BASEADDR	(APB1PERIPH_BASEADDR + 0x5C00)
//////
//////#define SPI2_BASEADDR	(APB1PERIPH_BASEADDR + 0x3800)
//////#define SPI3_BASEADDR	(APB1PERIPH_BASEADDR + 0x3C00)
//////
//////
//////#define USART2_BASEADDR	(APB1PERIPH_BASEADDR + 0x4400)
//////#define USART3_BASEADDR	(APB1PERIPH_BASEADDR + 0x4800)
//////#define UART4_BASEADDR	(APB1PERIPH_BASEADDR + 0x4C00)
//////#define UART5_BASEADDR	(APB1PERIPH_BASEADDR + 0x5000)
//////
////////base address of APB2 bus
//////
//////#define EXTI_BASEADDR	(APB2PERIPH_BASEADDR + 0x3C00)
//////#define SPI1_BASEADDR	(APB2PERIPH_BASEADDR + 0x3000)
//////#define SPI4_BASEADDR	(APB2PERIPH_BASEADDR + 0x3400)
//////#define SYSCFG_BASEADDR	(APB2PERIPH_BASEADDR + 0x3800)
//////#define USART1_BASEADDR	(APB2PERIPH_BASEADDR + 0x1000)
//////#define USART6_BASEADDR	(APB2PERIPH_BASEADDR + 0x1400)
//////
//////
////////peripheral register definition structure
//////typedef struct
//////{
//////	volatile uint32_t MODER;
//////	volatile uint32_t OTYPER;
//////	volatile uint32_t OSPEEDR;
//////	volatile uint32_t PUPDR;
//////	volatile uint32_t IDR;
//////	volatile uint32_t ODR;
//////	volatile uint32_t BSRR;
//////	volatile uint32_t LCKR;
//////	volatile uint32_t ARF[2]; //ARF[0] low register ARF[1] high register
//////}GPIO_RegDef_t;
//////
//////
//////typedef struct
//////{
//////	volatile uint32_t CR;
//////	volatile uint32_t PLLCFGR;
//////	volatile uint32_t CFGR;
//////	volatile uint32_t CIR;
//////	volatile uint32_t AHB1RSTR;
//////	volatile uint32_t AHB2RSTR;
//////	volatile uint32_t AHB3RSTR;
//////	uint32_t reserved0;
//////	volatile uint32_t APB1RSTR;
//////	volatile uint32_t CRAPB2RSTR;
//////	uint32_t reserved1[2];
//////	volatile uint32_t AHB1ENR;
//////	volatile uint32_t AHB2ENR;
//////	volatile uint32_t AHB3ENR;
//////	uint32_t reserved2;
//////	volatile uint32_t APB1ENR;
//////	volatile uint32_t APB2ENR;
//////	uint32_t reserved3[2];
//////	volatile uint32_t AHB1LPENR;
//////	volatile uint32_t AHB2LPENR;
//////	volatile uint32_t AHB3LPENR;
//////	uint32_t reserved4;
//////	volatile uint32_t APB1LPENR;
//////	volatile uint32_t APB2LPENR;
//////	uint32_t reserved5[2];
//////	volatile uint32_t BDCR;
//////	volatile uint32_t CSR;
//////	uint32_t reserved6[2];
//////	volatile uint32_t PLLI2SCFGR;
//////	volatile uint32_t PLLSAICFGR;
//////	volatile uint32_t DCKCFGR;
//////}RCC_RegDef_t;
//////
//////
//////
////////peripheral register definition structure for EXTI
//////typedef struct
//////{
//////	volatile uint32_t IMR;
//////	volatile uint32_t EMR;
//////	volatile uint32_t RTSR;
//////	volatile uint32_t FTSR;
//////	volatile uint32_t SWIER;
//////	volatile uint32_t PR;
//////}EXTI_RegDef_t;
//////
////////peripheral register definition structure for SYSCFG
//////typedef struct
//////{
//////	volatile uint32_t MEMRMP;
//////	volatile uint32_t PMC;
//////	volatile uint32_t EXTICR[4];
//////	uint32_t reserved1[2];
//////	volatile uint32_t CMPCR;
//////	uint32_t reserved2[2];
//////	volatile uint32_t CFGR;
//////}SYSCFG_RegDef_t;
//////
//////
//////#define GPIOA    ((GPIO_RegDef_t*)GPIOA_BASEADDR)
//////#define GPIOB    ((GPIO_RegDef_t*)GPIOB_BASEADDR)
//////#define GPIOC    ((GPIO_RegDef_t*)GPIOC_BASEADDR)
//////#define GPIOD    ((GPIO_RegDef_t*)GPIOD_BASEADDR)
//////#define GPIOE    ((GPIO_RegDef_t*)GPIOE_BASEADDR)
//////#define GPIOF    ((GPIO_RegDef_t*)GPIOF_BASEADDR)
//////#define GPIOG    ((GPIO_RegDef_t*)GPIOG_BASEADDR)
//////#define GPIOH    ((GPIO_RegDef_t*)GPIOH_BASEADDR)
//////#define GPIOI    ((GPIO_RegDef_t*)GPIOI_BASEADDR)
//////#define RCC    	 ((RCC_RegDef_t*)RCC_BASEADDR)
//////#define EXTI     ((EXTI_RegDef_t*)EXTI_BASEADDR)
//////#define SYSCFG   ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
//////
//////
////////GPIO I2C SPI USART SYSCFG
//////#define GPIOA_PCLK_EN()   (RCC->AHB1ENR |= (1<<0))
//////#define GPIOB_PCLK_EN()   (RCC->AHB1ENR |= (1<<1))
//////#define GPIOC_PCLK_EN()   (RCC->AHB1ENR |= (1<<2))
//////#define GPIOD_PCLK_EN()   (RCC->AHB1ENR |= (1<<3))
//////#define GPIOE_PCLK_EN()   (RCC->AHB1ENR |= (1<<4))
//////#define GPIOF_PCLK_EN()   (RCC->AHB1ENR |= (1<<5))
//////#define GPIOG_PCLK_EN()   (RCC->AHB1ENR |= (1<<6))
//////#define GPIOH_PCLK_EN()   (RCC->AHB1ENR |= (1<<7))
//////#define GPIOI_PCLK_EN()   (RCC->AHB1ENR |= (1<<8))
//////
//////#define GPIOA_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<0))
//////#define GPIOB_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<1))
//////#define GPIOC_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<2))
//////#define GPIOD_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<3))
//////#define GPIOE_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<4))
//////#define GPIOF_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<5))
//////#define GPIOG_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<6))
//////#define GPIOH_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<7))
//////#define GPIOI_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<8))
//////
//////#define I2C1_PCLK_EN()    (RCC->APB1ENR |= (1<<21))
//////#define I2C2_PCLK_EN()    (RCC->APB1ENR |= (1<<22))
//////#define I2C3_PCLK_EN()    (RCC->APB1ENR |= (1<<23))
//////
//////#define I2C1_PCLK_DI()    (RCC->APB1ENR &= ~(1<<21))
//////#define I2C2_PCLK_DI()    (RCC->APB1ENR &= ~(1<<22))
//////#define I2C3_PCLK_DI()    (RCC->APB1ENR &= ~(1<<23))
//////
//////
//////#define USART1_PCLK_EN()  (RCC->APB2ENR |= (1<<4))
//////#define USART2_PCLK_EN()  (RCC->APB1ENR |= (1<<17))
//////#define USART3_PCLK_EN()  (RCC->APB1ENR |= (1<<18))
//////#define UART4_PCLK_EN()   (RCC->APB1ENR |= (1<<19))
//////#define UART5_PCLK_EN()   (RCC->APB1ENR |= (1<<20))
//////#define UART6_PCLK_EN()   (RCC->APB2ENR |= (1<<5))
//////
//////#define USART1_PCLK_DI()  (RCC->APB2ENR &= ~(1<<4))
//////#define USART2_PCLK_DI()  (RCC->APB1ENR &= ~(1<<17))
//////#define USART3_PCLK_DI()  (RCC->APB1ENR &= ~(1<<18))
//////#define UART4_PCLK_DI()   (RCC->APB1ENR &= ~(1<<19))
//////#define UART5_PCLK_DI()   (RCC->APB1ENR &= ~(1<<20))
//////#define UART6_PCLK_DI()   (RCC->APB2ENR &= ~(1<<5))
//////
//////#define SPI1_PCLK_EN() 	  (RCC->APB2ENR |= (1<<12))
//////#define SPI2_PCLK_EN()    (RCC->APB1ENR |= (1<<14))
//////#define SPI3_PCLK_EN()    (RCC->APB1ENR |= (1<<15))
//////#define SPI4_PCLK_EN() 	  (RCC->APB2ENR |= (1<<13))
//////
//////#define SPI1_PCLK_DI()    (RCC->APB2ENR &= ~(1<<12))
//////#define SPI2_PCLK_DI()    (RCC->APB1ENR &= ~(1<<14))
//////#define SPI3_PCLK_DI()    (RCC->APB1ENR &= ~(1<<15))
//////#define SPI4_PCLK_DI()    (RCC->APB2ENR &= ~(1<<13))
//////
//////#define SYSCFG_PCLK_EN()  (RCC->APB2ENR |= (1<<14))
//////#define SYSCFG_PCLK_DI()  (RCC->APB2ENR &= ~(1<<14))
//////
//////
///////*
////// *  Macros to reset GPIOx peripherals
////// */
//////
//////#define GPIOA_REG_RESTART()  	do {(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0)); }while(0)
//////#define GPIOB_REG_RESTART()  	do {(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1)); }while(0)
//////#define GPIOC_REG_RESTART()  	do {(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2)); }while(0)
//////#define GPIOD_REG_RESTART()  	do {(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3)); }while(0)
//////#define GPIOE_REG_RESTART()  	do {(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4)); }while(0)
//////#define GPIOF_REG_RESTART()  	do {(RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5)); }while(0)
//////#define GPIOG_REG_RESTART()  	do {(RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6)); }while(0)
//////#define GPIOH_REG_RESTART()  	do {(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7)); }while(0)
//////#define GPIOI_REG_RESTART()  	do {(RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8)); }while(0)
//////
///////*
////// *  returns port code for given GPIOx base address
////// */
///////*
////// * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
////// */
////
///////*
////// * IRQ(Interrupt Request) Numbers of STM32F407x MCU
////// * NOTE: update these macros with valid values according to your MCU
////// * TODO: You may complete this list for other peripherals
////// */
//////
//////#define IRQ_NO_EXTI0 		6
//////#define IRQ_NO_EXTI1 		7
//////#define IRQ_NO_EXTI2 		8
//////#define IRQ_NO_EXTI3 		9
//////#define IRQ_NO_EXTI4 		10
//////#define IRQ_NO_EXTI5_9 		23
//////#define IRQ_NO_EXTI10_15 	40
//////
//////#define NVIC_IRQ_PRI0  0
//////#define NVIC_IRQ_PRI1  1
//////#define NVIC_IRQ_PRI2  2
//////#define NVIC_IRQ_PRI3  3
//////#define NVIC_IRQ_PRI4  4
//////#define NVIC_IRQ_PRI5  5
//////#define NVIC_IRQ_PRI6  6
//////#define NVIC_IRQ_PRI7  7
//////#define NVIC_IRQ_PRI8  8
//////#define NVIC_IRQ_PRI9  9
//////#define NVIC_IRQ_PRI10 10
//////#define NVIC_IRQ_PRI11 11
//////#define NVIC_IRQ_PRI12 12
//////#define NVIC_IRQ_PRI13 13
//////#define NVIC_IRQ_PRI14 14
//////#define NVIC_IRQ_PRI15 15
//////
//////#define ENABLE    			1
//////#define DISABLE   			0
//////#define SET		  			ENABLE
//////#define RESET	  			DISABLE
//////#define GPIO_PIN_SET		SET
//////#define GPIO_PIN_RESET	  	RESET
//////#include "stm32f407xx_gpio_driver.h"
//////
//////#endif /* INC_STM32F407XX_H_ */
/////*
//// * stm3f407xx.h
//// *
//// *  Created on: Jan 29, 2019
//// *      Author: admin
//// */
////
////
////
////#ifndef INC_STM3F407XX_H_
////#define INC_STM3F407XX_H_
////
////#include<stddef.h>
////#include<stdint.h>
////
////#define __vo volatile
////#define __weak __attribute__((weak))
////
////
////
/////**********************************START:Processor Specific Details **********************************/
/////*
//// * ARM Cortex Mx Processor NVIC ISERx register Addresses
//// */
////
////#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
////#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
////#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
////#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )
////
////
/////*
//// * ARM Cortex Mx Processor NVIC ICERx register Addresses
//// */
////#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
////#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
////#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
////#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)
////
////
/////*
//// * ARM Cortex Mx Processor Priority Register Address Calculation
//// */
////#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)
////
/////*
//// * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
//// */
////#define NO_PR_BITS_IMPLEMENTED  4
////
/////*
//// * base addresses of Flash and SRAM memories
//// */
////
////#define FLASH_BASEADDR						0x08000000U   		/*!<explain this macro briefly here  */
////#define SRAM1_BASEADDR						0x20000000U  		/*!<explain this macro briefly here  */
////#define SRAM2_BASEADDR						0x2001C000U 		/*!<explain this macro briefly here  */
////#define ROM_BASEADDR						0x1FFF0000U
////#define SRAM 								SRAM1_BASEADDR
////
////
/////*
//// * AHBx and APBx Bus Peripheral base addresses
//// */
////
////#define PERIPH_BASEADDR 						0x40000000U
////#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
////#define APB2PERIPH_BASEADDR						0x40010000U
////#define AHB1PERIPH_BASEADDR						0x40020000U
////#define AHB2PERIPH_BASEADDR						0x50000000U
////
/////*
//// * Base addresses of peripherals which are hanging on AHB1 bus
//// * TODO : Complete for all other peripherals
//// */
////
////#define GPIOA_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0000)
////#define GPIOB_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0400)
////#define GPIOC_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0800)
////#define GPIOD_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0C00)
////#define GPIOE_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1000)
////#define GPIOF_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1400)
////#define GPIOG_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1800)
////#define GPIOH_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1C00)
////#define GPIOI_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x2000)
////#define RCC_BASEADDR                     (AHB1PERIPH_BASEADDR + 0x3800)
/////*
//// * Base addresses of peripherals which are hanging on APB1 bus
//// * TODO : Complete for all other peripherals
//// */
////#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
////#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
////#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)
////
////#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
////#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)
////
////#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
////#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
////#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
////#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)
////
/////*
//// * Base addresses of peripherals which are hanging on APB2 bus
//// * TODO : Complete for all other peripherals
//// */
////#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)
////#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
////#define SPI4_BASEADDR						(APB2PERIPH_BASEADDR + 0x3400)
////#define SYSCFG_BASEADDR        				(APB2PERIPH_BASEADDR + 0x3800)
////#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
////#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)
//////ref manual 2.4 memory map
////
////
////
////
/////**********************************peripheral register definition structures **********************************/
////
/////*
//// * Note : Registers of a peripheral are specific to MCU
//// * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
//// * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
//// * Please check your Device RM
//// */
////
////typedef struct
////{
////	__vo uint32_t MODER;                        /*!< GPIO port mode register,                    	Address offset: 0x00      */
////	__vo uint32_t OTYPER;                       /*!< TODO,     										Address offset: 0x04      */
////	__vo uint32_t OSPEEDR;
////	__vo uint32_t PUPDR;
////	__vo uint32_t IDR;
////	__vo uint32_t ODR;
////	__vo uint32_t BSRR;
////	__vo uint32_t LCKR;
////	__vo uint32_t AFR[2];					 /*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
////}GPIO_RegDef_t;
////
////
////
/////*
//// * peripheral register definition structure for RCC
//// */
////typedef struct
////{
////  __vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
////  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
////  __vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
////  __vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
////  __vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
////  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
////  __vo uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
////  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
////  __vo uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
////  __vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
////  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
////  __vo uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
////  __vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
////  __vo uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
////  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
////  __vo uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
////  __vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
////  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
////  __vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
////  __vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
////  __vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
////  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
////  __vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
////  __vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
////  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
////  __vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
////  __vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
////  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
////  __vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
////  __vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
////  __vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
////  __vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
////  __vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
////  __vo uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */
////
////} RCC_RegDef_t;
////
////
////
/////*
//// * peripheral register definition structure for EXTI
//// */
////typedef struct
////{
////	__vo uint32_t IMR;    /*!< Give a short description,          	  	    Address offset: 0x00 */
////	__vo uint32_t EMR;    /*!< TODO,                						Address offset: 0x04 */
////	__vo uint32_t RTSR;   /*!< TODO,  									     Address offset: 0x08 */
////	__vo uint32_t FTSR;   /*!< TODO, 										Address offset: 0x0C */
////	__vo uint32_t SWIER;  /*!< TODO,  									   Address offset: 0x10 */
////	__vo uint32_t PR;     /*!< TODO,                   					   Address offset: 0x14 */
////
////}EXTI_RegDef_t;
////
////
/////*
//// * peripheral register definition structure for SPI
//// */
////typedef struct
////{
////	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
////	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
////	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x08 */
////	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x0C */
////	__vo uint32_t CRCPR;      /*!< TODO,     										Address offset: 0x10 */
////	__vo uint32_t RXCRCR;     /*!< TODO,     										Address offset: 0x14 */
////	__vo uint32_t TXCRCR;     /*!< TODO,     										Address offset: 0x18 */
////	__vo uint32_t I2SCFGR;    /*!< TODO,     										Address offset: 0x1C */
////	__vo uint32_t I2SPR;      /*!< TODO,     										Address offset: 0x20 */
////} SPI_RegDef_t;//REF manual 28.5.10 register map
////
////
/////*
//// * peripheral register definition structure for SYSCFG
//// */
////typedef struct
////{
////	__vo uint32_t MEMRMP;       /*!< Give a short description,                    Address offset: 0x00      */
////	__vo uint32_t PMC;          /*!< TODO,     									  Address offset: 0x04      */
////	__vo uint32_t EXTICR[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
////	uint32_t      RESERVED1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
////	__vo uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */
////	uint32_t      RESERVED2[2];  /*!<                                             Reserved, 0x24-0x28 	    */
////	__vo uint32_t CFGR;         /*!< TODO                                         Address offset: 0x2C   	*/
////} SYSCFG_RegDef_t;
////
////
/////*
//// * peripheral register definition structure for I2C
//// */
////
////
/////*
//// * peripheral register definition structure for USART
//// */
////
////
/////*
//// * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
//// */
////
////#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
////#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
////#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
////#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
////#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
////#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
////#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
////#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)
////#define GPIOI  				((GPIO_RegDef_t*)GPIOI_BASEADDR)
////
////#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
////#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
////#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
////
////
////#define SPI1  				((SPI_RegDef_t*)SPI1_BASEADDR)
////#define SPI2  				((SPI_RegDef_t*)SPI2_BASEADDR)
////#define SPI3  				((SPI_RegDef_t*)SPI3_BASEADDR)
////
////#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR)
////#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR)
////#define I2C3  				((I2C_RegDef_t*)I2C3_BASEADDR)
////
////#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
////#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
////#define USART3  			((USART_RegDef_t*)USART3_BASEADDR)
////#define UART4  				((USART_RegDef_t*)UART4_BASEADDR)
////#define UART5  				((USART_RegDef_t*)UART5_BASEADDR)
////#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)
////
/////*
//// * Clock Enable Macros for GPIOx peripherals
//// */
////
////#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))
////#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
////#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
////#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
////#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
////#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
////#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
////#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
////#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))
////
////
/////*
//// * Clock Enable Macros for I2Cx peripherals
//// */
////#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
////#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
////#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))
////
////
/////*
//// * Clock Enable Macros for SPIx peripheralsbu
//// */
////#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
////#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
////#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
////#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))
////
////
//////
////
////#define SPI4_PCLK_DI()    (RCC->APB2ENR &= ~(1<<13))
////
////
/////*
//// * Clock Enable Macros for USARTx peripherals
//// */
////#define USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 4))
////#define USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17))
////#define USART3_PCCK_EN() (RCC->APB1ENR |= (1 << 18))
////#define UART4_PCCK_EN()  (RCC->APB1ENR |= (1 << 19))
////#define UART5_PCCK_EN()  (RCC->APB1ENR |= (1 << 20))
////#define USART6_PCCK_EN() (RCC->APB1ENR |= (1 << 5))
////
/////*
//// * Clock Enable Macros for SYSCFG peripheral
//// */
////#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))
////
////
/////*
//// * Clock Disable Macros for GPIOx peripherals
//// */
////#define GPIOA_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<0))
////#define GPIOB_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<1))
////#define GPIOC_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<2))
////#define GPIOD_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<3))
////#define GPIOE_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<4))
////#define GPIOF_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<5))
////#define GPIOG_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<6))
////#define GPIOH_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<7))
////#define GPIOI_PCLK_DI()   (RCC->AHB1ENR &= ~(1<<8))
/////*
//// * Clock Disable Macros for SPIx peripherals
//// */
////#define SPI1_PCLK_DI()    (RCC->APB2ENR &= ~(1<<12))
////#define SPI2_PCLK_DI()    (RCC->APB1ENR &= ~(1<<14))
////#define SPI3_PCLK_DI()    (RCC->APB1ENR &= ~(1<<15))
/////*
//// * Clock Disable Macros for USARTx peripherals
//// */
////
////
/////*
//// * Clock Disable Macros for SYSCFG peripheral
//// */
////
////
/////*
//// *  Macros to reset GPIOx peripherals
//// */
////#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
////#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
////#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
////#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
////#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
////#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
////#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
////#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
////#define GPIOI_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)
////
/////*
//// *  Macros to reset SPIx peripherals
//// */
////
////#define SPI1_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
////#define SPI2_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
////#define SPI3_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
/////*
//// *  returns port code for given GPIOx base address
//// */
/////*
//// * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
//// */

////
////
/////*
//// * IRQ(Interrupt Request) Numbers of STM32F407x MCU
//// * NOTE: update these macros with valid values according to your MCU
//// * TODO: You may complete this list for other peripherals
//// */
////
////#define IRQ_NO_EXTI0 		6
////#define IRQ_NO_EXTI1 		7
////#define IRQ_NO_EXTI2 		8
////#define IRQ_NO_EXTI3 		9
////#define IRQ_NO_EXTI4 		10
////#define IRQ_NO_EXTI9_5 		23
////#define IRQ_NO_EXTI15_10 	40
////
////#define IRQ_NO_SPI1			35
////#define IRQ_NO_SPI2         36
////#define IRQ_NO_SPI3         51
////#define IRQ_NO_SPI4
////
////#define IRQ_NO_I2C1_EV     31
////#define IRQ_NO_I2C1_ER     32
////
////#define IRQ_NO_USART1	    37
////#define IRQ_NO_USART2	    38
////#define IRQ_NO_USART3	    39
////#define IRQ_NO_UART4	    52
////#define IRQ_NO_UART5	    53
////#define IRQ_NO_USART6	    71
////
////
/////*
//// * macros for all the possible priority levels
//// */
////#define NVIC_IRQ_PRI0    0
////#define NVIC_IRQ_PRI15    15
////
////
//////some generic macros
////
////#define ENABLE 				1
////#define DISABLE 			0
////#define SET 				ENABLE
////#define RESET 				DISABLE
////#define GPIO_PIN_SET        SET
////#define GPIO_PIN_RESET      RESET
////#define FLAG_RESET         RESET
////#define FLAG_SET 			SET
////
////
/////******************************************************************************************
//// *Bit position definitions of SPI peripheral
//// ******************************************************************************************/
/////*
//// * Bit position definitions SPI_CR1 control register 1
//// */
////#define SPI_CR1_CPHA     				 0
////#define SPI_CR1_CPOL      				 1
////#define SPI_CR1_MSTR     				 2
////#define SPI_CR1_BR   					 3
////#define SPI_CR1_SPE     				 6
////#define SPI_CR1_LSBFIRST   			 	 7
////#define SPI_CR1_SSI     				 8
////#define SPI_CR1_SSM      				 9
////#define SPI_CR1_RXONLY      		 	10
////#define SPI_CR1_DFF     			 	11
////#define SPI_CR1_CRCNEXT   			 	12
////#define SPI_CR1_CRCEN   			 	13
////#define SPI_CR1_BIDIOE     			 	14
////#define SPI_CR1_BIDIMODE      			15
////
/////*
//// * Bit position definitions SPI_CR2 control register 2
//// */
////#define SPI_CR2_RXDMAEN		 			0
////#define SPI_CR2_TXDMAEN				 	1
////#define SPI_CR2_SSOE				 	2
////#define SPI_CR2_FRF						4
////#define SPI_CR2_ERRIE					5
////#define SPI_CR2_RXNEIE				 	6
////#define SPI_CR2_TXEIE					7
////
////
/////*
//// * Bit position definitions SPI_SR (status register)
//// */
////#define SPI_SR_RXNE						0
////#define SPI_SR_TXE				 		1 //tx buffer empty
////#define SPI_SR_CHSIDE				 	2
////#define SPI_SR_UDR					 	3
////#define SPI_SR_CRCERR				 	4
////#define SPI_SR_MODF					 	5
////#define SPI_SR_OVR					 	6
////#define SPI_SR_BSY					 	7
////#define SPI_SR_FRE					 	8
////
/////******************************************************************************************
//// *Bit position definitions of I2C peripheral
//// ******************************************************************************************/
/////*
//// * Bit position definitions I2C_CR1
//// */
////
////
/////*
//// * Bit position definitions I2C_CR2
//// */
////
////
/////*
//// * Bit position definitions I2C_OAR1
//// */
////
////
/////*
//// * Bit position definitions I2C_SR1
//// */
////
////
////
/////*
//// * Bit position definitions I2C_SR2
//// */
////
////
/////*
//// * Bit position definitions I2C_CCR
//// */
////
////
/////******************************************************************************************
//// *Bit position definitions of USART peripheral
//// ******************************************************************************************/
////
/////*
//// * Bit position definitions USART_CR1
//// */
////
////
////
////
/////*
//// * Bit position definitions USART_CR2
//// */
////
////
////
/////*
//// * Bit position definitions USART_CR3
//// */
////
////
/////*
//// * Bit position definitions USART_SR
//// */
////
////
////
////#include "stm32f407xx_gpio_driver.h"
////#include "stm32f407xx_spi_driver.h"
////
////#endif /* INC_STM3F407XX_H_ */
