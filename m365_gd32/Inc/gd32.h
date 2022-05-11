#ifndef __GD32_H
#define __GD32_H
#include "main.h"
void SystemClock_Config_gd32(void);

#define REG32(addr)                  	(*(volatile uint32_t *)(uint32_t)(addr))
#define AHB1_BUS_BASE         			((uint32_t)0x40018000U)        /*!< ahb1 base address                */
#define RCU_BASE              			(AHB1_BUS_BASE + 0x00009000U)  /*!< RCU base address                 */
#define RCU                             RCU_BASE

#define RCU_CTL                         REG32(RCU + 0x00U)        /*!< control register */
#define RCU_CFG0                        REG32(RCU + 0x04U)        /*!< clock configuration register 0 */
#define RCU_INT                         REG32(RCU + 0x08U)        /*!< clock interrupt register */
#define RCU_APB2RST                     REG32(RCU + 0x0CU)        /*!< APB2 reset register */
#define RCU_APB1RST                     REG32(RCU + 0x10U)        /*!< APB1 reset register */
#define RCU_AHBEN                       REG32(RCU + 0x14U)        /*!< AHB enable register */
#define RCU_APB2EN                      REG32(RCU + 0x18U)        /*!< APB2 enable register */
#define RCU_APB1EN                      REG32(RCU + 0x1CU)        /*!< APB1 enable register */
#define RCU_BDCTL                       REG32(RCU + 0x20U)        /*!< backup domain control register */
#define RCU_RSTSCK                      REG32(RCU + 0x24U)        /*!< reset source / clock register */
#define RCU_CFG1                        REG32(RCU + 0x2CU)        /*!< clock configuration register 1 */
#define RCU_DSV                         REG32(RCU + 0x34U)        /*!< deep-sleep mode voltage register */
#define RCU_ADDCTL                      REG32(RCU + 0xC0U)        /*!< Additional clock control register */
#define RCU_ADDINT                      REG32(RCU + 0xCCU)        /*!< Additional clock interrupt register */
#define RCU_ADDAPB1RST                  REG32(RCU + 0xE0U)        /*!< APB1 additional reset register */
#define RCU_ADDAPB1EN                   REG32(RCU + 0xE4U)        /*!< APB1 additional enable register */


#endif
