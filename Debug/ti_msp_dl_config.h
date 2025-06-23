/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     80000000



/* Defines for PWM_MOTOR */
#define PWM_MOTOR_INST                                                     TIMA0
#define PWM_MOTOR_INST_IRQHandler                               TIMA0_IRQHandler
#define PWM_MOTOR_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define PWM_MOTOR_INST_CLK_FREQ                                         40000000
/* GPIO defines for channel 2 */
#define GPIO_PWM_MOTOR_C2_PORT                                             GPIOB
#define GPIO_PWM_MOTOR_C2_PIN                                      DL_GPIO_PIN_0
#define GPIO_PWM_MOTOR_C2_IOMUX                                  (IOMUX_PINCM12)
#define GPIO_PWM_MOTOR_C2_IOMUX_FUNC                 IOMUX_PINCM12_PF_TIMA0_CCP2
#define GPIO_PWM_MOTOR_C2_IDX                                DL_TIMER_CC_2_INDEX
/* GPIO defines for channel 3 */
#define GPIO_PWM_MOTOR_C3_PORT                                             GPIOB
#define GPIO_PWM_MOTOR_C3_PIN                                     DL_GPIO_PIN_13
#define GPIO_PWM_MOTOR_C3_IOMUX                                  (IOMUX_PINCM30)
#define GPIO_PWM_MOTOR_C3_IOMUX_FUNC                 IOMUX_PINCM30_PF_TIMA0_CCP3
#define GPIO_PWM_MOTOR_C3_IDX                                DL_TIMER_CC_3_INDEX



/* Defines for TIMER_PID */
#define TIMER_PID_INST                                                   (TIMA1)
#define TIMER_PID_INST_IRQHandler                               TIMA1_IRQHandler
#define TIMER_PID_INST_INT_IRQN                                 (TIMA1_INT_IRQn)
#define TIMER_PID_INST_LOAD_VALUE                                          (49U)
/* Defines for TIMER_INTOWHILE */
#define TIMER_INTOWHILE_INST                                             (TIMG0)
#define TIMER_INTOWHILE_INST_IRQHandler                         TIMG0_IRQHandler
#define TIMER_INTOWHILE_INST_INT_IRQN                           (TIMG0_INT_IRQn)
#define TIMER_INTOWHILE_INST_LOAD_VALUE                                  (1997U)
/* Defines for TIMER_GENERAL */
#define TIMER_GENERAL_INST                                               (TIMG6)
#define TIMER_GENERAL_INST_IRQHandler                           TIMG6_IRQHandler
#define TIMER_GENERAL_INST_INT_IRQN                             (TIMG6_INT_IRQn)
#define TIMER_GENERAL_INST_LOAD_VALUE                                    (1997U)




/* Defines for I2C_MPU6050 */
#define I2C_MPU6050_INST                                                    I2C0
#define I2C_MPU6050_INST_IRQHandler                              I2C0_IRQHandler
#define I2C_MPU6050_INST_INT_IRQN                                  I2C0_INT_IRQn
#define I2C_MPU6050_BUS_SPEED_HZ                                          400000
#define GPIO_I2C_MPU6050_SDA_PORT                                          GPIOA
#define GPIO_I2C_MPU6050_SDA_PIN                                  DL_GPIO_PIN_28
#define GPIO_I2C_MPU6050_IOMUX_SDA                                (IOMUX_PINCM3)
#define GPIO_I2C_MPU6050_IOMUX_SDA_FUNC                 IOMUX_PINCM3_PF_I2C0_SDA
#define GPIO_I2C_MPU6050_SCL_PORT                                          GPIOA
#define GPIO_I2C_MPU6050_SCL_PIN                                  DL_GPIO_PIN_31
#define GPIO_I2C_MPU6050_IOMUX_SCL                                (IOMUX_PINCM6)
#define GPIO_I2C_MPU6050_IOMUX_SCL_FUNC                 IOMUX_PINCM6_PF_I2C0_SCL

/* Defines for I2C_OLED */
#define I2C_OLED_INST                                                       I2C1
#define I2C_OLED_INST_IRQHandler                                 I2C1_IRQHandler
#define I2C_OLED_INST_INT_IRQN                                     I2C1_INT_IRQn
#define I2C_OLED_BUS_SPEED_HZ                                             400000
#define GPIO_I2C_OLED_SDA_PORT                                             GPIOB
#define GPIO_I2C_OLED_SDA_PIN                                      DL_GPIO_PIN_3
#define GPIO_I2C_OLED_IOMUX_SDA                                  (IOMUX_PINCM16)
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC                   IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_I2C_OLED_SCL_PORT                                             GPIOB
#define GPIO_I2C_OLED_SCL_PIN                                      DL_GPIO_PIN_2
#define GPIO_I2C_OLED_IOMUX_SCL                                  (IOMUX_PINCM15)
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC                   IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for UART_PC */
#define UART_PC_INST                                                       UART0
#define UART_PC_INST_FREQUENCY                                          40000000
#define UART_PC_INST_IRQHandler                                 UART0_IRQHandler
#define UART_PC_INST_INT_IRQN                                     UART0_INT_IRQn
#define GPIO_UART_PC_RX_PORT                                               GPIOA
#define GPIO_UART_PC_TX_PORT                                               GPIOA
#define GPIO_UART_PC_RX_PIN                                       DL_GPIO_PIN_11
#define GPIO_UART_PC_TX_PIN                                       DL_GPIO_PIN_10
#define GPIO_UART_PC_IOMUX_RX                                    (IOMUX_PINCM22)
#define GPIO_UART_PC_IOMUX_TX                                    (IOMUX_PINCM21)
#define GPIO_UART_PC_IOMUX_RX_FUNC                     IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_PC_IOMUX_TX_FUNC                     IOMUX_PINCM21_PF_UART0_TX
#define UART_PC_BAUD_RATE                                               (115200)
#define UART_PC_IBRD_40_MHZ_115200_BAUD                                     (21)
#define UART_PC_FBRD_40_MHZ_115200_BAUD                                     (45)





/* Port definition for Pin Group GPIO_SWICH */
#define GPIO_SWICH_PORT                                                  (GPIOB)

/* Defines for PIN_S2: GPIOB.21 with pinCMx 49 on package pin 20 */
// groups represented: ["GPIO_MPU6050","GPIO_ENCODER_LEFT","GPIO_SWICH"]
// pins affected: ["PIN_INT","PIN_EA_LEFT","PIN_S2"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_SWICH_PIN_S2_IIDX                              (DL_GPIO_IIDX_DIO21)
#define GPIO_SWICH_PIN_S2_PIN                                   (DL_GPIO_PIN_21)
#define GPIO_SWICH_PIN_S2_IOMUX                                  (IOMUX_PINCM49)
/* Port definition for Pin Group GPIO_MPU6050 */
#define GPIO_MPU6050_PORT                                                (GPIOB)

/* Defines for PIN_INT: GPIOB.15 with pinCMx 32 on package pin 3 */
#define GPIO_MPU6050_PIN_INT_IIDX                           (DL_GPIO_IIDX_DIO15)
#define GPIO_MPU6050_PIN_INT_PIN                                (DL_GPIO_PIN_15)
#define GPIO_MPU6050_PIN_INT_IOMUX                               (IOMUX_PINCM32)
/* Defines for PIN_LEFT_DIRECT_1: GPIOA.13 with pinCMx 35 on package pin 6 */
#define GPIO_MOTOR_PIN_LEFT_DIRECT_1_PORT                                (GPIOA)
#define GPIO_MOTOR_PIN_LEFT_DIRECT_1_PIN                        (DL_GPIO_PIN_13)
#define GPIO_MOTOR_PIN_LEFT_DIRECT_1_IOMUX                       (IOMUX_PINCM35)
/* Defines for PIN_LEFT_DIRECT_2: GPIOA.12 with pinCMx 34 on package pin 5 */
#define GPIO_MOTOR_PIN_LEFT_DIRECT_2_PORT                                (GPIOA)
#define GPIO_MOTOR_PIN_LEFT_DIRECT_2_PIN                        (DL_GPIO_PIN_12)
#define GPIO_MOTOR_PIN_LEFT_DIRECT_2_IOMUX                       (IOMUX_PINCM34)
/* Defines for PIN_LEFT_STSNDBY: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_MOTOR_PIN_LEFT_STSNDBY_PORT                                 (GPIOB)
#define GPIO_MOTOR_PIN_LEFT_STSNDBY_PIN                         (DL_GPIO_PIN_16)
#define GPIO_MOTOR_PIN_LEFT_STSNDBY_IOMUX                        (IOMUX_PINCM33)
/* Defines for PIN_RIGHT_DIRECT_1: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_MOTOR_PIN_RIGHT_DIRECT_1_PORT                               (GPIOB)
#define GPIO_MOTOR_PIN_RIGHT_DIRECT_1_PIN                        (DL_GPIO_PIN_6)
#define GPIO_MOTOR_PIN_RIGHT_DIRECT_1_IOMUX                      (IOMUX_PINCM23)
/* Defines for PIN_RIGHT_DIRECT_2: GPIOB.7 with pinCMx 24 on package pin 59 */
#define GPIO_MOTOR_PIN_RIGHT_DIRECT_2_PORT                               (GPIOB)
#define GPIO_MOTOR_PIN_RIGHT_DIRECT_2_PIN                        (DL_GPIO_PIN_7)
#define GPIO_MOTOR_PIN_RIGHT_DIRECT_2_IOMUX                      (IOMUX_PINCM24)
/* Defines for PIN_RIGHT_STSNDBY: GPIOB.8 with pinCMx 25 on package pin 60 */
#define GPIO_MOTOR_PIN_RIGHT_STSNDBY_PORT                                (GPIOB)
#define GPIO_MOTOR_PIN_RIGHT_STSNDBY_PIN                         (DL_GPIO_PIN_8)
#define GPIO_MOTOR_PIN_RIGHT_STSNDBY_IOMUX                       (IOMUX_PINCM25)
/* Defines for PIN_EA_LEFT: GPIOB.20 with pinCMx 48 on package pin 19 */
#define GPIO_ENCODER_LEFT_PIN_EA_LEFT_PORT                               (GPIOB)
#define GPIO_ENCODER_LEFT_PIN_EA_LEFT_IIDX                  (DL_GPIO_IIDX_DIO20)
#define GPIO_ENCODER_LEFT_PIN_EA_LEFT_PIN                       (DL_GPIO_PIN_20)
#define GPIO_ENCODER_LEFT_PIN_EA_LEFT_IOMUX                      (IOMUX_PINCM48)
/* Defines for PIN_EB_LEFT: GPIOA.15 with pinCMx 37 on package pin 8 */
#define GPIO_ENCODER_LEFT_PIN_EB_LEFT_PORT                               (GPIOA)
#define GPIO_ENCODER_LEFT_PIN_EB_LEFT_PIN                       (DL_GPIO_PIN_15)
#define GPIO_ENCODER_LEFT_PIN_EB_LEFT_IOMUX                      (IOMUX_PINCM37)
/* Defines for PIN_EA_RIGHT: GPIOA.27 with pinCMx 60 on package pin 31 */
#define GPIO_ENCODER_RIGHT_PIN_EA_RIGHT_PORT                             (GPIOA)
// pins affected by this interrupt request:["PIN_EA_RIGHT"]
#define GPIO_ENCODER_RIGHT_INT_IRQN                             (GPIOA_INT_IRQn)
#define GPIO_ENCODER_RIGHT_INT_IIDX             (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define GPIO_ENCODER_RIGHT_PIN_EA_RIGHT_IIDX                (DL_GPIO_IIDX_DIO27)
#define GPIO_ENCODER_RIGHT_PIN_EA_RIGHT_PIN                     (DL_GPIO_PIN_27)
#define GPIO_ENCODER_RIGHT_PIN_EA_RIGHT_IOMUX                    (IOMUX_PINCM60)
/* Defines for PIN_EB_RIGHT: GPIOB.1 with pinCMx 13 on package pin 48 */
#define GPIO_ENCODER_RIGHT_PIN_EB_RIGHT_PORT                             (GPIOB)
#define GPIO_ENCODER_RIGHT_PIN_EB_RIGHT_PIN                      (DL_GPIO_PIN_1)
#define GPIO_ENCODER_RIGHT_PIN_EB_RIGHT_IOMUX                    (IOMUX_PINCM13)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTOR_init(void);
void SYSCFG_DL_TIMER_PID_init(void);
void SYSCFG_DL_TIMER_INTOWHILE_init(void);
void SYSCFG_DL_TIMER_GENERAL_init(void);
void SYSCFG_DL_I2C_MPU6050_init(void);
void SYSCFG_DL_I2C_OLED_init(void);
void SYSCFG_DL_UART_PC_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
