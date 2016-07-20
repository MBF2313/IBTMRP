/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CUR4_OUT_Pin GPIO_PIN_3
#define CUR4_OUT_GPIO_Port GPIOF
#define CUR3_OUT_Pin GPIO_PIN_4
#define CUR3_OUT_GPIO_Port GPIOF
#define CUR2_OUT_Pin GPIO_PIN_8
#define CUR2_OUT_GPIO_Port GPIOF
#define CUR1_OUT_Pin GPIO_PIN_9
#define CUR1_OUT_GPIO_Port GPIOF
#define PWR_DWN__Pin GPIO_PIN_1
#define PWR_DWN__GPIO_Port GPIOA
#define PWR_INH__Pin GPIO_PIN_2
#define PWR_INH__GPIO_Port GPIOA
#define POT_RST__1_Pin GPIO_PIN_3
#define POT_RST__1_GPIO_Port GPIOA
#define SPI1_SCLK_ADC_Pin GPIO_PIN_5
#define SPI1_SCLK_ADC_GPIO_Port GPIOA
#define SPI1_MISO_ADC_Pin GPIO_PIN_6
#define SPI1_MISO_ADC_GPIO_Port GPIOA
#define SPI1_MOSI_ADC_Pin GPIO_PIN_7
#define SPI1_MOSI_ADC_GPIO_Port GPIOA
#define TEST0_ADC_Pin GPIO_PIN_4
#define TEST0_ADC_GPIO_Port GPIOC
#define TEST1_ADC_Pin GPIO_PIN_5
#define TEST1_ADC_GPIO_Port GPIOC
#define CLKDIV_ADC_Pin GPIO_PIN_0
#define CLKDIV_ADC_GPIO_Port GPIOB
#define SYNC__ADC_Pin GPIO_PIN_1
#define SYNC__ADC_GPIO_Port GPIOB
#define POT_ADDR_1_Pin GPIO_PIN_14
#define POT_ADDR_1_GPIO_Port GPIOF
#define POT_ADDR_2_Pin GPIO_PIN_15
#define POT_ADDR_2_GPIO_Port GPIOF
#define CLK_ADC_Pin GPIO_PIN_8
#define CLK_ADC_GPIO_Port GPIOE
#define DRDY__ADC_Pin GPIO_PIN_9
#define DRDY__ADC_GPIO_Port GPIOE
#define PWDWN4__ADC_Pin GPIO_PIN_10
#define PWDWN4__ADC_GPIO_Port GPIOE
#define PWDWN3__ADC_Pin GPIO_PIN_11
#define PWDWN3__ADC_GPIO_Port GPIOE
#define PWDWN2__ADC_Pin GPIO_PIN_12
#define PWDWN2__ADC_GPIO_Port GPIOE
#define PWDWN1__ADC_Pin GPIO_PIN_13
#define PWDWN1__ADC_GPIO_Port GPIOE
#define POT_RST_2_Pin GPIO_PIN_14
#define POT_RST_2_GPIO_Port GPIOE
#define POT_ADDR_3_Pin GPIO_PIN_15
#define POT_ADDR_3_GPIO_Port GPIOE
#define I2C2_SCL_POT2_Pin GPIO_PIN_10
#define I2C2_SCL_POT2_GPIO_Port GPIOB
#define I2C2_SDA_POT2_Pin GPIO_PIN_11
#define I2C2_SDA_POT2_GPIO_Port GPIOB
#define SPI2_SCLK_THERM_Pin GPIO_PIN_13
#define SPI2_SCLK_THERM_GPIO_Port GPIOB
#define SPI2_MISO_THERM_Pin GPIO_PIN_14
#define SPI2_MISO_THERM_GPIO_Port GPIOB
#define SPI2_MOSI_THERM_Pin GPIO_PIN_15
#define SPI2_MOSI_THERM_GPIO_Port GPIOB
#define CS_6_THERM_Pin GPIO_PIN_8
#define CS_6_THERM_GPIO_Port GPIOD
#define CS_5_THERM_Pin GPIO_PIN_9
#define CS_5_THERM_GPIO_Port GPIOD
#define CS_2_THERM_Pin GPIO_PIN_10
#define CS_2_THERM_GPIO_Port GPIOD
#define CS_1_THERM_Pin GPIO_PIN_11
#define CS_1_THERM_GPIO_Port GPIOD
#define CS_4_THERM_Pin GPIO_PIN_12
#define CS_4_THERM_GPIO_Port GPIOD
#define CS_3_THERM_Pin GPIO_PIN_13
#define CS_3_THERM_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_4
#define LED4_GPIO_Port GPIOG
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOG
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOG
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOG
#define UART_TX_TEMP_Pin GPIO_PIN_6
#define UART_TX_TEMP_GPIO_Port GPIOC
#define UART_RX_TEMP_Pin GPIO_PIN_7
#define UART_RX_TEMP_GPIO_Port GPIOC
#define USB_OTG_OVER_CURRENT_Pin GPIO_PIN_9
#define USB_OTG_OVER_CURRENT_GPIO_Port GPIOC
#define USB_OTG_PWR_SWITCH_ON_Pin GPIO_PIN_8
#define USB_OTG_PWR_SWITCH_ON_GPIO_Port GPIOA
#define CUR2_A_Pin GPIO_PIN_11
#define CUR2_A_GPIO_Port GPIOC
#define CUR2_B_Pin GPIO_PIN_12
#define CUR2_B_GPIO_Port GPIOC
#define CUR4_EN_Pin GPIO_PIN_0
#define CUR4_EN_GPIO_Port GPIOD
#define DICKE4_IN2_Pin GPIO_PIN_1
#define DICKE4_IN2_GPIO_Port GPIOD
#define DICKE4_IN1_Pin GPIO_PIN_2
#define DICKE4_IN1_GPIO_Port GPIOD
#define CUR3_EN_Pin GPIO_PIN_3
#define CUR3_EN_GPIO_Port GPIOD
#define DICKE3_IN2_Pin GPIO_PIN_4
#define DICKE3_IN2_GPIO_Port GPIOD
#define DICKE3_IN1_Pin GPIO_PIN_5
#define DICKE3_IN1_GPIO_Port GPIOD
#define CUR2_EN_Pin GPIO_PIN_7
#define CUR2_EN_GPIO_Port GPIOD
#define DICKE2_IN2_Pin GPIO_PIN_9
#define DICKE2_IN2_GPIO_Port GPIOG
#define DICKE2_IN1_Pin GPIO_PIN_10
#define DICKE2_IN1_GPIO_Port GPIOG
#define CUR1_EN_Pin GPIO_PIN_11
#define CUR1_EN_GPIO_Port GPIOG
#define DICKE1_IN2_Pin GPIO_PIN_12
#define DICKE1_IN2_GPIO_Port GPIOG
#define DICKE1_IN1_Pin GPIO_PIN_13
#define DICKE1_IN1_GPIO_Port GPIOG
#define I2C1_SCL_POT1_Pin GPIO_PIN_6
#define I2C1_SCL_POT1_GPIO_Port GPIOB
#define I2C1_SDA_POT1_Pin GPIO_PIN_7
#define I2C1_SDA_POT1_GPIO_Port GPIOB
#define CUR1_A_Pin GPIO_PIN_8
#define CUR1_A_GPIO_Port GPIOB
#define CUR1_B_Pin GPIO_PIN_9
#define CUR1_B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
