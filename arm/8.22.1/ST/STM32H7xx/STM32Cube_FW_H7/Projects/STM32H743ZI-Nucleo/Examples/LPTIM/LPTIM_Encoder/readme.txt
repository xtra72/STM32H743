/**
  @page LPTIM_Encoder example

  @verbatim
  ********************* COPYRIGHT(c) 2017 STMicroelectronics *******************
  * @file    LPTIM/LPTIM_Encoder/readme.txt
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    21-April-2017
  * @brief   Description of the LPTIM Encoder mode
  ******************************************************************************
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
  @endverbatim

@par Example Description

This example shows how to configure the LPTIM peripheral in Encoder mode.

At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the Flash interface and the systick.
The SystemClock_Config() function is used to configure the system clock for STM32H743xx Devices :
The CPU at 400MHz 
The HCLK for D1 Domain AXI and AHB3 peripherals , D2 Domain AHB1/AHB2 peripherals and D3 Domain AHB4  peripherals at 200MHz.
The APB clock dividers for D1 Domain APB3 peripherals, D2 Domain APB1 and APB2 peripherals and D3 Domain APB4 peripherals to  run at 100MHz.

In this example, the LPTIM encoder is configured to generate up or down counting
according to the phase between the two inputs of LPTIM1.

In this exemple :

        - At the first there is no change of state , LED1 & LED3 are Off .
        - If LED3 On and LED1 Off , the LPTIM encoder mode is in up counting   .
        - If LED1 On and LED3 Off , the LPTIM encoder is mode in down counting .
        - In case of error , LED1 & LED 3 On


To change the direction of counting switch the inputs 1 & 2 of LPTIM1.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.

@note The example need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents

  - LPTIM/LPTIM_Encoder/Inc/stm32h7xx_hal_conf.h    HAL configuration file
  - LPTIM/LPTIM_Encoder/Inc/stm32h7xx_it.h          Interrupt handlers header file
  - LPTIM/LPTIM_Encoder/Inc/main.h                  Header for main.c module
  - LPTIM/LPTIM_Encoder/Src/stm32h7xx_it.c          Interrupt handlers
  - LPTIM/LPTIM_Encoder/Src/main.c                  Main program
  - LPTIM/LPTIM_Encoder/Src/stm32h7xx_hal_msp.c     HAL MSP file
  - LPTIM/LPTIM_Encoder/Src/system_stm32h7xx.c      STM32H7xx system source file


@par Hardware and Software environment

  - This example runs on STM32H743xx devices.

  - This example has been tested with STMicroelectronics STM32H743ZI-NUCLEO
    board and can be easily tailored to any other supported device
    and development board.

  - STM32H743ZI-NUCLEO Set-up

 Use GPIOx to generate 2 square signals with different phases
   Connect the following pins of GPIOx outputs :
        - GPIOB : pin (PB.12)( pin 16 in CN 12 connector )
        - GPIOB : pin (PB.13)(pin 30 in CN 12 connector)
   To the following inputs of LPTIM 1 using wires :
        - Input 1 : pin (PD.12)(pin 43 in CN12 connector)
        - Input 2 : pin (PE.1)(pin 61 in CN 11 connector)
   

@par How to use it ?

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
