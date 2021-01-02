################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c 

OBJS += \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal.o \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_adc.o \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_adc_ex.o \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_cortex.o \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_dma.o \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_exti.o \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_flash.o \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_flash_ex.o \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_gpio.o \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_gpio_ex.o \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_pwr.o \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_rcc.o \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_rcc_ex.o \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_tim.o \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_tim_ex.o \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_uart.o 

C_DEPS += \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal.d \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_adc.d \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_adc_ex.d \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_cortex.d \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_dma.d \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_exti.d \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_flash.d \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_flash_ex.d \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_gpio.d \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_gpio_ex.d \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_pwr.d \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_rcc.d \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_rcc_ex.d \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_tim.d \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_tim_ex.d \
./Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_uart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal.o: C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_adc.o: C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_adc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_adc_ex.o: C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_adc_ex.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_cortex.o: C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_cortex.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_dma.o: C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_dma.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_exti.o: C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_exti.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_flash.o: C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_flash.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_flash_ex.o: C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_flash_ex.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_gpio.o: C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_gpio.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_gpio_ex.o: C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_gpio_ex.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_pwr.o: C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_pwr.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_rcc.o: C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_rcc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_rcc_ex.o: C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_rcc_ex.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_tim.o: C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_tim.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_tim_ex.o: C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_tim_ex.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_uart.o: C:/VSARM/Workspace/M365_MC_WB/m365/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F1xx_HAL_Driver/stm32f1xx_hal_uart.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

