################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/VSARM/Workspace/M365_MC_WB/m365/Src/system_stm32f1xx.c 

OBJS += \
./Drivers/CMSIS/system_stm32f1xx.o 

C_DEPS += \
./Drivers/CMSIS/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/system_stm32f1xx.o: C:/VSARM/Workspace/M365_MC_WB/m365/Src/system_stm32f1xx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/system_stm32f1xx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

