################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/list.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/port.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/queue.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/timers.c 

OBJS += \
./Middlewares/FreeRTOS/cmsis_os2.o \
./Middlewares/FreeRTOS/croutine.o \
./Middlewares/FreeRTOS/event_groups.o \
./Middlewares/FreeRTOS/heap_4.o \
./Middlewares/FreeRTOS/list.o \
./Middlewares/FreeRTOS/port.o \
./Middlewares/FreeRTOS/queue.o \
./Middlewares/FreeRTOS/stream_buffer.o \
./Middlewares/FreeRTOS/tasks.o \
./Middlewares/FreeRTOS/timers.o 

C_DEPS += \
./Middlewares/FreeRTOS/cmsis_os2.d \
./Middlewares/FreeRTOS/croutine.d \
./Middlewares/FreeRTOS/event_groups.d \
./Middlewares/FreeRTOS/heap_4.d \
./Middlewares/FreeRTOS/list.d \
./Middlewares/FreeRTOS/port.d \
./Middlewares/FreeRTOS/queue.d \
./Middlewares/FreeRTOS/stream_buffer.d \
./Middlewares/FreeRTOS/tasks.d \
./Middlewares/FreeRTOS/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/FreeRTOS/cmsis_os2.o: C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FreeRTOS/cmsis_os2.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/FreeRTOS/croutine.o: C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/croutine.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FreeRTOS/croutine.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/FreeRTOS/event_groups.o: C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FreeRTOS/event_groups.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/FreeRTOS/heap_4.o: C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FreeRTOS/heap_4.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/FreeRTOS/list.o: C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/list.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FreeRTOS/list.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/FreeRTOS/port.o: C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/port.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FreeRTOS/port.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/FreeRTOS/queue.o: C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/queue.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FreeRTOS/queue.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/FreeRTOS/stream_buffer.o: C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FreeRTOS/stream_buffer.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/FreeRTOS/tasks.o: C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FreeRTOS/tasks.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Middlewares/FreeRTOS/timers.o: C:/VSARM/Workspace/M365_MC_WB/m365/Middlewares/Third_Party/FreeRTOS/Source/timers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DARM_MATH_CM3 -DSTM32F103xB -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.5/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/FreeRTOS/timers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

