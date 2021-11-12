################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Git/M365/Merged/g30p/Src/freertos.c \
C:/Git/M365/Merged/g30p/Src/main.c \
../Application/User/ntc.c \
C:/Git/M365/Merged/common_files/src/printf.c \
C:/Git/M365/Merged/common_files/src/stm32f10x_mc_it.c \
C:/Git/M365/Merged/g30p/Src/stm32f1xx_hal_msp.c \
C:/Git/M365/Merged/g30p/Src/stm32f1xx_hal_timebase_tim.c \
C:/Git/M365/Merged/g30p/Src/stm32f1xx_it.c \
../Application/User/syscalls.c \
../Application/User/sysmem.c \
C:/Git/M365/Merged/common_files/src/system.c \
C:/Git/M365/Merged/common_files/src/task_LED.c \
C:/Git/M365/Merged/common_files/src/task_cli.c \
C:/Git/M365/Merged/common_files/src/task_init.c \
../Application/User/task_pwr.c 

OBJS += \
./Application/User/freertos.o \
./Application/User/main.o \
./Application/User/ntc.o \
./Application/User/printf.o \
./Application/User/stm32f10x_mc_it.o \
./Application/User/stm32f1xx_hal_msp.o \
./Application/User/stm32f1xx_hal_timebase_tim.o \
./Application/User/stm32f1xx_it.o \
./Application/User/syscalls.o \
./Application/User/sysmem.o \
./Application/User/system.o \
./Application/User/task_LED.o \
./Application/User/task_cli.o \
./Application/User/task_init.o \
./Application/User/task_pwr.o 

C_DEPS += \
./Application/User/freertos.d \
./Application/User/main.d \
./Application/User/ntc.d \
./Application/User/printf.d \
./Application/User/stm32f10x_mc_it.d \
./Application/User/stm32f1xx_hal_msp.d \
./Application/User/stm32f1xx_hal_timebase_tim.d \
./Application/User/stm32f1xx_it.d \
./Application/User/syscalls.d \
./Application/User/sysmem.d \
./Application/User/system.d \
./Application/User/task_LED.d \
./Application/User/task_cli.d \
./Application/User/task_init.d \
./Application/User/task_pwr.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/freertos.o: C:/Git/M365/Merged/g30p/Src/freertos.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/main.o: C:/Git/M365/Merged/g30p/Src/main.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/%.o: ../Application/User/%.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/printf.o: C:/Git/M365/Merged/common_files/src/printf.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/stm32f10x_mc_it.o: C:/Git/M365/Merged/common_files/src/stm32f10x_mc_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/stm32f1xx_hal_msp.o: C:/Git/M365/Merged/g30p/Src/stm32f1xx_hal_msp.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/stm32f1xx_hal_timebase_tim.o: C:/Git/M365/Merged/g30p/Src/stm32f1xx_hal_timebase_tim.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/stm32f1xx_it.o: C:/Git/M365/Merged/g30p/Src/stm32f1xx_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/system.o: C:/Git/M365/Merged/common_files/src/system.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/task_LED.o: C:/Git/M365/Merged/common_files/src/task_LED.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/task_cli.o: C:/Git/M365/Merged/common_files/src/task_cli.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/task_init.o: C:/Git/M365/Merged/common_files/src/task_init.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

