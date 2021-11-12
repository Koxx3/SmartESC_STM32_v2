################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Git/M365/Merged/common_files/src/mc_config.c \
C:/Git/M365/Merged/common_files/src/mc_interface.c \
C:/Git/M365/Merged/common_files/src/mc_math.c \
C:/Git/M365/Merged/common_files/src/mc_parameters.c \
C:/Git/M365/Merged/common_files/src/mc_tasks.c \
C:/Git/M365/Merged/common_files/src/motorcontrol.c \
C:/Git/M365/Merged/common_files/src/regular_conversion_manager.c 

OBJS += \
./Application/User/BLDC/mc_config.o \
./Application/User/BLDC/mc_interface.o \
./Application/User/BLDC/mc_math.o \
./Application/User/BLDC/mc_parameters.o \
./Application/User/BLDC/mc_tasks.o \
./Application/User/BLDC/motorcontrol.o \
./Application/User/BLDC/regular_conversion_manager.o 

C_DEPS += \
./Application/User/BLDC/mc_config.d \
./Application/User/BLDC/mc_interface.d \
./Application/User/BLDC/mc_math.d \
./Application/User/BLDC/mc_parameters.d \
./Application/User/BLDC/mc_tasks.d \
./Application/User/BLDC/motorcontrol.d \
./Application/User/BLDC/regular_conversion_manager.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/BLDC/mc_config.o: C:/Git/M365/Merged/common_files/src/mc_config.c Application/User/BLDC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/BLDC/mc_interface.o: C:/Git/M365/Merged/common_files/src/mc_interface.c Application/User/BLDC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/BLDC/mc_math.o: C:/Git/M365/Merged/common_files/src/mc_math.c Application/User/BLDC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/BLDC/mc_parameters.o: C:/Git/M365/Merged/common_files/src/mc_parameters.c Application/User/BLDC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/BLDC/mc_tasks.o: C:/Git/M365/Merged/common_files/src/mc_tasks.c Application/User/BLDC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/BLDC/motorcontrol.o: C:/Git/M365/Merged/common_files/src/motorcontrol.c Application/User/BLDC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/BLDC/regular_conversion_manager.o: C:/Git/M365/Merged/common_files/src/regular_conversion_manager.c Application/User/BLDC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

