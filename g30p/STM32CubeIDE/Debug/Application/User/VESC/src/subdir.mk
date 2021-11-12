################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Git/M365/Merged/common_files/src/VescCommand.c \
C:/Git/M365/Merged/common_files/src/VescToSTM.c \
C:/Git/M365/Merged/common_files/src/app.c \
C:/Git/M365/Merged/common_files/src/app_uartcomm.c \
C:/Git/M365/Merged/common_files/src/buffer.c \
C:/Git/M365/Merged/common_files/src/conf_general.c \
C:/Git/M365/Merged/common_files/src/confgenerator.c \
C:/Git/M365/Merged/common_files/src/crc.c \
C:/Git/M365/Merged/common_files/src/packet.c \
C:/Git/M365/Merged/common_files/src/terminal.c \
C:/Git/M365/Merged/common_files/src/tune.c \
C:/Git/M365/Merged/common_files/src/utils.c 

OBJS += \
./Application/User/VESC/src/VescCommand.o \
./Application/User/VESC/src/VescToSTM.o \
./Application/User/VESC/src/app.o \
./Application/User/VESC/src/app_uartcomm.o \
./Application/User/VESC/src/buffer.o \
./Application/User/VESC/src/conf_general.o \
./Application/User/VESC/src/confgenerator.o \
./Application/User/VESC/src/crc.o \
./Application/User/VESC/src/packet.o \
./Application/User/VESC/src/terminal.o \
./Application/User/VESC/src/tune.o \
./Application/User/VESC/src/utils.o 

C_DEPS += \
./Application/User/VESC/src/VescCommand.d \
./Application/User/VESC/src/VescToSTM.d \
./Application/User/VESC/src/app.d \
./Application/User/VESC/src/app_uartcomm.d \
./Application/User/VESC/src/buffer.d \
./Application/User/VESC/src/conf_general.d \
./Application/User/VESC/src/confgenerator.d \
./Application/User/VESC/src/crc.d \
./Application/User/VESC/src/packet.d \
./Application/User/VESC/src/terminal.d \
./Application/User/VESC/src/tune.d \
./Application/User/VESC/src/utils.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/VESC/src/VescCommand.o: C:/Git/M365/Merged/common_files/src/VescCommand.c Application/User/VESC/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/VESC/src/VescToSTM.o: C:/Git/M365/Merged/common_files/src/VescToSTM.c Application/User/VESC/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/VESC/src/app.o: C:/Git/M365/Merged/common_files/src/app.c Application/User/VESC/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/VESC/src/app_uartcomm.o: C:/Git/M365/Merged/common_files/src/app_uartcomm.c Application/User/VESC/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/VESC/src/buffer.o: C:/Git/M365/Merged/common_files/src/buffer.c Application/User/VESC/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/VESC/src/conf_general.o: C:/Git/M365/Merged/common_files/src/conf_general.c Application/User/VESC/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/VESC/src/confgenerator.o: C:/Git/M365/Merged/common_files/src/confgenerator.c Application/User/VESC/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/VESC/src/crc.o: C:/Git/M365/Merged/common_files/src/crc.c Application/User/VESC/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/VESC/src/packet.o: C:/Git/M365/Merged/common_files/src/packet.c Application/User/VESC/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/VESC/src/terminal.o: C:/Git/M365/Merged/common_files/src/terminal.c Application/User/VESC/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/VESC/src/tune.o: C:/Git/M365/Merged/common_files/src/tune.c Application/User/VESC/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/User/VESC/src/utils.o: C:/Git/M365/Merged/common_files/src/utils.c Application/User/VESC/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DG30P -DARM_MATH_CM3 -DSTM32F103xB -DDEBUG -c -I../../Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc -I../../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../../../common_files/MotorControl/MCSDK/MCLib/Any/Inc -I../../../common_files/src -I../../../common_files/inc -I../../../common_files/MotorControl/MCSDK/MCLib/F1xx/Inc -I../../../common_files/MotorControl/MCSDK/UILibrary/Inc -I../../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/CMSIS/DSP/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../../Middlewares/Third_Party/FreeRTOS/Source/include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

