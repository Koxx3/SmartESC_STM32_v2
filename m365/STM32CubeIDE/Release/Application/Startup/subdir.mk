################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Application/Startup/startup_stm32f103c8tx.s 

OBJS += \
./Application/Startup/startup_stm32f103c8tx.o 

S_DEPS += \
./Application/Startup/startup_stm32f103c8tx.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Startup/startup_stm32f103c8tx.o: ../Application/Startup/startup_stm32f103c8tx.s
	arm-none-eabi-gcc -mcpu=cortex-m3 -c -x assembler-with-cpp -MMD -MP -MF"Application/Startup/startup_stm32f103c8tx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

