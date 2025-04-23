################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Bsp/MPU_gy_521.c 

OBJS += \
./Bsp/MPU_gy_521.o 

C_DEPS += \
./Bsp/MPU_gy_521.d 


# Each subdirectory must supply rules for building sources it contributes
Bsp/%.o Bsp/%.su Bsp/%.cyclo: ../Bsp/%.c Bsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"/home/bright/STM32CubeIDE/chapter_1/MPU_GY521/Driver/Inc" -I"/home/bright/STM32CubeIDE/chapter_1/MPU_GY521/Bsp" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Bsp

clean-Bsp:
	-$(RM) ./Bsp/MPU_gy_521.cyclo ./Bsp/MPU_gy_521.d ./Bsp/MPU_gy_521.o ./Bsp/MPU_gy_521.su

.PHONY: clean-Bsp

