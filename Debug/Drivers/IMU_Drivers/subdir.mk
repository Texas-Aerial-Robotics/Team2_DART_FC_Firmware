################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/IMU_Drivers/MPU6500.c 

OBJS += \
./Drivers/IMU_Drivers/MPU6500.o 

C_DEPS += \
./Drivers/IMU_Drivers/MPU6500.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/IMU_Drivers/%.o Drivers/IMU_Drivers/%.su Drivers/IMU_Drivers/%.cyclo: ../Drivers/IMU_Drivers/%.c Drivers/IMU_Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/IMU_Drivers -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-IMU_Drivers

clean-Drivers-2f-IMU_Drivers:
	-$(RM) ./Drivers/IMU_Drivers/MPU6500.cyclo ./Drivers/IMU_Drivers/MPU6500.d ./Drivers/IMU_Drivers/MPU6500.o ./Drivers/IMU_Drivers/MPU6500.su

.PHONY: clean-Drivers-2f-IMU_Drivers

