################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Inc/DHT22_TempHumidity.cpp \
../Core/Inc/LSM6_IMU.cpp \
../Core/Inc/Step_Motor.cpp 

OBJS += \
./Core/Inc/DHT22_TempHumidity.o \
./Core/Inc/LSM6_IMU.o \
./Core/Inc/Step_Motor.o 

CPP_DEPS += \
./Core/Inc/DHT22_TempHumidity.d \
./Core/Inc/LSM6_IMU.d \
./Core/Inc/Step_Motor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su: ../Core/Inc/%.cpp Core/Inc/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/DHT22_TempHumidity.d ./Core/Inc/DHT22_TempHumidity.o ./Core/Inc/DHT22_TempHumidity.su ./Core/Inc/LSM6_IMU.d ./Core/Inc/LSM6_IMU.o ./Core/Inc/LSM6_IMU.su ./Core/Inc/Step_Motor.d ./Core/Inc/Step_Motor.o ./Core/Inc/Step_Motor.su

.PHONY: clean-Core-2f-Inc

