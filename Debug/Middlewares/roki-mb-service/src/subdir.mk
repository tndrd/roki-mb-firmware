################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Middlewares/roki-mb-service/src/MbMessages.cpp \
../Middlewares/roki-mb-service/src/MbService.cpp 

OBJS += \
./Middlewares/roki-mb-service/src/MbMessages.o \
./Middlewares/roki-mb-service/src/MbService.o 

CPP_DEPS += \
./Middlewares/roki-mb-service/src/MbMessages.d \
./Middlewares/roki-mb-service/src/MbService.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/roki-mb-service/src/%.o Middlewares/roki-mb-service/src/%.su: ../Middlewares/roki-mb-service/src/%.cpp Middlewares/roki-mb-service/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-roki-2d-mb-2d-service-2f-src

clean-Middlewares-2f-roki-2d-mb-2d-service-2f-src:
	-$(RM) ./Middlewares/roki-mb-service/src/MbMessages.d ./Middlewares/roki-mb-service/src/MbMessages.o ./Middlewares/roki-mb-service/src/MbMessages.su ./Middlewares/roki-mb-service/src/MbService.d ./Middlewares/roki-mb-service/src/MbService.o ./Middlewares/roki-mb-service/src/MbService.su

.PHONY: clean-Middlewares-2f-roki-2d-mb-2d-service-2f-src

