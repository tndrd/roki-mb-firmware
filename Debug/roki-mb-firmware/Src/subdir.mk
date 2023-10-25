################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../roki-mb-firmware/Src/MotherboardMain.cpp 

OBJS += \
./roki-mb-firmware/Src/MotherboardMain.o 

CPP_DEPS += \
./roki-mb-firmware/Src/MotherboardMain.d 


# Each subdirectory must supply rules for building sources it contributes
roki-mb-firmware/Src/MotherboardMain.o: ../roki-mb-firmware/Src/MotherboardMain.cpp roki-mb-firmware/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"/home/tndrd/STM32CubeIDE/workspace_1.10.1/StarkitMotherboard/roki-mb-firmware/Inc" -I"/home/tndrd/STM32CubeIDE/workspace_1.10.1/StarkitMotherboard/roki-mb-service/inc" -I"/home/tndrd/STM32CubeIDE/workspace_1.10.1/StarkitMotherboard/IMUHelpers/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-roki-2d-mb-2d-firmware-2f-Src

clean-roki-2d-mb-2d-firmware-2f-Src:
	-$(RM) ./roki-mb-firmware/Src/MotherboardMain.d ./roki-mb-firmware/Src/MotherboardMain.o ./roki-mb-firmware/Src/MotherboardMain.su

.PHONY: clean-roki-2d-mb-2d-firmware-2f-Src

