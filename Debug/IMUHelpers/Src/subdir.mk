################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../IMUHelpers/Src/bhy2.c \
../IMUHelpers/Src/bhy2_hif.c \
../IMUHelpers/Src/bhy2_parse.c 

CPP_SRCS += \
../IMUHelpers/Src/IMU_funcs.cpp 

C_DEPS += \
./IMUHelpers/Src/bhy2.d \
./IMUHelpers/Src/bhy2_hif.d \
./IMUHelpers/Src/bhy2_parse.d 

OBJS += \
./IMUHelpers/Src/IMU_funcs.o \
./IMUHelpers/Src/bhy2.o \
./IMUHelpers/Src/bhy2_hif.o \
./IMUHelpers/Src/bhy2_parse.o 

CPP_DEPS += \
./IMUHelpers/Src/IMU_funcs.d 


# Each subdirectory must supply rules for building sources it contributes
IMUHelpers/Src/%.o IMUHelpers/Src/%.su: ../IMUHelpers/Src/%.cpp IMUHelpers/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
IMUHelpers/Src/%.o IMUHelpers/Src/%.su: ../IMUHelpers/Src/%.c IMUHelpers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-IMUHelpers-2f-Src

clean-IMUHelpers-2f-Src:
	-$(RM) ./IMUHelpers/Src/IMU_funcs.d ./IMUHelpers/Src/IMU_funcs.o ./IMUHelpers/Src/IMU_funcs.su ./IMUHelpers/Src/bhy2.d ./IMUHelpers/Src/bhy2.o ./IMUHelpers/Src/bhy2.su ./IMUHelpers/Src/bhy2_hif.d ./IMUHelpers/Src/bhy2_hif.o ./IMUHelpers/Src/bhy2_hif.su ./IMUHelpers/Src/bhy2_parse.d ./IMUHelpers/Src/bhy2_parse.o ./IMUHelpers/Src/bhy2_parse.su

.PHONY: clean-IMUHelpers-2f-Src

