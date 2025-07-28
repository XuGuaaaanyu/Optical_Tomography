################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/AD717x/ad717x.c \
../Drivers/AD717x/no_os_alloc.c \
../Drivers/AD717x/no_os_spi.c \
../Drivers/AD717x/no_os_util.c 

OBJS += \
./Drivers/AD717x/ad717x.o \
./Drivers/AD717x/no_os_alloc.o \
./Drivers/AD717x/no_os_spi.o \
./Drivers/AD717x/no_os_util.o 

C_DEPS += \
./Drivers/AD717x/ad717x.d \
./Drivers/AD717x/no_os_alloc.d \
./Drivers/AD717x/no_os_spi.d \
./Drivers/AD717x/no_os_util.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/AD717x/%.o Drivers/AD717x/%.su Drivers/AD717x/%.cyclo: ../Drivers/AD717x/%.c Drivers/AD717x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/AD717x -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-AD717x

clean-Drivers-2f-AD717x:
	-$(RM) ./Drivers/AD717x/ad717x.cyclo ./Drivers/AD717x/ad717x.d ./Drivers/AD717x/ad717x.o ./Drivers/AD717x/ad717x.su ./Drivers/AD717x/no_os_alloc.cyclo ./Drivers/AD717x/no_os_alloc.d ./Drivers/AD717x/no_os_alloc.o ./Drivers/AD717x/no_os_alloc.su ./Drivers/AD717x/no_os_spi.cyclo ./Drivers/AD717x/no_os_spi.d ./Drivers/AD717x/no_os_spi.o ./Drivers/AD717x/no_os_spi.su ./Drivers/AD717x/no_os_util.cyclo ./Drivers/AD717x/no_os_util.d ./Drivers/AD717x/no_os_util.o ./Drivers/AD717x/no_os_util.su

.PHONY: clean-Drivers-2f-AD717x

