################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../basicFunctions/API.c \
../basicFunctions/motors.c \
../basicFunctions/queues.c \
../basicFunctions/solver.c \
../basicFunctions/tofCalc.c \
../basicFunctions/wallCheck.c \
../basicFunctions/wallFollow.c 

OBJS += \
./basicFunctions/API.o \
./basicFunctions/motors.o \
./basicFunctions/queues.o \
./basicFunctions/solver.o \
./basicFunctions/tofCalc.o \
./basicFunctions/wallCheck.o \
./basicFunctions/wallFollow.o 

C_DEPS += \
./basicFunctions/API.d \
./basicFunctions/motors.d \
./basicFunctions/queues.d \
./basicFunctions/solver.d \
./basicFunctions/tofCalc.d \
./basicFunctions/wallCheck.d \
./basicFunctions/wallFollow.d 


# Each subdirectory must supply rules for building sources it contributes
basicFunctions/%.o basicFunctions/%.su basicFunctions/%.cyclo: ../basicFunctions/%.c basicFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/user/Desktop/stm32Board/Tof with motors/Tof with motors/basicFunctions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-basicFunctions

clean-basicFunctions:
	-$(RM) ./basicFunctions/API.cyclo ./basicFunctions/API.d ./basicFunctions/API.o ./basicFunctions/API.su ./basicFunctions/motors.cyclo ./basicFunctions/motors.d ./basicFunctions/motors.o ./basicFunctions/motors.su ./basicFunctions/queues.cyclo ./basicFunctions/queues.d ./basicFunctions/queues.o ./basicFunctions/queues.su ./basicFunctions/solver.cyclo ./basicFunctions/solver.d ./basicFunctions/solver.o ./basicFunctions/solver.su ./basicFunctions/tofCalc.cyclo ./basicFunctions/tofCalc.d ./basicFunctions/tofCalc.o ./basicFunctions/tofCalc.su ./basicFunctions/wallCheck.cyclo ./basicFunctions/wallCheck.d ./basicFunctions/wallCheck.o ./basicFunctions/wallCheck.su ./basicFunctions/wallFollow.cyclo ./basicFunctions/wallFollow.d ./basicFunctions/wallFollow.o ./basicFunctions/wallFollow.su

.PHONY: clean-basicFunctions

