################################################################################
# MRS Version: 1.9.1
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ESP8266_lib.c \
../Core/Src/MQTT.c \
../Core/Src/ModbusRTU.c \
../Core/Src/ModbusRTU_Master.c \
../Core/Src/SoftwareTimer.c \
../Core/Src/ch32v203xx_it.c \
../Core/Src/ch32v20x_RVMSIS.c \
../Core/Src/main.c \
../Core/Src/syscalls.c 

OBJS += \
./Core/Src/ESP8266_lib.o \
./Core/Src/MQTT.o \
./Core/Src/ModbusRTU.o \
./Core/Src/ModbusRTU_Master.o \
./Core/Src/SoftwareTimer.o \
./Core/Src/ch32v203xx_it.o \
./Core/Src/ch32v20x_RVMSIS.o \
./Core/Src/main.o \
./Core/Src/syscalls.o 

C_DEPS += \
./Core/Src/ESP8266_lib.d \
./Core/Src/MQTT.d \
./Core/Src/ModbusRTU.d \
./Core/Src/ModbusRTU_Master.d \
./Core/Src/SoftwareTimer.d \
./Core/Src/ch32v203xx_it.d \
./Core/Src/ch32v20x_RVMSIS.d \
./Core/Src/main.d \
./Core/Src/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I../Core/Inc -I../Drivers/inc -std=c11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

