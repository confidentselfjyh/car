################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
c:/Users/32655/Desktop/my\ car/Seekfree_CH32V307VCT6_Opensource_Library/libraries/sdk/Startup/startup_ch32v30x_D8C.S 

S_UPPER_DEPS += \
./sdk/Startup/startup_ch32v30x_D8C.d 

OBJS += \
./sdk/Startup/startup_ch32v30x_D8C.o 

DIR_OBJS += \
./sdk/Startup/*.o \

DIR_DEPS += \
./sdk/Startup/*.d \

DIR_EXPANDS += \
./sdk/Startup/*.234r.expand \


# Each subdirectory must supply rules for building sources it contributes
sdk/Startup/startup_ch32v30x_D8C.o: c:/Users/32655/Desktop/my\ car/Seekfree_CH32V307VCT6_Opensource_Library/libraries/sdk/Startup/startup_ch32v30x_D8C.S
	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -fmax-errors=20 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall -g -x assembler -I"c:/Users/32655/Desktop/my car/Seekfree_CH32V307VCT6_Opensource_Library/libraries/sdk/Startup" -I"c:/Users/32655/Desktop/my car/Seekfree_CH32V307VCT6_Opensource_Library/libraries/zf_driver" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

