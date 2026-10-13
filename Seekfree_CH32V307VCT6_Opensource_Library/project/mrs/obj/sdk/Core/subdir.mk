################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
c:/Users/32655/Desktop/my\ car/Seekfree_CH32V307VCT6_Opensource_Library/libraries/sdk/Core/core_riscv.c 

C_DEPS += \
./sdk/Core/core_riscv.d 

OBJS += \
./sdk/Core/core_riscv.o 

DIR_OBJS += \
./sdk/Core/*.o \

DIR_DEPS += \
./sdk/Core/*.d \

DIR_EXPANDS += \
./sdk/Core/*.234r.expand \


# Each subdirectory must supply rules for building sources it contributes
sdk/Core/core_riscv.o: c:/Users/32655/Desktop/my\ car/Seekfree_CH32V307VCT6_Opensource_Library/libraries/sdk/Core/core_riscv.c
	@	riscv-none-embed-gcc -march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -fmax-errors=20 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -pedantic -Wunused -Wuninitialized -Wall -g -I"c:/Users/32655/Desktop/my car/Seekfree_CH32V307VCT6_Opensource_Library/Libraries/doc" -I"c:/Users/32655/Desktop/my car/Seekfree_CH32V307VCT6_Opensource_Library/libraries/sdk/Core" -I"c:/Users/32655/Desktop/my car/Seekfree_CH32V307VCT6_Opensource_Library/libraries/sdk/Ld" -I"c:/Users/32655/Desktop/my car/Seekfree_CH32V307VCT6_Opensource_Library/libraries/sdk/Peripheral" -I"c:/Users/32655/Desktop/my car/Seekfree_CH32V307VCT6_Opensource_Library/libraries/sdk/Startup" -I"c:/Users/32655/Desktop/my car/Seekfree_CH32V307VCT6_Opensource_Library/project/user/inc" -I"c:/Users/32655/Desktop/my car/Seekfree_CH32V307VCT6_Opensource_Library/libraries/zf_common" -I"c:/Users/32655/Desktop/my car/Seekfree_CH32V307VCT6_Opensource_Library/libraries/zf_device" -I"c:/Users/32655/Desktop/my car/Seekfree_CH32V307VCT6_Opensource_Library/project/code" -I"c:/Users/32655/Desktop/my car/Seekfree_CH32V307VCT6_Opensource_Library/libraries/zf_driver" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

