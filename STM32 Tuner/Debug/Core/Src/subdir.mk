################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/arm_bitreversal2.c \
../Core/Src/arm_cfft_f32.c \
../Core/Src/arm_cfft_init_f32.c \
../Core/Src/arm_cfft_radix4_f32.c \
../Core/Src/arm_cfft_radix4_init_f32.c \
../Core/Src/arm_cfft_radix8_f32.c \
../Core/Src/arm_cmplx_mag_f32.c \
../Core/Src/arm_common_tables.c \
../Core/Src/arm_const_structs.c \
../Core/Src/arm_cos_f32.c \
../Core/Src/arm_max_f32.c \
../Core/Src/arm_mult_f32.c \
../Core/Src/arm_mve_tables.c \
../Core/Src/arm_mve_tables_f16.c \
../Core/Src/arm_neon_tables.c \
../Core/Src/arm_neon_tables_f16.c \
../Core/Src/arm_power_f32.c \
../Core/Src/arm_rfft_fast_f32.c \
../Core/Src/arm_rfft_fast_init_f16.c \
../Core/Src/arm_rfft_fast_init_f32.c \
../Core/Src/arm_rfft_fast_init_f64.c \
../Core/Src/freertos.c \
../Core/Src/freq.c \
../Core/Src/main.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c 

OBJS += \
./Core/Src/arm_bitreversal2.o \
./Core/Src/arm_cfft_f32.o \
./Core/Src/arm_cfft_init_f32.o \
./Core/Src/arm_cfft_radix4_f32.o \
./Core/Src/arm_cfft_radix4_init_f32.o \
./Core/Src/arm_cfft_radix8_f32.o \
./Core/Src/arm_cmplx_mag_f32.o \
./Core/Src/arm_common_tables.o \
./Core/Src/arm_const_structs.o \
./Core/Src/arm_cos_f32.o \
./Core/Src/arm_max_f32.o \
./Core/Src/arm_mult_f32.o \
./Core/Src/arm_mve_tables.o \
./Core/Src/arm_mve_tables_f16.o \
./Core/Src/arm_neon_tables.o \
./Core/Src/arm_neon_tables_f16.o \
./Core/Src/arm_power_f32.o \
./Core/Src/arm_rfft_fast_f32.o \
./Core/Src/arm_rfft_fast_init_f16.o \
./Core/Src/arm_rfft_fast_init_f32.o \
./Core/Src/arm_rfft_fast_init_f64.o \
./Core/Src/freertos.o \
./Core/Src/freq.o \
./Core/Src/main.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o 

C_DEPS += \
./Core/Src/arm_bitreversal2.d \
./Core/Src/arm_cfft_f32.d \
./Core/Src/arm_cfft_init_f32.d \
./Core/Src/arm_cfft_radix4_f32.d \
./Core/Src/arm_cfft_radix4_init_f32.d \
./Core/Src/arm_cfft_radix8_f32.d \
./Core/Src/arm_cmplx_mag_f32.d \
./Core/Src/arm_common_tables.d \
./Core/Src/arm_const_structs.d \
./Core/Src/arm_cos_f32.d \
./Core/Src/arm_max_f32.d \
./Core/Src/arm_mult_f32.d \
./Core/Src/arm_mve_tables.d \
./Core/Src/arm_mve_tables_f16.d \
./Core/Src/arm_neon_tables.d \
./Core/Src/arm_neon_tables_f16.d \
./Core/Src/arm_power_f32.d \
./Core/Src/arm_rfft_fast_f32.d \
./Core/Src/arm_rfft_fast_init_f16.d \
./Core/Src/arm_rfft_fast_init_f32.d \
./Core/Src/arm_rfft_fast_init_f64.d \
./Core/Src/freertos.d \
./Core/Src/freq.d \
./Core/Src/main.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/arm_bitreversal2.cyclo ./Core/Src/arm_bitreversal2.d ./Core/Src/arm_bitreversal2.o ./Core/Src/arm_bitreversal2.su ./Core/Src/arm_cfft_f32.cyclo ./Core/Src/arm_cfft_f32.d ./Core/Src/arm_cfft_f32.o ./Core/Src/arm_cfft_f32.su ./Core/Src/arm_cfft_init_f32.cyclo ./Core/Src/arm_cfft_init_f32.d ./Core/Src/arm_cfft_init_f32.o ./Core/Src/arm_cfft_init_f32.su ./Core/Src/arm_cfft_radix4_f32.cyclo ./Core/Src/arm_cfft_radix4_f32.d ./Core/Src/arm_cfft_radix4_f32.o ./Core/Src/arm_cfft_radix4_f32.su ./Core/Src/arm_cfft_radix4_init_f32.cyclo ./Core/Src/arm_cfft_radix4_init_f32.d ./Core/Src/arm_cfft_radix4_init_f32.o ./Core/Src/arm_cfft_radix4_init_f32.su ./Core/Src/arm_cfft_radix8_f32.cyclo ./Core/Src/arm_cfft_radix8_f32.d ./Core/Src/arm_cfft_radix8_f32.o ./Core/Src/arm_cfft_radix8_f32.su ./Core/Src/arm_cmplx_mag_f32.cyclo ./Core/Src/arm_cmplx_mag_f32.d ./Core/Src/arm_cmplx_mag_f32.o ./Core/Src/arm_cmplx_mag_f32.su ./Core/Src/arm_common_tables.cyclo ./Core/Src/arm_common_tables.d ./Core/Src/arm_common_tables.o ./Core/Src/arm_common_tables.su ./Core/Src/arm_const_structs.cyclo ./Core/Src/arm_const_structs.d ./Core/Src/arm_const_structs.o ./Core/Src/arm_const_structs.su ./Core/Src/arm_cos_f32.cyclo ./Core/Src/arm_cos_f32.d ./Core/Src/arm_cos_f32.o ./Core/Src/arm_cos_f32.su ./Core/Src/arm_max_f32.cyclo ./Core/Src/arm_max_f32.d ./Core/Src/arm_max_f32.o ./Core/Src/arm_max_f32.su ./Core/Src/arm_mult_f32.cyclo ./Core/Src/arm_mult_f32.d ./Core/Src/arm_mult_f32.o ./Core/Src/arm_mult_f32.su ./Core/Src/arm_mve_tables.cyclo ./Core/Src/arm_mve_tables.d ./Core/Src/arm_mve_tables.o ./Core/Src/arm_mve_tables.su ./Core/Src/arm_mve_tables_f16.cyclo ./Core/Src/arm_mve_tables_f16.d ./Core/Src/arm_mve_tables_f16.o ./Core/Src/arm_mve_tables_f16.su ./Core/Src/arm_neon_tables.cyclo ./Core/Src/arm_neon_tables.d ./Core/Src/arm_neon_tables.o ./Core/Src/arm_neon_tables.su ./Core/Src/arm_neon_tables_f16.cyclo ./Core/Src/arm_neon_tables_f16.d ./Core/Src/arm_neon_tables_f16.o ./Core/Src/arm_neon_tables_f16.su ./Core/Src/arm_power_f32.cyclo ./Core/Src/arm_power_f32.d ./Core/Src/arm_power_f32.o ./Core/Src/arm_power_f32.su ./Core/Src/arm_rfft_fast_f32.cyclo ./Core/Src/arm_rfft_fast_f32.d ./Core/Src/arm_rfft_fast_f32.o ./Core/Src/arm_rfft_fast_f32.su ./Core/Src/arm_rfft_fast_init_f16.cyclo ./Core/Src/arm_rfft_fast_init_f16.d ./Core/Src/arm_rfft_fast_init_f16.o ./Core/Src/arm_rfft_fast_init_f16.su ./Core/Src/arm_rfft_fast_init_f32.cyclo ./Core/Src/arm_rfft_fast_init_f32.d ./Core/Src/arm_rfft_fast_init_f32.o ./Core/Src/arm_rfft_fast_init_f32.su ./Core/Src/arm_rfft_fast_init_f64.cyclo ./Core/Src/arm_rfft_fast_init_f64.d ./Core/Src/arm_rfft_fast_init_f64.o ./Core/Src/arm_rfft_fast_init_f64.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/freq.cyclo ./Core/Src/freq.d ./Core/Src/freq.o ./Core/Src/freq.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32l4xx_hal_msp.cyclo ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.cyclo ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.cyclo ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su

.PHONY: clean-Core-2f-Src

