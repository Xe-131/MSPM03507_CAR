################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
OLED_Hardware_I2C/%.o: ../OLED_Hardware_I2C/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/APPLICATION/TIccs2011/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/Xe-131/workspace_ccstheia/empty_LP_MSPM0G3507_nortos_ticlang/user" -I"C:/Users/Xe-131/workspace_ccstheia/empty_LP_MSPM0G3507_nortos_ticlang/motor" -I"C:/Users/Xe-131/workspace_ccstheia/empty_LP_MSPM0G3507_nortos_ticlang" -I"C:/Users/Xe-131/workspace_ccstheia/empty_LP_MSPM0G3507_nortos_ticlang/Debug" -I"C:/APPLICATION/ti_SDK/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/Core/Include" -I"C:/APPLICATION/ti_SDK/mspm0_sdk_2_04_00_06/source" -I"C:/Users/Xe-131/workspace_ccstheia/empty_LP_MSPM0G3507_nortos_ticlang/OLED_Hardware_I2C" -I"C:/Users/Xe-131/workspace_ccstheia/empty_LP_MSPM0G3507_nortos_ticlang/MSPM0" -I"C:/Users/Xe-131/workspace_ccstheia/empty_LP_MSPM0G3507_nortos_ticlang/MPU6050" -gdwarf-3 -MMD -MP -MF"OLED_Hardware_I2C/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


