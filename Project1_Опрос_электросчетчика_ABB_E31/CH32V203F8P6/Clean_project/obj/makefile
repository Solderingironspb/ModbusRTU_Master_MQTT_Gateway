################################################################################
# MRS Version: 1.9.1
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
Clean_project.hex \

SECONDARY_LIST += \
Clean_project.lst \

SECONDARY_SIZE += \
Clean_project.siz \


# All Target
all: Clean_project.elf secondary-outputs

# Tool invocations
Clean_project.elf: $(OBJS) $(USER_OBJS)
	@	@	riscv-none-embed-gcc -Wl,--print-memory-usage -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -T "C:\Users\Admin\Desktop\RS485_MQTT\Opros_ABB\CH32V203F8P6\Clean_project\Link.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"Clean_project.map" --specs=nano.specs --specs=nosys.specs -o "Clean_project.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@	@
Clean_project.hex: Clean_project.elf
	@	riscv-none-embed-objcopy -O ihex "Clean_project.elf"  "Clean_project.hex"
	@	@
Clean_project.lst: Clean_project.elf
	@	riscv-none-embed-objdump --all-headers --demangle --disassemble -M xw "Clean_project.elf" > "Clean_project.lst"
	@	@
Clean_project.siz: Clean_project.elf
	@	riscv-none-embed-size --format=berkeley "Clean_project.elf"
	@	@
# Other Targets
clean:
	-$(RM) $(ASM_UPPER_DEPS)$(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS) Clean_project.elf
	-@
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
