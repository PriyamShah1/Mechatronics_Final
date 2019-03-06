################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CFG_SRCS += \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/SYSBIOS/Lab5.cfg 

CMD_SRCS += \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_headers/cmd/DSP2833x_Headers_BIOS.cmd \
../TMS320F28335.cmd \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/cmd/user_lnk.cmd 

ASM_SRCS += \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_ADC_cal.asm \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_CSMPasswords.asm \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_CodeStartBranch.asm \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_usDelay.asm 

C_SRCS += \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/28335_dma.c \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/28335_eqep.c \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/28335_inits.c \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/28335_mcbsp.c \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/28335_pwm.c \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/28335_serial.c \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/28335_spi.c \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_Adc.c \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_CpuTimers.c \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_headers/source/DSP2833x_GlobalVariableDefs.c \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_Mcbsp.c \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_PieCtrl.c \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_Spi.c \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_SysCtrl.c \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/i2c.c \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/lcd.c \
C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/user_Lab5.c 

GEN_CMDS += \
./configPkg/linker.cmd 

GEN_FILES += \
./configPkg/linker.cmd \
./configPkg/compiler.opt 

GEN_MISC_DIRS += \
./configPkg/ 

C_DEPS += \
./28335_dma.d \
./28335_eqep.d \
./28335_inits.d \
./28335_mcbsp.d \
./28335_pwm.d \
./28335_serial.d \
./28335_spi.d \
./DSP2833x_Adc.d \
./DSP2833x_CpuTimers.d \
./DSP2833x_GlobalVariableDefs.d \
./DSP2833x_Mcbsp.d \
./DSP2833x_PieCtrl.d \
./DSP2833x_Spi.d \
./DSP2833x_SysCtrl.d \
./i2c.d \
./lcd.d \
./user_Lab5.d 

GEN_OPTS += \
./configPkg/compiler.opt 

OBJS += \
./28335_dma.obj \
./28335_eqep.obj \
./28335_inits.obj \
./28335_mcbsp.obj \
./28335_pwm.obj \
./28335_serial.obj \
./28335_spi.obj \
./DSP2833x_ADC_cal.obj \
./DSP2833x_Adc.obj \
./DSP2833x_CSMPasswords.obj \
./DSP2833x_CodeStartBranch.obj \
./DSP2833x_CpuTimers.obj \
./DSP2833x_GlobalVariableDefs.obj \
./DSP2833x_Mcbsp.obj \
./DSP2833x_PieCtrl.obj \
./DSP2833x_Spi.obj \
./DSP2833x_SysCtrl.obj \
./DSP2833x_usDelay.obj \
./i2c.obj \
./lcd.obj \
./user_Lab5.obj 

ASM_DEPS += \
./DSP2833x_ADC_cal.d \
./DSP2833x_CSMPasswords.d \
./DSP2833x_CodeStartBranch.d \
./DSP2833x_usDelay.d 

GEN_MISC_DIRS__QUOTED += \
"configPkg\" 

OBJS__QUOTED += \
"28335_dma.obj" \
"28335_eqep.obj" \
"28335_inits.obj" \
"28335_mcbsp.obj" \
"28335_pwm.obj" \
"28335_serial.obj" \
"28335_spi.obj" \
"DSP2833x_ADC_cal.obj" \
"DSP2833x_Adc.obj" \
"DSP2833x_CSMPasswords.obj" \
"DSP2833x_CodeStartBranch.obj" \
"DSP2833x_CpuTimers.obj" \
"DSP2833x_GlobalVariableDefs.obj" \
"DSP2833x_Mcbsp.obj" \
"DSP2833x_PieCtrl.obj" \
"DSP2833x_Spi.obj" \
"DSP2833x_SysCtrl.obj" \
"DSP2833x_usDelay.obj" \
"i2c.obj" \
"lcd.obj" \
"user_Lab5.obj" 

C_DEPS__QUOTED += \
"28335_dma.d" \
"28335_eqep.d" \
"28335_inits.d" \
"28335_mcbsp.d" \
"28335_pwm.d" \
"28335_serial.d" \
"28335_spi.d" \
"DSP2833x_Adc.d" \
"DSP2833x_CpuTimers.d" \
"DSP2833x_GlobalVariableDefs.d" \
"DSP2833x_Mcbsp.d" \
"DSP2833x_PieCtrl.d" \
"DSP2833x_Spi.d" \
"DSP2833x_SysCtrl.d" \
"i2c.d" \
"lcd.d" \
"user_Lab5.d" 

GEN_FILES__QUOTED += \
"configPkg\linker.cmd" \
"configPkg\compiler.opt" 

ASM_DEPS__QUOTED += \
"DSP2833x_ADC_cal.d" \
"DSP2833x_CSMPasswords.d" \
"DSP2833x_CodeStartBranch.d" \
"DSP2833x_usDelay.d" 

C_SRCS__QUOTED += \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/28335_dma.c" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/28335_eqep.c" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/28335_inits.c" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/28335_mcbsp.c" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/28335_pwm.c" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/28335_serial.c" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/28335_spi.c" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_Adc.c" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_CpuTimers.c" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_headers/source/DSP2833x_GlobalVariableDefs.c" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_Mcbsp.c" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_PieCtrl.c" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_Spi.c" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_SysCtrl.c" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/i2c.c" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/lcd.c" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/source/user_Lab5.c" 

ASM_SRCS__QUOTED += \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_ADC_cal.asm" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_CSMPasswords.asm" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_CodeStartBranch.asm" \
"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab5/v110/DSP2833x_common/source/DSP2833x_usDelay.asm" 


