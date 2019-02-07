################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
COECSL_edma3.obj: C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/source/COECSL_edma3.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/bin/cl6x" -mv6740 -O3 --opt_for_speed=5 --include_path="../../" --include_path="configPkg/package/cfg/" --include_path="../../../../LabFiles/sharedmem_com" --include_path="../../../../LabFiles/mcbsp_com" --include_path="../../include" --include_path="../../../../LabFiles/bsl_forSYSBIOS/inc" --include_path="C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/Lab1BIOSProject" --include_path="C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/include" --define=RUNNING_ON_OMAPL138 --define=PHILIPSCOLORLCD --define=omapl138 -g --diag_warning=225 --diag_wrap=off --display_error_number --auto_inline=0 --preproc_with_compile --preproc_dependency="COECSL_edma3.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

COECSL_mcbsp.obj: C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/source/COECSL_mcbsp.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/bin/cl6x" -mv6740 -O3 --opt_for_speed=5 --include_path="../../" --include_path="configPkg/package/cfg/" --include_path="../../../../LabFiles/sharedmem_com" --include_path="../../../../LabFiles/mcbsp_com" --include_path="../../include" --include_path="../../../../LabFiles/bsl_forSYSBIOS/inc" --include_path="C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/Lab1BIOSProject" --include_path="C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/include" --define=RUNNING_ON_OMAPL138 --define=PHILIPSCOLORLCD --define=omapl138 -g --diag_warning=225 --diag_wrap=off --display_error_number --auto_inline=0 --preproc_with_compile --preproc_dependency="COECSL_mcbsp.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

ColorLCD.obj: C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/source/ColorLCD.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/bin/cl6x" -mv6740 -O3 --opt_for_speed=5 --include_path="../../" --include_path="configPkg/package/cfg/" --include_path="../../../../LabFiles/sharedmem_com" --include_path="../../../../LabFiles/mcbsp_com" --include_path="../../include" --include_path="../../../../LabFiles/bsl_forSYSBIOS/inc" --include_path="C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/Lab1BIOSProject" --include_path="C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/include" --define=RUNNING_ON_OMAPL138 --define=PHILIPSCOLORLCD --define=omapl138 -g --diag_warning=225 --diag_wrap=off --display_error_number --auto_inline=0 --preproc_with_compile --preproc_dependency="ColorLCD.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

ColorVision.obj: C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/source/ColorVision.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/bin/cl6x" -mv6740 -O3 --opt_for_speed=5 --include_path="../../" --include_path="configPkg/package/cfg/" --include_path="../../../../LabFiles/sharedmem_com" --include_path="../../../../LabFiles/mcbsp_com" --include_path="../../include" --include_path="../../../../LabFiles/bsl_forSYSBIOS/inc" --include_path="C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/Lab1BIOSProject" --include_path="C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/include" --define=RUNNING_ON_OMAPL138 --define=PHILIPSCOLORLCD --define=omapl138 -g --diag_warning=225 --diag_wrap=off --display_error_number --auto_inline=0 --preproc_with_compile --preproc_dependency="ColorVision.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

LCDprintf.obj: C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/source/LCDprintf.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/bin/cl6x" -mv6740 -O3 --opt_for_speed=5 --include_path="../../" --include_path="configPkg/package/cfg/" --include_path="../../../../LabFiles/sharedmem_com" --include_path="../../../../LabFiles/mcbsp_com" --include_path="../../include" --include_path="../../../../LabFiles/bsl_forSYSBIOS/inc" --include_path="C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/Lab1BIOSProject" --include_path="C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/include" --define=RUNNING_ON_OMAPL138 --define=PHILIPSCOLORLCD --define=omapl138 -g --diag_warning=225 --diag_wrap=off --display_error_number --auto_inline=0 --preproc_with_compile --preproc_dependency="LCDprintf.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-734493073:
	@$(MAKE) --no-print-directory -Onone -f subdir_rules.mk build-734493073-inproc

build-734493073-inproc: C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/SYSBIOS/Lab1BIOS.cfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: XDCtools'
	"C:/CCStudio_v8/xdctools_3_50_05_12_core/xs" --xdcpath="C:/CCStudio_v8/bios_6_70_01_03/packages;C:/CCStudio_v8/ccsv8/ccs_base;" xdc.tools.configuro -o configPkg -t ti.targets.elf.C674 -p ti.platforms.evmOMAPL138 -r debug -c "C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2" --compileOptions "-g" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

configPkg/linker.cmd: build-734493073 C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/SYSBIOS/Lab1BIOS.cfg
configPkg/compiler.opt: build-734493073
configPkg/: build-734493073

Ladar.obj: C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/source/Ladar.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/bin/cl6x" -mv6740 -O3 --opt_for_speed=5 --include_path="../../" --include_path="configPkg/package/cfg/" --include_path="../../../../LabFiles/sharedmem_com" --include_path="../../../../LabFiles/mcbsp_com" --include_path="../../include" --include_path="../../../../LabFiles/bsl_forSYSBIOS/inc" --include_path="C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/Lab1BIOSProject" --include_path="C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/include" --define=RUNNING_ON_OMAPL138 --define=PHILIPSCOLORLCD --define=omapl138 -g --diag_warning=225 --diag_wrap=off --display_error_number --auto_inline=0 --preproc_with_compile --preproc_dependency="Ladar.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

MatrixMath.obj: C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/source/MatrixMath.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/bin/cl6x" -mv6740 -O3 --opt_for_speed=5 --include_path="../../" --include_path="configPkg/package/cfg/" --include_path="../../../../LabFiles/sharedmem_com" --include_path="../../../../LabFiles/mcbsp_com" --include_path="../../include" --include_path="../../../../LabFiles/bsl_forSYSBIOS/inc" --include_path="C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/Lab1BIOSProject" --include_path="C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/include" --define=RUNNING_ON_OMAPL138 --define=PHILIPSCOLORLCD --define=omapl138 -g --diag_warning=225 --diag_wrap=off --display_error_number --auto_inline=0 --preproc_with_compile --preproc_dependency="MatrixMath.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

pru.obj: C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/source/pru.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/bin/cl6x" -mv6740 -O3 --opt_for_speed=5 --include_path="../../" --include_path="configPkg/package/cfg/" --include_path="../../../../LabFiles/sharedmem_com" --include_path="../../../../LabFiles/mcbsp_com" --include_path="../../include" --include_path="../../../../LabFiles/bsl_forSYSBIOS/inc" --include_path="C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/Lab1BIOSProject" --include_path="C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/include" --define=RUNNING_ON_OMAPL138 --define=PHILIPSCOLORLCD --define=omapl138 -g --diag_warning=225 --diag_wrap=off --display_error_number --auto_inline=0 --preproc_with_compile --preproc_dependency="pru.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

user_Lab1BIOS.obj: C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/source/user_Lab1BIOS.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/bin/cl6x" -mv6740 -O3 --opt_for_speed=5 --include_path="../../" --include_path="configPkg/package/cfg/" --include_path="../../../../LabFiles/sharedmem_com" --include_path="../../../../LabFiles/mcbsp_com" --include_path="../../include" --include_path="../../../../LabFiles/bsl_forSYSBIOS/inc" --include_path="C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/Lab1BIOSProject" --include_path="C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/include" --define=RUNNING_ON_OMAPL138 --define=PHILIPSCOLORLCD --define=omapl138 -g --diag_warning=225 --diag_wrap=off --display_error_number --auto_inline=0 --preproc_with_compile --preproc_dependency="user_Lab1BIOS.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

user_xy.obj: C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/source/user_xy.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/bin/cl6x" -mv6740 -O3 --opt_for_speed=5 --include_path="../../" --include_path="configPkg/package/cfg/" --include_path="../../../../LabFiles/sharedmem_com" --include_path="../../../../LabFiles/mcbsp_com" --include_path="../../include" --include_path="../../../../LabFiles/bsl_forSYSBIOS/inc" --include_path="C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/Lab1BIOSProject" --include_path="C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2/include" --define=RUNNING_ON_OMAPL138 --define=PHILIPSCOLORLCD --define=omapl138 -g --diag_warning=225 --diag_wrap=off --display_error_number --auto_inline=0 --preproc_with_compile --preproc_dependency="user_xy.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


