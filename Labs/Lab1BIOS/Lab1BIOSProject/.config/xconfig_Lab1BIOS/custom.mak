## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,e674 linker.cmd package/cfg/Lab1BIOS_pe674.oe674

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/Lab1BIOS_pe674.xdl
	$(SED) 's"^\"\(package/cfg/Lab1BIOS_pe674cfg.cmd\)\"$""\"C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/Lab1BIOS/Lab1BIOSProject/.config/xconfig_Lab1BIOS/\1\""' package/cfg/Lab1BIOS_pe674.xdl > $@
	-$(SETDATE) -r:max package/cfg/Lab1BIOS_pe674.h compiler.opt compiler.opt.defs
