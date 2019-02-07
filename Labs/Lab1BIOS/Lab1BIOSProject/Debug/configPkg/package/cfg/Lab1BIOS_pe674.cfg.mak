# invoke SourceDir generated makefile for Lab1BIOS.pe674
Lab1BIOS.pe674: .libraries,Lab1BIOS.pe674
.libraries,Lab1BIOS.pe674: package/cfg/Lab1BIOS_pe674.xdl
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\Lab1BIOS\SYSBIOS/src/makefile.libs

clean::
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\Lab1BIOS\SYSBIOS/src/makefile.libs clean

