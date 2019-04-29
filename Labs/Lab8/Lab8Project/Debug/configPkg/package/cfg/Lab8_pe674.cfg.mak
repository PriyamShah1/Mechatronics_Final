# invoke SourceDir generated makefile for Lab8.pe674
Lab8.pe674: .libraries,Lab8.pe674
.libraries,Lab8.pe674: package/cfg/Lab8_pe674.xdl
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\Lab8\SYSBIOS/src/makefile.libs

clean::
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\Lab8\SYSBIOS/src/makefile.libs clean

