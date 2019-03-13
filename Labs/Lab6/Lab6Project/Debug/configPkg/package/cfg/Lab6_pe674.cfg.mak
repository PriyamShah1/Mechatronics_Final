# invoke SourceDir generated makefile for Lab6.pe674
Lab6.pe674: .libraries,Lab6.pe674
.libraries,Lab6.pe674: package/cfg/Lab6_pe674.xdl
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\Lab6\SYSBIOS/src/makefile.libs

clean::
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\Lab6\SYSBIOS/src/makefile.libs clean

