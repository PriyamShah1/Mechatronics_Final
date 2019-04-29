# invoke SourceDir generated makefile for Lab7.pe674
Lab7.pe674: .libraries,Lab7.pe674
.libraries,Lab7.pe674: package/cfg/Lab7_pe674.xdl
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\Lab7\SYSBIOS/src/makefile.libs

clean::
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\Lab7\SYSBIOS/src/makefile.libs clean

