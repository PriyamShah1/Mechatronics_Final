# invoke SourceDir generated makefile for lab1.pe674
lab1.pe674: .libraries,lab1.pe674
.libraries,lab1.pe674: package/cfg/lab1_pe674.xdl
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\lab1\SYSBIOS/src/makefile.libs

clean::
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\lab1\SYSBIOS/src/makefile.libs clean

