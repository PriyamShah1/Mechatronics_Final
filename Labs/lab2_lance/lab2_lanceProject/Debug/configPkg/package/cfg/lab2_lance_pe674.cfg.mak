# invoke SourceDir generated makefile for lab2_lance.pe674
lab2_lance.pe674: .libraries,lab2_lance.pe674
.libraries,lab2_lance.pe674: package/cfg/lab2_lance_pe674.xdl
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\lab2_lance\SYSBIOS/src/makefile.libs

clean::
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\lab2_lance\SYSBIOS/src/makefile.libs clean

