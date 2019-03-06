# invoke SourceDir generated makefile for Lab4.p28FP
Lab4.p28FP: .libraries,Lab4.p28FP
.libraries,Lab4.p28FP: package/cfg/Lab4_p28FP.xdl
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\Lab4\SYSBIOS/src/makefile.libs

clean::
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\Lab4\SYSBIOS/src/makefile.libs clean

