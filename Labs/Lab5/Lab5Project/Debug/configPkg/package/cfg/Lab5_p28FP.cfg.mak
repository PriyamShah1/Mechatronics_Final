# invoke SourceDir generated makefile for Lab5.p28FP
Lab5.p28FP: .libraries,Lab5.p28FP
.libraries,Lab5.p28FP: package/cfg/Lab5_p28FP.xdl
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\Lab5\SYSBIOS/src/makefile.libs

clean::
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\Lab5\SYSBIOS/src/makefile.libs clean

