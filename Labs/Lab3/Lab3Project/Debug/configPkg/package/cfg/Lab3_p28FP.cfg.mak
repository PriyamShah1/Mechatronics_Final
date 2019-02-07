# invoke SourceDir generated makefile for Lab3.p28FP
Lab3.p28FP: .libraries,Lab3.p28FP
.libraries,Lab3.p28FP: package/cfg/Lab3_p28FP.xdl
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\Lab3\SYSBIOS/src/makefile.libs

clean::
	$(MAKE) -f C:\hzhou39_decamar2_ashwina2\SE423Repo\Labs\Lab3\SYSBIOS/src/makefile.libs clean

