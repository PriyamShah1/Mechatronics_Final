#
#  Do not edit this file.  This file is generated from 
#  package.bld.  Any modifications to this file will be 
#  overwritten whenever makefiles are re-generated.
#

unexport MAKEFILE_LIST
MK_NOGENDEPS := $(filter clean,$(MAKECMDGOALS))
override PKGDIR = xconfig_Lab1BIOS
XDCINCS = -I. -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(XPKGPATH))))
XDCCFGDIR = package/cfg/

#
# The following dependencies ensure package.mak is rebuilt
# in the event that some included BOM script changes.
#
ifneq (clean,$(MAKECMDGOALS))
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/utils.js:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/utils.js
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/xdc.tci:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/xdc.tci
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/template.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/template.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/om2.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/om2.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/xmlgen.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/xmlgen.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/xmlgen2.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/xmlgen2.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/Warnings.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/Warnings.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/IPackage.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/IPackage.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/package.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/package.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/services/global/Clock.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/services/global/Clock.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/services/global/Trace.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/services/global/Trace.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/bld.js:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/bld.js
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/BuildEnvironment.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/BuildEnvironment.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/PackageContents.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/PackageContents.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/_gen.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/_gen.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/Library.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/Library.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/Executable.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/Executable.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/Repository.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/Repository.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/Configuration.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/Configuration.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/Script.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/Script.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/Manifest.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/Manifest.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/Utils.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/Utils.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/ITarget.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/ITarget.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/ITarget2.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/ITarget2.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/ITarget3.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/ITarget3.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/ITargetFilter.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/ITargetFilter.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/package.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/bld/package.xs
package.mak: config.bld
C:/CCStudio_v8/bios_6_70_01_03/packages/ti/targets/ITarget.xs:
package.mak: C:/CCStudio_v8/bios_6_70_01_03/packages/ti/targets/ITarget.xs
C:/CCStudio_v8/bios_6_70_01_03/packages/ti/targets/C28_large.xs:
package.mak: C:/CCStudio_v8/bios_6_70_01_03/packages/ti/targets/C28_large.xs
C:/CCStudio_v8/bios_6_70_01_03/packages/ti/targets/C28_float.xs:
package.mak: C:/CCStudio_v8/bios_6_70_01_03/packages/ti/targets/C28_float.xs
C:/CCStudio_v8/bios_6_70_01_03/packages/ti/targets/package.xs:
package.mak: C:/CCStudio_v8/bios_6_70_01_03/packages/ti/targets/package.xs
C:/CCStudio_v8/bios_6_70_01_03/packages/ti/targets/elf/ITarget.xs:
package.mak: C:/CCStudio_v8/bios_6_70_01_03/packages/ti/targets/elf/ITarget.xs
C:/CCStudio_v8/bios_6_70_01_03/packages/ti/targets/elf/C28_float.xs:
package.mak: C:/CCStudio_v8/bios_6_70_01_03/packages/ti/targets/elf/C28_float.xs
C:/CCStudio_v8/bios_6_70_01_03/packages/ti/targets/elf/package.xs:
package.mak: C:/CCStudio_v8/bios_6_70_01_03/packages/ti/targets/elf/package.xs
package.mak: package.bld
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/tools/configuro/template/compiler.opt.xdt:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/tools/configuro/template/compiler.opt.xdt
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/services/io/File.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/services/io/File.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/services/io/package.xs:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/services/io/package.xs
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/tools/configuro/template/compiler.defs.xdt:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/tools/configuro/template/compiler.defs.xdt
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/tools/configuro/template/custom.mak.exe.xdt:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/tools/configuro/template/custom.mak.exe.xdt
C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/tools/configuro/template/package.xs.xdt:
package.mak: C:/CCStudio_v8/xdctools_3_50_05_12_core/packages/xdc/tools/configuro/template/package.xs.xdt
endif

ti.targets.elf.C674.rootDir ?= C:/CCStudio_v8/ccsv8/tools/compiler/ti-cgt-c6000_8.2.2
ti.targets.elf.packageBase ?= C:/CCStudio_v8/bios_6_70_01_03/packages/ti/targets/elf/
.PRECIOUS: $(XDCCFGDIR)/%.oe674
.PHONY: all,e674 .dlls,e674 .executables,e674 test,e674
all,e674: .executables,e674
.executables,e674: .libraries,e674
.executables,e674: .dlls,e674
.dlls,e674: .libraries,e674
.libraries,e674: .interfaces
	@$(RM) $@
	@$(TOUCH) "$@"

.help::
	@$(ECHO) xdc test,e674
	@$(ECHO) xdc .executables,e674
	@$(ECHO) xdc .libraries,e674
	@$(ECHO) xdc .dlls,e674


all: .executables 
.executables: .libraries .dlls
.libraries: .interfaces

PKGCFGS := $(wildcard package.xs) package/build.cfg
.interfaces: package/package.xdc.inc package/package.defs.h package.xdc $(PKGCFGS)

-include package/package.xdc.dep
package/%.xdc.inc package/%_xconfig_Lab1BIOS.c package/%.defs.h: %.xdc $(PKGCFGS)
	@$(MSG) generating interfaces for package xconfig_Lab1BIOS" (because $@ is older than $(firstword $?))" ...
	$(XSRUN) -f xdc/services/intern/cmd/build.xs $(MK_IDLOPTS) -m package/package.xdc.dep -i package/package.xdc.inc package.xdc

.dlls,e674 .dlls: Lab1BIOS.pe674

-include package/cfg/Lab1BIOS_pe674.mak
-include package/cfg/Lab1BIOS_pe674.cfg.mak
ifeq (,$(MK_NOGENDEPS))
-include package/cfg/Lab1BIOS_pe674.dep
endif
Lab1BIOS.pe674: package/cfg/Lab1BIOS_pe674.xdl
	@


ifeq (,$(wildcard .libraries,e674))
Lab1BIOS.pe674 package/cfg/Lab1BIOS_pe674.c: .libraries,e674
endif

package/cfg/Lab1BIOS_pe674.c package/cfg/Lab1BIOS_pe674.h package/cfg/Lab1BIOS_pe674.xdl: override _PROG_NAME := Lab1BIOS.xe674
package/cfg/Lab1BIOS_pe674.c: package/cfg/Lab1BIOS_pe674.cfg
package/cfg/Lab1BIOS_pe674.xdc.inc: package/cfg/Lab1BIOS_pe674.xdl
package/cfg/Lab1BIOS_pe674.xdl package/cfg/Lab1BIOS_pe674.c: .interfaces

clean:: clean,e674
	-$(RM) package/cfg/Lab1BIOS_pe674.cfg
	-$(RM) package/cfg/Lab1BIOS_pe674.dep
	-$(RM) package/cfg/Lab1BIOS_pe674.c
	-$(RM) package/cfg/Lab1BIOS_pe674.xdc.inc

clean,e674::
	-$(RM) Lab1BIOS.pe674
.executables,e674 .executables: Lab1BIOS.xe674

Lab1BIOS.xe674: |Lab1BIOS.pe674

-include package/cfg/Lab1BIOS.xe674.mak
Lab1BIOS.xe674: package/cfg/Lab1BIOS_pe674.oe674 
	$(RM) $@
	@$(MSG) lnke674 $@ ...
	$(RM) $(XDCCFGDIR)/$@.map
	$(ti.targets.elf.C674.rootDir)/bin/cl6x -fs $(XDCCFGDIR)$(dir $@). -q -u _c_int00 --abi=eabi -z  -o $@ package/cfg/Lab1BIOS_pe674.oe674   package/cfg/Lab1BIOS_pe674.xdl  -w -c -m $(XDCCFGDIR)/$@.map -l $(ti.targets.elf.C674.rootDir)/lib/libc.a
	
Lab1BIOS.xe674: export C_DIR=
Lab1BIOS.xe674: PATH:=$(ti.targets.elf.C674.rootDir)/bin/;$(PATH)
Lab1BIOS.xe674: Path:=$(ti.targets.elf.C674.rootDir)/bin/;$(PATH)

Lab1BIOS.test test,e674 test: Lab1BIOS.xe674.test

Lab1BIOS.xe674.test:: Lab1BIOS.xe674
ifeq (,$(_TESTLEVEL))
	@$(MAKE) -R -r --no-print-directory -f $(XDCROOT)/packages/xdc/bld/xdc.mak _TESTLEVEL=1 Lab1BIOS.xe674.test
else
	@$(MSG) running $<  ...
	$(call EXEC.Lab1BIOS.xe674, ) 
endif

clean,e674::
	-$(RM) $(wildcard .tmp,Lab1BIOS.xe674,*)


clean:: clean,e674

clean,e674::
	-$(RM) Lab1BIOS.xe674
%,copy:
	@$(if $<,,$(MSG) don\'t know how to build $*; exit 1)
	@$(MSG) cp $< $@
	$(RM) $@
	$(CP) $< $@
Lab1BIOS_pe674.oe674,copy : package/cfg/Lab1BIOS_pe674.oe674
Lab1BIOS_pe674.se674,copy : package/cfg/Lab1BIOS_pe674.se674

$(XDCCFGDIR)%.c $(XDCCFGDIR)%.h $(XDCCFGDIR)%.xdl: $(XDCCFGDIR)%.cfg $(XDCROOT)/packages/xdc/cfg/Main.xs | .interfaces
	@$(MSG) "configuring $(_PROG_NAME) from $< ..."
	$(CONFIG) $(_PROG_XSOPTS) xdc.cfg $(_PROG_NAME) $(XDCCFGDIR)$*.cfg $(XDCCFGDIR)$*

.PHONY: release,xconfig_Lab1BIOS
ifeq (,$(MK_NOGENDEPS))
-include package/rel/xconfig_Lab1BIOS.tar.dep
endif
package/rel/xconfig_Lab1BIOS/xconfig_Lab1BIOS/package/package.rel.xml: package/package.bld.xml
package/rel/xconfig_Lab1BIOS/xconfig_Lab1BIOS/package/package.rel.xml: package/build.cfg
package/rel/xconfig_Lab1BIOS/xconfig_Lab1BIOS/package/package.rel.xml: package/package.xdc.inc
package/rel/xconfig_Lab1BIOS/xconfig_Lab1BIOS/package/package.rel.xml: .force
	@$(MSG) generating external release references $@ ...
	$(XS) $(JSENV) -f $(XDCROOT)/packages/xdc/bld/rel.js $(MK_RELOPTS) . $@

xconfig_Lab1BIOS.tar: package/rel/xconfig_Lab1BIOS.xdc.inc package/rel/xconfig_Lab1BIOS/xconfig_Lab1BIOS/package/package.rel.xml
	@$(MSG) making release file $@ "(because of $(firstword $?))" ...
	-$(RM) $@
	$(call MKRELTAR,package/rel/xconfig_Lab1BIOS.xdc.inc,package/rel/xconfig_Lab1BIOS.tar.dep)


release release,xconfig_Lab1BIOS: all xconfig_Lab1BIOS.tar
clean:: .clean
	-$(RM) xconfig_Lab1BIOS.tar
	-$(RM) package/rel/xconfig_Lab1BIOS.xdc.inc
	-$(RM) package/rel/xconfig_Lab1BIOS.tar.dep

clean:: .clean
	-$(RM) .libraries $(wildcard .libraries,*)
clean:: 
	-$(RM) .dlls $(wildcard .dlls,*)
#
# The following clean rule removes user specified
# generated files or directories.
#

ifneq (clean,$(MAKECMDGOALS))
ifeq (,$(wildcard package))
    $(shell $(MKDIR) package)
endif
ifeq (,$(wildcard package/cfg))
    $(shell $(MKDIR) package/cfg)
endif
ifeq (,$(wildcard package/lib))
    $(shell $(MKDIR) package/lib)
endif
ifeq (,$(wildcard package/rel))
    $(shell $(MKDIR) package/rel)
endif
ifeq (,$(wildcard package/internal))
    $(shell $(MKDIR) package/internal)
endif
endif
clean::
	-$(RMDIR) package

include custom.mak
