#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/CCStudio_v8/bios_6_70_01_03/packages;C:/CCStudio_v8/ccsv8/ccs_base;C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/lab2_prob/lab2_probProject/.config
override XDCROOT = C:/CCStudio_v8/xdctools_3_50_05_12_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/CCStudio_v8/bios_6_70_01_03/packages;C:/CCStudio_v8/ccsv8/ccs_base;C:/hzhou39_decamar2_ashwina2/SE423Repo/Labs/lab2_prob/lab2_probProject/.config;C:/CCStudio_v8/xdctools_3_50_05_12_core/packages;..
HOSTOS = Windows
endif
