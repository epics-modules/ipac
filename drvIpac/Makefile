# Makefile
TOP = ..
include $(TOP)/configure/CONFIG

DBD += drvIpac.dbd

INC_vxWorks += drvIpac.h

HTMLS_DIR = .
HTMLS += index.html
HTMLS += logo101.gif
HTMLS += drvIpac.html
HTMLS += ipacRelease.html

LIBRARY_IOC_vxWorks = Ipac

LIBSRCS += drvIpac.c

# VMEbus: GreenSpring VIPC carrier drivers:
LIBSRCS += drvVipc310.c
LIBSRCS += drvVipc610.c
LIBSRCS += drvVipc616.c

# MVME162 & MVME172: IPchip carrier driver:
LIBSRCS_vxWorks-68040 += drvIpMv162.c
LIBSRCS_vxWorks-68040lc += drvIpMv162.c

# GreenSpring ATC40 Carrier driver: for ISA bus systems only.
LIBSRCS_vxWorks-pentium += drvAtc40.c

# Normally you add filenames to LIBSRCS_<arch-class>, but this
# line makes the above LIBSRCS_<target-arch> stuff work:
LIBSRCS += $(LIBSRCS_$(T_A))

include $(TOP)/configure/RULES
