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
MYSRCS_68k += drvIpMv162.c

# GreenSpring ATC40 Carrier driver: for ISA bus systems only.
MYSRCS_pcPertium += drvAtc40.c

LIBSRCS_vxWorks +=  $(MYSRCS_$(ARCH_CLASS))

LIBSRCS += $(SRCS_$(T_A))

include $(TOP)/configure/RULES
