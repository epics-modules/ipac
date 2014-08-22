# Makefile
TOP = ..
include $(TOP)/configure/CONFIG

DBD += drvIpac.dbd

INC += drvIpac.h

HTMLS_DIR = .
HTMLS += index.html
HTMLS += logo101.gif
HTMLS += drvIpac.html
HTMLS += ipacRelease.html

LIBRARY_IOC = Ipac

LIBSRCS += drvIpac.c

# Any VMEbus: VIPC/TVME/XVME carrier drivers
LIBSRCS += drvVipc310.c
LIBSRCS += drvVipc610.c
LIBSRCS += drvVipc616.c
LIBSRCS += drvTvme200.c
LIBSRCS += drvXy9660.c
LIBSRCS += drvHy8002.c

# MVME162 & MVME172: IPchip carrier driver (68k only)
LIBSRCS_vxWorks-68040 += drvIpMv162.c

# ISAbus: SBS ATC40 carrier driver (Intel only)
LIBSRCS_vxWorks-pentium += drvAtc40.c

LIBSRCS_vxWorks += $(LIBSRCS_$(T_A))

Ipac_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
