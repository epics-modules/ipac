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

Ipac_SRCS += drvIpac.c

# Any VMEbus: VIPC/TVME/XVME carrier drivers
Ipac_SRCS += drvVipc310.c
Ipac_SRCS += drvVipc610.c
Ipac_SRCS += drvVipc616.c
Ipac_SRCS += drvTvme200.c
Ipac_SRCS += drvXy9660.c
Ipac_SRCS += drvHy8002.c

# MVME162 & MVME172: IPchip carrier driver (68k only)
Ipac_SRCS_vxWorks-68040 += drvIpMv162.c

# ISAbus: SBS ATC40 carrier driver (Intel only)
Ipac_SRCS_vxWorks-pentium += drvAtc40.c

Ipac_SRCS_vxWorks += $(Ipac_SRCS_$(T_A))

USR_CFLAGS += -DUSE_TYPED_RSET -DUSE_TYPED_DSET -DUSE_TYPED_DRVET

Ipac_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
