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

# Any VMEbus: GreenSpring VIPC carrier drivers
LIBSRCS += drvVipc310.c
LIBSRCS += drvVipc610.c
LIBSRCS += drvVipc616.c

# MVME162 & MVME172: IPchip carrier driver
LIBSRCS += drvIpMv162.c

# ISAbus: GreenSpring ATC40 carrier driver (Intel-based systems only!)
LIBSRCS += drvAtc40.c

include $(TOP)/configure/RULES
