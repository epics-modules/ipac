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

PROD_IOC_vxWorks = ipacLib

SRCS += drvIpac.c

# VMEbus: GreenSpring VIPC carrier drivers:
SRCS += drvVipc310.c
SRCS += drvVipc610.c
SRCS += drvVipc610_01.c
SRCS += drvVipc616.c
SRCS += drvVipc616_01.c

# MVME162 & MVME172: IPchip carrier driver:
SRCS_vxWorks-68040 += drvIpMv162.c
SRCS_vxWorks-68040lc += drvIpMv162.c

# GreenSpring ATC40 Carrier driver: for ISA bus systems only.
SRCS_vxWorks-pentium += drvAtc40.c

SRCS += $(SRCS_$(T_A))

include $(TOP)/configure/RULES
