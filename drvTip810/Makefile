# Makefile
TOP=..
include $(TOP)/configure/CONFIG

DBD += devTip810.dbd

INC_vxWorks += canBus.h
INC_vxWorks += drvTip810.h

HTMLS_DIR = .
HTMLS += devCan.html
HTMLS += drvTip810.html
HTMLS += canRelease.html

LIBSRCS += devAiCan.c
LIBSRCS += devAoCan.c
LIBSRCS += devBiCan.c
LIBSRCS += devBoCan.c
LIBSRCS += devMbbiCan.c
LIBSRCS += devMbboCan.c
LIBSRCS += devMbbiDirectCan.c
LIBSRCS += devMbboDirectCan.c
LIBSRCS += devSiWiener.c
LIBSRCS += devBiTip810.c
LIBSRCS += drvTip810.c

LIBRARY_IOC_vxWorks = Tip810
Tip810_LIBS += Ipac

include $(TOP)/configure/RULES
