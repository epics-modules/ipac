# Makefile
TOP=..
include $(TOP)/configure/CONFIG

DBD += devTip810.dbd

INC += canBus.h
INC += drvTip810.h

HTMLS_DIR = .
HTMLS += devCan.html
HTMLS += drvTip810.html
HTMLS += canRelease.html

Tip810_SRCS += devAiCan.c
Tip810_SRCS += devAoCan.c
Tip810_SRCS += devBiCan.c
Tip810_SRCS += devBoCan.c
Tip810_SRCS += devMbbiCan.c
Tip810_SRCS += devMbboCan.c
Tip810_SRCS += devMbbiDirectCan.c
Tip810_SRCS += devMbboDirectCan.c
Tip810_SRCS += devSiWiener.c
Tip810_SRCS += devBiTip810.c
Tip810_SRCS += drvTip810.c

USR_CFLAGS += -DUSE_TYPED_RSET -DUSE_TYPED_DSET -DUSE_TYPED_DRVET

LIBRARY_IOC_vxWorks = Tip810
LIBRARY_IOC_RTEMS = Tip810

Tip810_LIBS = Ipac $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
