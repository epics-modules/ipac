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

SRCS += devAiCan.c
SRCS += devAoCan.c
SRCS += devBiCan.c
SRCS += devBoCan.c
SRCS += devMbbiCan.c
SRCS += devMbboCan.c
SRCS += devMbbiDirectCan.c
SRCS += devMbboDirectCan.c
SRCS += devSiWiener.c
SRCS += devBiTip810.c
SRCS += drvTip810.c

PROD_IOC_vxWorks = tip810Lib

include $(TOP)/configure/RULES
