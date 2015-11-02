# Makefile
TOP = ..
include $(TOP)/configure/CONFIG

HTMLS_DIR = .
HTMLS += IP520.html
HTMLS += IP520Release.html

DBD += IP520.dbd

INC += IP520Ext.h

LIBSRCS_vxWorks += ip520.c

LIBRARY_IOC_vxWorks = IP520
IP520_LIBS += Ipac

include $(TOP)/configure/RULES
