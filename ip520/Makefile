# Makefile
TOP = ..
include $(TOP)/configure/CONFIG

DBD += IP520.dbd

INC += IP520Ext.h

LIBSRCS_vxWorks += ip520.c
#!LIBSRCS_RTEMS += IP520_RTEMS.c

LIBRARY_IOC_vxWorks = IP520
#!LIBRARY_IOC_RTEMS = IP520
IP520_LIBS += Ipac

include $(TOP)/configure/RULES
