# Makefile
TOP = ..
include $(TOP)/configure/CONFIG

HTMLS_DIR = .
HTMLS += tyGSOctal.html
HTMLS += tyGSOctalRelease.html

DBD += tyGSOctal.dbd

LIBSRCS_vxWorks += tyGSOctal.c
LIBSRCS_RTEMS += tyGSOctal_RTEMS.c

LIBRARY_IOC_vxWorks = TyGSOctal
LIBRARY_IOC_RTEMS = TyGSOctal
TyGSOctal_LIBS += Ipac

include $(TOP)/configure/RULES
