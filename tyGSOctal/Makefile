# Makefile
TOP = ..
include $(TOP)/configure/CONFIG

HTMLS_DIR = .
HTMLS += tyGSOctal.html
HTMLS += tyGSOctalRelease.html

DBD += tyGSOctal.dbd

LIBSRCS += tyGSOctal.c

LIBRARY_IOC_vxWorks = TyGSOctal
TyGSOctal_LIBS += Ipac

include $(TOP)/configure/RULES
