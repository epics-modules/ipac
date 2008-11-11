# Makefile at top of ipac support tree

TOP = .
include $(TOP)/configure/CONFIG

# Different sites may need to able to select which ipac module drivers
# are to be built, thus there is no wildcard for DIRS.  Sites may
# comment out any DIRS lines below which are not required.

DIRS := configure

DIRS += drvIpac
drvIpac_DEPEND_DIRS = configure

DIRS += drvTip810
drvTip810_DEPEND_DIRS = drvIpac

DIRS += tyGSOctal
tyGSOctal_DEPEND_DIRS = drvIpac

include $(TOP)/configure/RULES_TOP
