# Makefile at top of ipac support tree
# $Id: Makefile,v 1.6 2008-08-30 04:59:59 anj Exp $

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
