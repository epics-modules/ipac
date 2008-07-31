# Makefile at top of ipac support tree
# $Id: Makefile,v 1.5 2008-07-31 04:59:47 anj Exp $

TOP = .
include $(TOP)/configure/CONFIG

# Different sites may need to able to select which ipac module drivers
# are to be built, thus there is no wildcard for DIRS.  Sites may
# comment out any DIRS lines below which are not required. Additional
# lines will need to be added as new drivers are installed.

DIRS := configure
DIRS += drvIpac
DIRS += drvTip810
DIRS += tyGSOctal

drvIpac_DEPEND_DIRS = configure
drvTip810_DEPEND_DIRS = drvIpac
tyGSOctal_DEPEND_DIRS = drvIpac

include $(TOP)/configure/RULES_TOP
