# Makefile at top of ipac support tree
# $Id: Makefile,v 1.4 2003-06-02 20:12:42 anj Exp $

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

include $(TOP)/configure/RULES_TOP
