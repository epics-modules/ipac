# Makefile at top of ipac support tree
# $Id: Makefile,v 1.3 1999-03-09 20:28:34 anj Exp $

TOP = .
include $(TOP)/config/CONFIG_APP

# Note this is different to most Makefiles at this level because 
# different sites may need to able to select which ipac module drivers
# are to be included, thus there is no wildcard for DIRS. Sites may
# comment out any DIRS lines below which are not required. Additional
# lines will need to be added as new drivers are installed.

DIRS += drvIpac
DIRS += drvTip810
DIRS += tyGSOctal

include $(TOP)/config/RULES_TOP
