#Makefile at top of ipac support tree
TOP = .
include $(TOP)/config/CONFIG_APP

# Note this is different to most Makefiles at this level because 
# different sites may need to able to select which ipac module drivers
# are to be included, thus there is no wildcard for DIRS. Sites may
# comment out any DIRS lines below which are not required. Additional
# drivers will need to be added as they are installed.

DIRS += drvIpac
DIRS += drvTip810

include $(TOP)/config/RULES_TOP
