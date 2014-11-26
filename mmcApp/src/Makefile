# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
# ADD MACRO DEFINITIONS AFTER THIS LINE

# The following are used for debugging messages.
USR_CXXFLAGS += -DDEBUG

DBDINC       += mmcRecord.h
DBDINC       += mmcaRecord.h

DBD          += mmcSupport.dbd

LIBRARY_IOC  += mmc

# The following are required for all mmc record configurations.
mmc_SRCS  += mmcRecord.cc mmcaRecord.cc

mmc_LIBS  += asyn
mmc_LIBS  += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
# ADD RULES AFTER THIS LINE
