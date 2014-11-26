# Makefile at top of application tree
# "#!" marks lines that can be uncommented.
TOP = .
include $(TOP)/configure/CONFIG
DIRS := $(DIRS) $(filter-out $(DIRS), configure)
DIRS := $(DIRS) $(filter-out $(DIRS), mmcApp)
include $(TOP)/configure/RULES_TOP
