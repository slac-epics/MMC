TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------------------
# ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
# Optimization of db files using dbst (DEFAULT: NO)
# DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install)
# databases, templates, substitutions like this
DB_INSTALLS += mmc.db
DB_INSTALLS += mmca.db
DB += usbsn.db

#----------------------------------------------------
# Create and install (or just install)
# Archive files, templates, substitutions like this
ARCHIVE += mmca_archive.tpl-arch

#----------------------------------------------------
# Create and install (or just install)
# Autosave request files, templates, substitutions like this
REQ     += mmc_autosave.tpl-req
REQ     += mmca_autosave.tpl-req


include $(TOP)/configure/RULES
#----------------------------------------
# ADD RULES AFTER THIS LINE

