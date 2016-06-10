TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC_Linux += tpxDetector
PROD_IOC_Linux += tpxDetectorApp

USR_INCLUDES += -I$(JAVA_HOME)/include
USR_INCLUDES += -I$(JAVA_HOME)/include/linux
USR_INCLUDES += -I$(JAVA_HOME)/bin
USR_INCLUDES += -I$(JAVA_HOME)/jre/lib/amd64
USR_INCLUDES += -I$(RELAXD)/src
USR_INCLUDES += -I$(RELAXD)/common
USR_INCLUDES += -I$(BOOST_INSTALL)
# USR_INCLUDES += -I$(SOPHY_HOME)/lib/common

USR_LDFLAGS_Linux += -L$(JAVA_HOME)/jre/lib/amd64/server
USR_CPPFLAGS_Linux += -g
USR_SYS_LIBS_Linux  += jvm

USR_LDFLAGS_Linux += -L$(BOOST_INSTALL)/stage/lib
USR_LDFLAGS_Linux += -L$(RELAXD)/Debug/static

USR_SYS_LIBS_Linux  += mpxhwrelaxd
USR_SYS_LIBS_Linux  += boost_system
USR_SYS_LIBS_Linux  += boost_thread
USR_SYS_LIBS_Linux  += boost_chrono

LIB_SRCS += tpxDetector.cpp

# <name>.dbd will be created from <name>Include.dbd
DBD += tpxDetectorApp.dbd
DBD += tpxDetectorSupport.dbd

LIBS += devIocStats
LIBS += caPutLog

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PROD_SRCS += tpxDetectorApp_registerRecordDeviceDriver.cpp tpxDetectorAppMain.cpp

# Add locally compiled object code
PROD_LIBS += tpxDetector

include $(TOP)/ADApp/commonDriverMakefile
#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

