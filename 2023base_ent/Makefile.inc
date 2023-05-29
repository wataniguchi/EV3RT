mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_COBJS +=

APPL_CXXOBJS += \
SRLF.o \
FIR.o \
FilteredMotor.o \
FilteredColorSensor.o \
Plotter.o \
PIDcalculator.o \

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif

APPL_DIRS += $(mkfile_path)app $(mkfile_path)unit


INCLUDES += \
	-I$(mkfile_path)app \
	-I$(mkfile_path)unit \
	-I$(ETROBO_HRP3_WORKSPACE)/etroboc_common

#COPTS += -fno-use-cxa-atexit
#COPTS += -DNDEBUG -std=gnu++11
COPTS += -std=gnu++11 $(USER_COPTS)