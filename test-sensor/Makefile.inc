mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_COBJS +=

APPL_CXXOBJS += FIR.o FilteredColorSensor.o

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif

APPL_DIRS += $(mkfile_path)app $(mkfile_path)unit

INCLUDES += \
	-I$(mkfile_path)app \
	-I$(mkfile_path)unit

APPL_LIBS += -lm

COPTS += -std=gnu++14

ifdef ETROBO_WITH_FILTER
COPTS += -DWITH_FILTER
endif

#COPTS += -fno-use-cxa-atexit
#COPTS += -DNDEBUG
COPTS += -g $(USER_COPTS)