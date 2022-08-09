mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_COBJS +=

APPL_CXXOBJS += \
SRLF.o \
FIR.o \
FilteredMotor.o \
FilteredColorSensor.o \
Plotter.o \
PIDcalculator.o \
Profile.o \
Video.o \

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif

APPL_DIRS += $(mkfile_path)app $(mkfile_path)unit

ifdef ETROBO_WITH_OPENCV

INCLUDES += \
	-I$(mkfile_path)app \
	-I$(mkfile_path)unit \
	-I/usr/local/include/opencv4

APPL_LIBS += -lm -lX11 \
        -lopencv_gapi -lopencv_stitching -lopencv_alphamat -lopencv_aruco -lopencv_barcode \
        -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dnn_objdetect -lopencv_dnn_superres \
        -lopencv_dpm -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hdf \
        -lopencv_hfs -lopencv_img_hash -lopencv_intensity_transform -lopencv_line_descriptor -lopencv_mcc \
        -lopencv_quality -lopencv_rapid -lopencv_reg -lopencv_rgbd -lopencv_saliency \
        -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_superres -lopencv_optflow \
        -lopencv_surface_matching -lopencv_tracking -lopencv_highgui -lopencv_datasets -lopencv_text \
        -lopencv_plot -lopencv_videostab -lopencv_videoio -lopencv_viz -lopencv_wechat_qrcode \
        -lopencv_xfeatures2d -lopencv_shape -lopencv_ml -lopencv_ximgproc -lopencv_video \
        -lopencv_xobjdetect -lopencv_objdetect -lopencv_calib3d -lopencv_imgcodecs -lopencv_features2d \
        -lopencv_dnn -lopencv_flann -lopencv_xphoto -lopencv_photo -lopencv_imgproc -lopencv_core

COPTS += -DWITH_OPENCV

else # ifdef WITH_OPENCV

INCLUDES += \
	-I$(mkfile_path)app \
	-I$(mkfile_path)unit

APPL_LIBS += -lm -lX11 \

endif # ifdef WITH_OPENCV

#COPTS += -fno-use-cxa-atexit
#COPTS += -DNDEBUG -std=gnu++11
#
# DO NOT add -g option
# -g makes robot to loose control on motors when initializing OpenCV in some cases
#
COPTS += -std=gnu++14 $(USER_COPTS)